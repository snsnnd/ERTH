#include "sensor.h"
#include "mpu6050.h"
#include "motor.h"
#include "BT.h"

/***************************  编码器  ********************************/
void sensor_encoder(ENCODER_SensorTypeDef *encoder)
{
    encoder->lf_speed = encoder_readspeed(encoder_front_left); // 返回m/s
    encoder->rf_speed = encoder_readspeed(encoder_front_right);
    encoder->lb_speed = encoder_readspeed(encoder_back_left);
    encoder->rb_speed = encoder_readspeed(encoder_back_right);
#if DEBUG_PRINT_VERBOSE
    log_print("sensor->Linear velocity:\n");
    log_print("lf:%f,rf:%f,lb:%f,rb:%f\n", encoder->lf_speed, encoder->rf_speed, encoder->lb_speed, encoder->rb_speed);
#endif
}

/***************************  MPU  ********************************/
bool sensor_mpu_raw(MPU_SensorTypeDef *mpu)
{
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;

    if (mpu == NULL ||
        !mpu6050_get_acc(&acc_x, &acc_y, &acc_z) ||
        !mpu6050_get_gyro(&gyro_x, &gyro_y, &gyro_z))
    {
        return false;
    }

    mpu->RAW.acc_raw_x = acc_x;
    mpu->RAW.acc_raw_y = acc_y;
    mpu->RAW.acc_raw_z = acc_z;
    mpu->RAW.gyro_raw_x = gyro_x;
    mpu->RAW.gyro_raw_y = gyro_y;
    mpu->RAW.gyro_raw_z = gyro_z;
#if DEBUG_PRINT_VERBOSE
    log_print("acc:\nx:%f,y:%f,z:%f\ngyro:\nx:%f,y:%f,z:%f\n", mpu->RAW.acc_raw_x, mpu->RAW.acc_raw_y, mpu->RAW.acc_raw_z, mpu->RAW.gyro_raw_x, mpu->RAW.gyro_raw_y, mpu->RAW.gyro_raw_z);
#endif
    return true;
}

bool sensor_mpu_tra(MPU_SensorTypeDef *mpu)
{
    // 若你希望每次都先刷新原始值，再做换算，用下面两行
    if (mpu == NULL || !sensor_mpu_raw(mpu))
    {
        return false;
    }
    sensor_mpu_acc_tra(mpu);
    sensor_mpu_gyro_tra(mpu);
#if DEBUG_PRINT_VERBOSE
    log_print("acc_tra:\tx:%f\ty%f\tz%f\n", mpu->TRA.acc_x, mpu->TRA.acc_y, mpu->TRA.acc_z);
    log_print("gyro_tra:\tx:%f\ty%f\tz%f\n", mpu->TRA.gyro_x, mpu->TRA.gyro_y, mpu->TRA.gyro_z);
#endif
    return true;
}

bool sensor_mpu_acc_raw(MPU_SensorTypeDef *mpu)
{
    return mpu != NULL && mpu6050_get_acc(&mpu->RAW.acc_raw_x,
                                          &mpu->RAW.acc_raw_y,
                                          &mpu->RAW.acc_raw_z);
}

bool sensor_mpu_gyro_raw(MPU_SensorTypeDef *mpu)
{
    return mpu != NULL && mpu6050_get_gyro(&mpu->RAW.gyro_raw_x,
                                           &mpu->RAW.gyro_raw_y,
                                           &mpu->RAW.gyro_raw_z);
}

void sensor_mpu_acc_tra(MPU_SensorTypeDef *mpu)
{
    // 调用加速度计转换函数，将原始数据转换为物理单位（g）
    mpu->TRA.acc_x = mpu6050_acc_transition(mpu->RAW.acc_raw_x);
    mpu->TRA.acc_y = mpu6050_acc_transition(mpu->RAW.acc_raw_y);
    mpu->TRA.acc_z = mpu6050_acc_transition(mpu->RAW.acc_raw_z);
#if DEBUG_PRINT_VERBOSE
    log_print("acc_tra:\nx:%fy:%fz:%f\n", mpu->TRA.acc_x, mpu->TRA.acc_y, mpu->TRA.acc_z);
#endif
}

void sensor_mpu_gyro_tra(MPU_SensorTypeDef *mpu)
{
    // 调用陀螺仪转换函数，将原始数据转换为物理单位（°/s）
    mpu->TRA.gyro_x = mpu6050_gyro_transition(mpu->RAW.gyro_raw_x);
    mpu->TRA.gyro_y = mpu6050_gyro_transition(mpu->RAW.gyro_raw_y);
    mpu->TRA.gyro_z = mpu6050_gyro_transition(mpu->RAW.gyro_raw_z);
#if DEBUG_PRINT_VERBOSE
    log_print("gyro_tra:\nx:%fy:%fz:%f\n", mpu->TRA.gyro_x, mpu->TRA.gyro_y, mpu->TRA.gyro_z);
#endif
}

/***************************  循迹传感器  ********************************/

/**
 * @brief 计算两个字节的汉明距离（二进制位不同的数量）
 * @param a 第一个字节
 * @param b 第二个字节
 * @return 不同位的数量
 */
static uint8_t hamming_distance(uint8_t a, uint8_t b)
{
    uint8_t xor = a ^ b; // 异或：不同位为1，相同位为0
    uint8_t count = 0;
    // 统计xor中1的个数
    while (xor)
    {
        count += xor & 0x01;
        xor >>= 1;
    }
    return count;
}

/**
 * @brief 读取8路循迹传感器数字量，并转换为十六进制值
 * @param track 循迹传感器数据结构体（存储转换后的十六进制值）
 * @return 1-读取成功，0-读取失败
 */
uint8_t sensor_hd_read_digital(Track_SensorTypeDef *track)
{
    if (track == NULL)
    {
        return 0; // 避免空指针访问
    }

    track->sensor = HD_Parallel_Read();
    // 注意：这里运行在 TIM1 中断（hd_it，10ms）上下文，禁止任何阻塞串口发送。
    // 上位机 final.py 解析的 "track_data:" 遥测已改由 FSM 主循环
    // robot_protocol_update() 以固定 10ms 节拍发出（协议格式不变）。
    return 1; // 读取成功
}

Track_SceneTypeDef sensor_hd_judge_scene(Track_SensorTypeDef *track)
{
    // 空指针保护
    if (track == NULL)
    {
#if DEBUG_PRINT_VERBOSE
        log_print("sensor->Unknown scene\n");
#endif
        return TRACK_SCENE_UNKNOWN;
    }

    // 语义：bit=1 表示该路"压到黑线"（极性取反在 hd.c 的 HD_Parallel_Read 内完成）
    uint8_t sensor_data = (uint8_t)track->sensor;

    // 全白：8 路都没压线 → 丢线
    if (sensor_data == TRACK_ALL_WHITE)
    {
#if DEBUG_PRINT_VERBOSE
        log_print("sensor->Lost line\n");
#endif
        return TRACK_SCENE_FULL_WHITE;
    }

    // 全黑：横向色带 / 校准点
    // 注意：这里只认"精确全黑"。原实现用"汉明距离 ≤1 匹配 0xF8/0x1F"来判校准点，
    // 会把"偏一边"（例如 0x3F = 左 6 路压黑，与 0x1F 只差 1 位）误判成校准点，
    // 实测后果是每 20ms 触发一次校准、状态机永远停在 CALIBRATING、车原地不动。
    // 偏左/偏右是循迹环该处理的事（track_pid 的加权偏差），不是校准点。
#if MAX_DIFF > 0
    if (hamming_distance(sensor_data, TRACK_ALL_BLACK) <= MAX_DIFF)
#else
    if (sensor_data == TRACK_ALL_BLACK)
#endif
    {
#if DEBUG_PRINT_VERBOSE
        log_print("sensor->All black (checkpoint)\n");
#endif
        return TRACK_CHECK_POINT;
    }

    // 其余情况（直线 0x18、偏左 0xF8、偏右 0x1F、任意中间态）一律按"在线上"处理，
    // 由 Track_CalcError() 的加权偏差决定左右差速。
    return TRACK_SCENE_STRAIGHT;
}

// 激光雷达三接口（sensor_get_distance / sensor_get_angle /
// sensor_get_rotation_speed，依赖 n10_* 点云解析）已随雷达模块删除
