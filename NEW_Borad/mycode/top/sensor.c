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
#if DEBUG_PRINT
    log_print("sensor->Linear velocity:\n");
    log_print("lf:%f,rf:%f,lb:%f,rb:%f\n", encoder->lf_speed, encoder->rf_speed, encoder->lb_speed, encoder->rb_speed);
#endif
}

/***************************  MPU  ********************************/
void sensor_mpu_raw(MPU_SensorTypeDef *mpu)
{
    mpu6050_get_acc(&mpu->RAW.acc_raw_x, &mpu->RAW.acc_raw_y, &mpu->RAW.acc_raw_z);
    mpu6050_get_gyro(&mpu->RAW.gyro_raw_x, &mpu->RAW.gyro_raw_y, &mpu->RAW.gyro_raw_z);
#if DEBUG_PRINT
    log_print("acc:\nx:%f,y:%f,z:%f\ngyro:\nx:%f,y:%f,z:%f\n", mpu->RAW.acc_raw_x, mpu->RAW.acc_raw_y, mpu->RAW.acc_raw_z, mpu->RAW.gyro_raw_x, mpu->RAW.gyro_raw_y, mpu->RAW.gyro_raw_z);
#endif
}

void sensor_mpu_tra(MPU_SensorTypeDef *mpu)
{
    // 若你希望每次都先刷新原始值，再做换算，用下面两行
    sensor_mpu_raw(mpu);
    sensor_mpu_acc_tra(mpu);
    sensor_mpu_gyro_tra(mpu);
#if DEBUG_PRINT
    log_print("acc_tra:\tx:%f\ty%f\tz%f\n", mpu->TRA.acc_x, mpu->TRA.acc_y, mpu->TRA.acc_z);
    log_print("gyro_tra:\tx:%f\ty%f\tz%f\n", mpu->TRA.gyro_x, mpu->TRA.gyro_y, mpu->TRA.gyro_z);
#endif
}

void sensor_mpu_acc_raw(MPU_SensorTypeDef *mpu)
{
    mpu6050_get_acc(&mpu->RAW.acc_raw_x, &mpu->RAW.acc_raw_y, &mpu->RAW.acc_raw_z);
}

void sensor_mpu_gyro_raw(MPU_SensorTypeDef *mpu)
{
    mpu6050_get_gyro(&mpu->RAW.gyro_raw_x, &mpu->RAW.gyro_raw_y, &mpu->RAW.gyro_raw_z);
}

void sensor_mpu_acc_tra(MPU_SensorTypeDef *mpu)
{
    // 调用加速度计转换函数，将原始数据转换为物理单位（g）
    mpu->TRA.acc_x = mpu6050_acc_transition(mpu->RAW.acc_raw_x);
    mpu->TRA.acc_y = mpu6050_acc_transition(mpu->RAW.acc_raw_y);
    mpu->TRA.acc_z = mpu6050_acc_transition(mpu->RAW.acc_raw_z);
#if DEBUG_PRINT
    log_print("acc_tra:\nx:%fy:%fz:%f\n", mpu->TRA.acc_x, mpu->TRA.acc_y, mpu->TRA.acc_z);
#endif
}

void sensor_mpu_gyro_tra(MPU_SensorTypeDef *mpu)
{
    // 调用陀螺仪转换函数，将原始数据转换为物理单位（°/s）
    mpu->TRA.gyro_x = mpu6050_gyro_transition(mpu->RAW.gyro_raw_x);
    mpu->TRA.gyro_y = mpu6050_gyro_transition(mpu->RAW.gyro_raw_y);
    mpu->TRA.gyro_z = mpu6050_gyro_transition(mpu->RAW.gyro_raw_z);
#if DEBUG_PRINT
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
    // 协议遥测（非调试日志）：上位机 final.py 按 "track_data:" 前缀解析，
    // 属对战/调试平台协议字段，故意不受 DEBUG_PRINT 控制
    log_print("track_data:%hu\n", track->sensor);
    return 1; // 读取成功
}

Track_SceneTypeDef sensor_hd_judge_scene(Track_SensorTypeDef *track)
{
    // 空指针保护
    if (track == NULL)
    {
#if DEBUG_PRINT
        log_print("sensor->Unknown scene\n");
#endif
        return TRACK_SCENE_UNKNOWN;
    }

    uint8_t sensor_data = track->sensor;

    if (sensor_data == 0x18)
    {
#if DEBUG_PRINT
        log_print("sensor->Straight line\n");
#endif
        return TRACK_SCENE_STRAIGHT;
    }

    // 全白场景（轨迹丢失）
    if (sensor_data == 0x00)
    {

#if DEBUG_PRINT
        log_print("sensor->Lost line\n");
#endif
        return TRACK_SCENE_FULL_WHITE;
    }

    // 1. 全黑情况：目标值0xFF（1111 1111），允许 ≤ MAX_DIFF个bit不同
    if (hamming_distance(sensor_data, 0xFF) <= MAX_DIFF)
    {
#if DEBUG_PRINT
        log_print("sensor->All black\n");

#endif
        return TRACK_CHECK_POINT;
    }

    // 2. 左侧偏黑（原目标0xF8=1111 1000，代表左5路黑、右3路白）
    // 放宽：允许与0xF8的差异 ≤ MAX_DIFF个bit
    if (hamming_distance(sensor_data, 0xF8) <= MAX_DIFF)
    {
#if DEBUG_PRINT
        log_print("sensor->Slightly to the left\n");
#endif
        return TRACK_CHECK_POINT;
    }

    // 3. 右侧偏黑（原目标0x1F=0001 1111，代表右5路黑、左3路白）
    // 放宽：允许与0x1F的差异 ≤ MAX_DIFF个bit
    if (hamming_distance(sensor_data, 0x1F) <= MAX_DIFF)
    {

#if DEBUG_PRINT
        log_print("sensor->Slightly to the right\n");
#endif
        return TRACK_CHECK_POINT;
    }
#if DEBUG_PRINT
    log_print("sensor->Straight line\n");
#endif
    return TRACK_SCENE_STRAIGHT;
}

// 激光雷达三接口（sensor_get_distance / sensor_get_angle /
// sensor_get_rotation_speed，依赖 n10_* 点云解析）已随雷达模块删除
