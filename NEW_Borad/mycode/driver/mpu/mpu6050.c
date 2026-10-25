#include "mpu6050.h"
#include "stdio.h"
#include "iic_mpu6050.h" // 新增：包含软件I2C头文件，调用软件I2C函数
#include "BT.h"

/**
 * @brief  软件I2C实现：写MPU6050单个寄存器
 * @param  RegAddress：目标寄存器地址
 * @param  Data：要写入的8位数据
 * @note   通信流程：Start → 发送从机地址（写）→ 应答 → 发送寄存器地址 → 应答 → 发送数据 → 应答 → Stop
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    uint8_t ack_flag; // 应答标志（0=应答，1=非应答）

    // 1. 发送I2C开始信号
    SOFT_I2C_Start();

    // 2. 发送MPU6050从机地址 + 写命令（最低位0表示写）
    SOFT_I2C_SendByte(MPU6050_ADDRESS); // 0xD0 = 7位地址0x68 << 1 + 0（写）
    ack_flag = SOFT_I2C_ReceiveAck();   // 接收从机应答
    if (ack_flag != 0)                  // 无应答 → 通信失败
    {
        SOFT_I2C_Stop(); // 发送停止信号，避免总线挂死
        return;
    }

    // 3. 发送目标寄存器地址
    SOFT_I2C_SendByte(RegAddress);
    ack_flag = SOFT_I2C_ReceiveAck();
    if (ack_flag != 0)
    {
        SOFT_I2C_Stop();

        return;
    }

    // 4. 发送要写入的数据
    SOFT_I2C_SendByte(Data);
    ack_flag = SOFT_I2C_ReceiveAck();
    if (ack_flag != 0)
    {
        SOFT_I2C_Stop();

        return;
    }

    // 5. 发送I2C停止信号，结束通信
    SOFT_I2C_Stop();
}

/**
 * @brief  软件I2C实现：读MPU6050多个连续寄存器
 * @param  RegAddress：起始寄存器地址
 * @param  pData：接收数据的缓冲区（需提前分配内存）
 * @param  Size：要读取的字节数
 * @note   通信流程：Start → 写地址 → 应答 → 发寄存器地址 → 应答 → 重复Start → 读地址 → 应答 → 循环读数据 → Stop
 */
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *pData, uint16_t Size)
{
    uint8_t ack_flag;
    uint16_t i;

    // 第一步：先发送“寄存器地址”（告诉MPU6050要读哪个寄存器）
    SOFT_I2C_Start();
    // 发送写地址（先写寄存器地址）
    SOFT_I2C_SendByte(MPU6050_ADDRESS); // 0xD0（写）
    ack_flag = SOFT_I2C_ReceiveAck();
    if (ack_flag != 0)
    {
        SOFT_I2C_Stop();

        return;
    }

    // 发送起始寄存器地址
    SOFT_I2C_SendByte(RegAddress);
    ack_flag = SOFT_I2C_ReceiveAck();
    if (ack_flag != 0)
    {
        SOFT_I2C_Stop();

        return;
    }

    // 第二步：重复开始信号（切换为读操作）
    SOFT_I2C_Start();
    // 发送读地址（最低位1表示读）
    SOFT_I2C_SendByte(MPU6050_ADDRESS | 0x01); // 0xD1 = 0xD0 + 1（读）
    ack_flag = SOFT_I2C_ReceiveAck();
    if (ack_flag != 0)
    {
        SOFT_I2C_Stop();

        return;
    }

    // 第三步：循环读取Size个字节（最后1字节发非应答，其余发应答）
    for (i = 0; i < Size; i++)
    {
        pData[i] = SOFT_I2C_ReceiveByte(); // 读1字节数据

        // 控制应答：前Size-1字节发ACK（继续读），最后1字节发NACK（结束读）
        if (i < Size - 1)
            SOFT_I2C_SendAck(0); // 应答（0=ACK）
        else
            SOFT_I2C_SendAck(1); // 非应答（1=NACK）
    }

    // 第四步：发送停止信号
    SOFT_I2C_Stop();
}

/**
 * @brief  读取加速度计数据（X/Y/Z轴，原始16位数据）
 * @param  AccX：X轴加速度（输出）
 * @param  AccY：Y轴加速度（输出）
 * @param  AccZ：Z轴加速度（输出）
 * @note   从0x3B寄存器开始，连续读6字节（XH/XL/YH/YL/ZH/ZL）
 */
void mpu6050_get_acc(int16_t *AccX, int16_t *AccY, int16_t *AccZ)
{
    uint8_t acc_data[6]; // 存储6字节原始数据

    // 读6个连续寄存器（从加速度计X轴高位开始）
    MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, acc_data, 6);

    // 拼接16位数据（高8位 << 8 | 低8位）
    *AccX = (int16_t)((acc_data[0] << 8) | acc_data[1]);
    *AccY = (int16_t)((acc_data[2] << 8) | acc_data[3]);
    *AccZ = (int16_t)((acc_data[4] << 8) | acc_data[5]);
}

/**
 * @brief  读取陀螺仪数据（X/Y/Z轴，原始16位数据）
 * @param  GyroX：X轴陀螺仪（输出）
 * @param  GyroY：Y轴陀螺仪（输出）
 * @param  GyroZ：Z轴陀螺仪（输出）
 * @note   从0x43寄存器开始，连续读6字节（XH/XL/YH/YL/ZH/ZL）
 */
void mpu6050_get_gyro(int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t gyro_data[6]; // 存储6字节原始数据

    // 读6个连续寄存器（从陀螺仪X轴高位开始）
    MPU6050_ReadRegs(MPU6050_GYRO_XOUT_H, gyro_data, 6);

    // 拼接16位数据
    *GyroX = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
    *GyroY = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
    *GyroZ = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);
}

/**
 * @brief  MPU6050自检（验证通信是否正常）
 * @retval 0=自检成功，1=自检失败
 * @note   写采样率分频寄存器（0x07），再读回校验
 */
static uint8_t mpu6050_selfcheck(void)
{
    uint8_t dat = 0;
    uint8_t return_state = 0;
    uint16_t timeout_count = 0;

    // 1. 唤醒MPU6050（上电默认休眠，PWR_MGMT_1=0x00）
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);
    // 2. 写入已知值到采样率分频寄存器（0x07）
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);

    // 3. 循环读回校验（超时保护）
    while (dat != 0x07)
    {
        if (timeout_count++ > MPU6050_TIMEOUT_COUNT) // 超时 → 失败
        {

            return_state = 1;
            break;
        }

        MPU6050_ReadRegs(MPU6050_SMPLRT_DIV, &dat, 1); // 读1字节校验
        HAL_Delay(10);                                 // 延迟等待寄存器更新
    }

    return return_state;
}

/**
 * @brief  MPU6050初始化（配置电源、量程、采样率）
 * @retval 0=初始化成功，1=初始化失败
 */
uint8_t mpu6050_init(void)
{
    uint8_t return_state = 0;
    HAL_Delay(100); // 上电延时，等待传感器稳定

    do
    {
        // 1. 先自检，通信失败则退出
        if (mpu6050_selfcheck())
        {
            return_state = 1;

            break;
        }

        // 2. 配置电源管理寄存器1：X轴陀螺仪为时钟源，取消休眠
        MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
        // 3. 电源管理寄存器2：所有轴不休眠
        MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
        // 4. 采样率分频：采样率=1000/(1+9)=100Hz
        MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
        // 5. 配置低通滤波：DLPF=6（带宽5Hz，适合静态测量）
        MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
        // 6. 陀螺仪量程：±2000°/s（MPU6050_GYRO_SAMPLE=0x18）
        MPU6050_WriteReg(MPU6050_GYRO_CONFIG, MPU6050_GYRO_SAMPLE);
        // 7. 加速度计量程：±16g（MPU6050_ACC_SAMPLE=0x18）
        MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, MPU6050_ACC_SAMPLE);

    } while (0); // 简化流程控制的代码块

    return return_state;
}

/**
 * @brief  加速度计原始数据→物理单位（g）转换
 * @param  acc_value：加速度计原始16位数据
 * @retval 转换后的物理值（单位：g，1g≈9.8m/s²）
 */
float mpu6050_acc_transition(int16_t acc_value)
{
    float acc_data = 0;
    // 根据量程选择转换系数（LSB/g）
    switch (MPU6050_ACC_SAMPLE)
    {
    case 0x00:
        acc_data = (float)acc_value / 16384.0f;
        break; // ±2g（16384 LSB/g）
    case 0x08:
        acc_data = (float)acc_value / 8192.0f;
        break; // ±4g（8192 LSB/g）
    case 0x10:
        acc_data = (float)acc_value / 4096.0f;
        break; // ±8g（4096 LSB/g）
    case 0x18:
        acc_data = (float)acc_value / 2048.0f;
        break; // ±16g（2048 LSB/g）
    default:
        acc_data = 0;
        break;
    }
    return acc_data;
}

/**
 * @brief  陀螺仪原始数据→物理单位（°/s）转换
 * @param  gyro_value：陀螺仪原始16位数据
 * @retval 转换后的物理值（单位：°/s，度/秒）
 */
float mpu6050_gyro_transition(int16_t gyro_value)
{
    float gyro_data = 0;
    // 根据量程选择转换系数（LSB/(°/s)）
    switch (MPU6050_GYRO_SAMPLE)
    {
    case 0x00:
        gyro_data = (float)gyro_value / 131.0f;
        break; // ±250°/s（131 LSB/(°/s)）
    case 0x08:
        gyro_data = (float)gyro_value / 65.5f;
        break; // ±500°/s（65.5 LSB/(°/s)）
    case 0x10:
        gyro_data = (float)gyro_value / 32.8f;
        break; // ±1000°/s（32.8 LSB/(°/s)）
    case 0x18:
        gyro_data = (float)gyro_value / 16.4f;
        break; // ±2000°/s（16.4 LSB/(°/s)）
    default:
        gyro_data = 0;
        break;
    }
    return gyro_data;
}

#define PI_F32 3.14159265358979323846f

void mpu6050_calibrate_gyro(float *gyro_offset, uint16_t sample_count)
{
    if (gyro_offset == NULL || sample_count == 0)
    {
        return;
    }

    // 初始化累加器
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    int16_t raw_gx, raw_gy, raw_gz;

    // 采集指定次数的陀螺仪数据
    for (uint16_t i = 0; i < sample_count; i++)
    {
        // 读取原始陀螺仪数据
        mpu6050_get_gyro(&raw_gx, &raw_gy, &raw_gz);

        // 转换为物理单位（°/s），再转为rad/s（滤波器使用弧度单位）
        float gx = mpu6050_gyro_transition(raw_gx) * PI_F32 / 180.0f;
        float gy = mpu6050_gyro_transition(raw_gy) * PI_F32 / 180.0f;
        float gz = mpu6050_gyro_transition(raw_gz) * PI_F32 / 180.0f;

        // 累加数据
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;

        // 短暂延时，避免数据过于密集
        HAL_Delay(1);
    }

    // 计算平均值作为零偏（静止时的平均偏移量）
    gyro_offset[0] = sum_gx / sample_count;
    gyro_offset[1] = sum_gy / sample_count;
    gyro_offset[2] = sum_gz / sample_count;

#if DEBUG_PRINT
    log_print("X: %.6f, Y: %.6f, Z: %.6f\r\n",
              gyro_offset[0], gyro_offset[1], gyro_offset[2]);
#endif
}

void test_mpu6050()
{
    // 初始化MPU6050
    uint8_t mpu_state = mpu6050_init();
    if (mpu_state != 0)
    {
        log_print("MPU6050 init failed!\r\n");
        while (1)
            ; // 初始化失败则卡死
    }

    while (1)
    {
        int16_t acc_x, acc_y, acc_z;
        int16_t gyro_x, gyro_y, gyro_z;

        // 读取原始数据
        mpu6050_get_acc(&acc_x, &acc_y, &acc_z);
        mpu6050_get_gyro(&gyro_x, &gyro_y, &gyro_z);

        // 转换为物理单位
        float ax = mpu6050_acc_transition(acc_x);
        float ay = mpu6050_acc_transition(acc_y);
        float az = mpu6050_acc_transition(acc_z);

        float gx = mpu6050_gyro_transition(gyro_x);
        float gy = mpu6050_gyro_transition(gyro_y);
        float gz = mpu6050_gyro_transition(gyro_z);

        // 打印数据
        log_print("Acc: X=%.2f, Y=%.2f, Z=%.2f\r\n", ax, ay, az);
        log_print("Gyro: X=%.2f, Y=%.2f, Z=%.2f\r\n", gx, gy, gz);

        HAL_Delay(500); // 延时500ms刷新一次
    }
}
