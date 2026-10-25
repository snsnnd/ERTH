#include "mpu6050_dmp.h"

#define MPU6050_ADDRESS 0xD0 // 定义MPU6050的地址

int8_t MPU6050_WriteReg_HAL(
    unsigned char slave_addr, // 设备地址（7位，例如0x68）
    unsigned char reg_addr,   // 寄存器地址（8位）
    unsigned char length,     // 数据长度
    unsigned char const *data // 待写入的数据缓冲区
)
{
    SOFT_I2C_Start();

    // 发送设备地址（左移1位 + 写模式）
    SOFT_I2C_SendByte((slave_addr << 1) | 0x00); // 关键修改点
    if (SOFT_I2C_ReceiveAck() != 0)
    { // ACK=0表示成功
        SOFT_I2C_Stop();
        return -1; // 设备无响应
    }

    // 发送寄存器地址
    SOFT_I2C_SendByte(reg_addr);
    if (SOFT_I2C_ReceiveAck() != 0)
    {
        SOFT_I2C_Stop();
        return -2; // 寄存器地址ACK错误
    }

    // 发送数据
    for (unsigned char i = 0; i < length; i++)
    {
        SOFT_I2C_SendByte(data[i]);
        if (SOFT_I2C_ReceiveAck() != 0)
        {
            SOFT_I2C_Stop();
            return -3; // 数据ACK错误
        }
    }

    SOFT_I2C_Stop();
    return 0; // 成功
}
int8_t MPU6050_ReadReg_HAL(
    unsigned char slave_addr, // 设备地址（7位）
    unsigned char reg_addr,   // 寄存器地址（8位）
    unsigned char length,     // 数据长度
    unsigned char *data       // 数据接收缓冲区
)
{
    SOFT_I2C_Start();

    // 发送设备地址（左移1位 + 写模式）
    SOFT_I2C_SendByte((slave_addr << 1) | 0x00); // 关键修改点
    if (SOFT_I2C_ReceiveAck() != 0)
    { // ACK=0表示成功
        SOFT_I2C_Stop();
        return -1; // 设备无响应
    }

    // 发送寄存器地址
    SOFT_I2C_SendByte(reg_addr);
    if (SOFT_I2C_ReceiveAck() != 0)
    {
        SOFT_I2C_Stop();
        return -2; // 寄存器地址ACK错误
    }

    // 重新发送起始条件并切换为读模式
    SOFT_I2C_Start();
    SOFT_I2C_SendByte((slave_addr << 1) | 0x01); // 关键修改点
    if (SOFT_I2C_ReceiveAck() != 0)
    {
        SOFT_I2C_Stop();
        return -1; // 设备无响应
    }

    // 接收数据
    for (unsigned char i = 0; i < length; i++)
    {
        uint8_t ack = (i == length - 1) ? 1 : 0; // 最后一个字节发送NACK
        data[i] = SOFT_I2C_ReceiveByte();
        SOFT_I2C_SendAck(ack); // 发送ACK/NACK
    }

    SOFT_I2C_Stop();
    return 0; // 成功
}
// 向MPU6050寄存器写入数据
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    SOFT_I2C_Start();                   // 开始I2C通信
    SOFT_I2C_SendByte(MPU6050_ADDRESS); // 发送设备地址
    SOFT_I2C_ReceiveAck();              // 等待应答
    SOFT_I2C_SendByte(RegAddress);      // 发送寄存器地址
    SOFT_I2C_ReceiveAck();              // 等待应答
    SOFT_I2C_SendByte(Data);            // 发送数据
    SOFT_I2C_ReceiveAck();              // 等待应答
    SOFT_I2C_Stop();                    // 停止I2C通信
}

// 从MPU6050寄存器读取数据
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    SOFT_I2C_Start();                   // 开始I2C通信
    SOFT_I2C_SendByte(MPU6050_ADDRESS); // 发送设备地址
    SOFT_I2C_ReceiveAck();              // 等待应答
    SOFT_I2C_SendByte(RegAddress);      // 发送寄存器地址
    SOFT_I2C_ReceiveAck();              // 等待应答

    SOFT_I2C_Start();                          // 重新开始I2C通信
    SOFT_I2C_SendByte(MPU6050_ADDRESS | 0x01); // 发送设备地址，并设置读操作位
    SOFT_I2C_ReceiveAck();                     // 等待应答
    Data = SOFT_I2C_ReceiveByte();             // 接收数据
    SOFT_I2C_SendAck(1);                       // 发送应答信号
    SOFT_I2C_Stop();                           // 停止I2C通信

    return Data; // 返回读取到的数据
}

// 初始化MPU6050
void MPU6050_Init(void)
{
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);   // 设置电源管理寄存器
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);   // 设置电源管理寄存器
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);   // 设置采样率寄存器
    MPU6050_WriteReg(MPU6050_CONFIG, 0x03);       // 设置配置寄存器
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);  // 设置陀螺仪配置寄存器
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); // 设置加速度计配置寄存器
}

// 获取MPU6050的设备ID
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I); // 读取设备ID寄存器
}

// 获取MPU6050的加速度计和陀螺仪数据
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t DataH, DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H); // 读取加速度计X轴高字节
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L); // 读取加速度计X轴低字节
    *AccX = (DataH << 8) | DataL;                  // 合并高字节和低字节，得到加速度计X轴数据

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H); // 读取加速度计Y轴高字节
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L); // 读取加速度计Y轴低字节
    *AccY = (DataH << 8) | DataL;                  // 合并高字节和低字节，得到加速度计Y轴数据

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H); // 读取加速度计Z轴高字节
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L); // 读取加速度计Z轴低字节
    *AccZ = (DataH << 8) | DataL;                  // 合并高字节和低字节，得到加速度计Z轴数据

    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H); // 读取陀螺仪X轴高字节
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L); // 读取陀螺仪X轴低字节
    *GyroX = (DataH << 8) | DataL;                // 合并高字节和低字节，得到陀螺仪X轴数据

    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H); // 读取陀螺仪Y轴高字节
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L); // 读取陀螺仪Y轴低字节
    *GyroY = (DataH << 8) | DataL;                // 合并高字节和低字节，得到陀螺仪Y轴数据

    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H); // 读取陀螺仪Z轴高字节
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L); // 读取陀螺仪Z轴低字节
    *GyroZ = (DataH << 8) | DataL;                // 合并高字节和低字节，得到陀螺仪Z轴数据
}
