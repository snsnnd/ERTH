// 根据逐飞以及江科大文件改写的，适配逐飞的滤波
#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

#define MPU6050_ADDRESS 0xD0 // 若硬件上AD0引脚接高电平，需改为0xD2

#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_WHO_AM_I 0x75

#define MPU6050_TIMEOUT_COUNT 30 // MPU6050 自检计数

#define MPU6050_ACC_SAMPLE 0x18
#define MPU6050_GYRO_SAMPLE 0x18
// MPU6050_GYRO_SAMPLE
// 设置为:0x00 陀螺仪量程为:±250 dps       获取到的陀螺仪数据除以131.2        可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x08 陀螺仪量程为:±500 dps       获取到的陀螺仪数据除以65.6         可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x10 陀螺仪量程为:±1000dps       获取到的陀螺仪数据除以32.8         可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x18 陀螺仪量程为:±2000dps       获取到的陀螺仪数据除以16.4         可以转化为带物理单位的数据，单位为：°/s

// MPU6050_ACC_SAMPLE
// 设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
// 设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
// 设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
// 设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)

/****************************** 加速度计 *********************************/
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

/****************************** 温度计 *********************************/
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42

/****************************** 陀螺仪 *********************************/
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *pData, uint16_t Size);

uint8_t mpu6050_init(void);
void mpu6050_get_acc(int16_t *AccX, int16_t *AccY, int16_t *AccZ);
void mpu6050_get_gyro(int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
float mpu6050_acc_transition(int16_t acc_value);
float mpu6050_gyro_transition(int16_t gyro_value);

/**
 * @brief  陀螺仪零漂校准（自动计算零偏）
 * @param  gyro_offset：输出参数，存储计算得到的零偏（rad/s）
 * @param  sample_count：校准采样次数（建议1000次以上）
 */
void mpu6050_calibrate_gyro(float *gyro_offset, uint16_t sample_count);

void test_mpu6050(void);
#endif
