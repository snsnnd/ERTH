#ifndef _MPU6050_DMP_H
#define _MPU6050_DMP_H

#include "main.h"
#include "iic_mpu6050.h"
#include "mpu6050_regs.h"
#include "stdio.h"
#include "stdint.h"

int8_t MPU6050_WriteReg_HAL(
    unsigned char slave_addr, // 设备地址（7位，例如0x68）
    unsigned char reg_addr,   // 寄存器地址（8位）
    unsigned char length,     // 数据长度
    unsigned char const *data // 待写入的数据缓冲区
);
int8_t MPU6050_ReadReg_HAL(
    unsigned char slave_addr, // 设备地址（7位）
    unsigned char reg_addr,   // 寄存器地址（8位）
    unsigned char length,     // 数据长度
    unsigned char *data       // 数据接收缓冲区
);                            // 向MPU6050寄存器写入数据的函数
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);

// 从MPU6050寄存器读取数据的函数
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

// 初始化MPU6050的函数
void MPU6050_Init(void);

// 获取MPU6050的设备ID的函数
uint8_t MPU6050_GetID(void);

// 获取MPU6050的加速度计和陀螺仪数据的函数
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
#endif
