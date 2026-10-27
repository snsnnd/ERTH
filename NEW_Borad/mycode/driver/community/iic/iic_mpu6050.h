#ifndef _IIC_MPU6050_H
#define _IIC_MPU6050_H

#include "main.h"
#include "delay.h"
#include "stdint.h"

extern int SDA_bit;
extern int SCL_bit;

void SOFT_I2C_W_SCL(uint8_t BitValue);

void SOFT_I2C_W_SDA(uint8_t BitValue);

uint8_t SOFT_I2C_R_SDA(void);

void SOFT_I2C_Start(void);

void SOFT_I2C_Stop(void);

void SOFT_I2C_SendByte(uint8_t Byte);

uint8_t SOFT_I2C_ReceiveByte(void);

void SOFT_I2C_SendAck(uint8_t AckBit);

uint8_t SOFT_I2C_ReceiveAck(void);

// 总线恢复序列：释放 SDA → 至多 9 个 SCL 时钟脉冲 → STOP
// 等待超时或通信失败时调用，释放被从机卡住的 SDA
void SOFT_I2C_BusRecover(void);

void RCCdelay_us(uint32_t udelay);

#endif
