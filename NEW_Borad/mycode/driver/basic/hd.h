#ifndef HD_H
#define HD_H

#include "main.h"

/**
 * @brief  读取串行设备8位数据（如灰度传感器）
 * @retval uint8_t：读取的8位数据（bit0~bit7对应设备的1~8路）
 */
uint8_t HD_Serial_Read(void);

#endif // HD_SERIAL_H
