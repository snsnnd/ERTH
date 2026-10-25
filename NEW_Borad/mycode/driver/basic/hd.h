#ifndef HD_H
#define HD_H

#include "main.h"

/**
 * @brief  并行读取循迹模块 8 路数字量
 * @retval uint8_t：8位数据（bit0~bit7 对应座子 1~8 脚，见 hd.c 引脚表）
 * @note   原 HD_Serial_Read（PC10/PC11 软串行）已随实际硬件改为并行读
 */
uint8_t HD_Parallel_Read(void);

#endif // HD_H
