#ifndef HD_H
#define HD_H

#include "main.h"

/**
 * @brief  并行读取循迹模块 8 路数字量
 * @retval uint8_t：bit0~bit7 对应座子 1~8 脚（默认 bit0 = 最左），
 *                 **1 = 该路压到黑线**、0 = 白底（极性见 hd.c 的 HD_BLACK_IS_LOW）
 * @note   原 HD_Serial_Read（PC10/PC11 软串行）已随实际硬件改为并行读；
 *         若实车左右通道相反，把 hd.c 的 HD_REVERSE_BITS 改成 1 即可
 */
uint8_t HD_Parallel_Read(void);

#endif // HD_H
