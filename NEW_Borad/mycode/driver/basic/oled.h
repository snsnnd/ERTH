#ifndef __OLED_H
#define __OLED_H

#include "stdint.h"

#define u8 uint8_t
#define u32 uint32_t

// 实际硬件（校赛原理图）：OLED 为 I2C 4 针（VCC/GND/SCL/SDA），SCL/SDA 走 PB8/PB9
// 软件 I2C 总线，与 MPU6050 共用（R12 10k 上拉）。该模块没有独立的 RST 与 D/C 引脚：
// RST 由上电自复位完成，D/C 改由 I2C 控制字节（0x00 命令 / 0x40 数据）代替。
// 原 4 线软件 SPI 时序（PD11~PD14 位带宏）已废弃——STM32F103C8T6 没有 GPIOD，
// 且原理图上不存在这 4 根线。
#define OLED_RST_Clr() ((void)0)
#define OLED_RST_Set() ((void)0)

#define OLED_CMD 0  // 命令
#define OLED_DATA 1 // 数据

// Oled control function

void OLED_WR_Byte(u8 dat, u8 cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x, u8 y, u8 t);
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size, u8 mode);
void OLED_ShowNumber(u8 x, u8 y, u32 num, u8 len, u8 size);
void OLED_ShowString(u8 x, u8 y, const u8 *p);

#define CNSizeWidth 16
#define CNSizeHeight 16
// extern char Hzk16[][16];
void OLED_ShowCHinese(u8 x, u8 y, u8 no, u8 font_width, u8 font_height);
void OLED_Set_Pos(unsigned char x, unsigned char y);

void test_oled(void);
void OLED_ShowBinary(u8 x, u8 y, u32 num, u8 len, u8 size);

#endif
