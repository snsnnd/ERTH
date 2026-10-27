#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "main.h"
#include "stdbool.h"

extern TIM_HandleTypeDef htim2; // 编码器左路（PA0/PA1，H15）
extern TIM_HandleTypeDef htim4; // 编码器右路（PB6/PB7，H14）
// 实际硬件仅 2 路编码器：同侧前后轮共用一路（htim3 已改作 PWM 输出）

enum ENCODER
{
    encoder_front_left,
    encoder_back_left,
    encoder_front_right,
    encoder_back_right
};

void encoder_init(void);
void encoder_clearcounter(void);
bool encoder_checkerror(enum ENCODER encoder);
float encoder_readspeed(enum ENCODER encoder);
void encoder_setparam(uint16_t ENCODER_PPR, uint8_t ENCODER_MULTIPLE, float WHEEL_DIAMETER);

// 编码器线数, 倍频系数, 轮子直径

// 编码器硬件参数参考
// #define ENCODER_PPR 13       // 编码器线数
// #define ENCODER_MULTIPLE 4    // 倍频系数
// #define WHEEL_DIAMETER 6.5f  // 轮子直径cm

void encoder_test_direction(enum ENCODER encoder);

#endif
