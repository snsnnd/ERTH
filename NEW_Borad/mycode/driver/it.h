#ifndef IT_H
#define IT_H

#include "main.h"
#include "tim.h"

void TIM6_Start(void);
void TIM7_Start(void);
void TIM12_Start(void);
void TIM12_Stop(void);

/* 定时器中断回调函数 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

// TIM6 保持 20ms 周期，负责核心控制逻辑
// TIM7 保持 27ms 周期，负责超声波超时检测
// TIM12 替代原 TIM5 功能，实现 310ms 单次触发控制

#endif