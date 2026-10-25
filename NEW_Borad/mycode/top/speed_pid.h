#ifndef SPEED_PID_H
#define SPEED_PID_H

#include "stm32f1xx_hal.h" // 实际 MCU：STM32F103C8T6（原理图卡片）

// 速度环参数宏定义（根据硬件调整）
#define SPEED_MAX_TARGET 30  // 最大目标速度（如rpm或PWM值）
#define SPEED_MIN_TARGET 10  // 最小目标速度
#define SPEED_OUTPUT_MAX 100 // 速度环输出限幅（PWM调节范围）

// 速度环PID结构体
typedef struct
{
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float target;       // 目标速度（外部设置）
    float current;      // 当前速度（反馈值）
    float error;        // 速度偏差（target - current）
    float last_error;   // 上一次偏差
    float integral;     // 积分项
    float integral_max; // 积分限幅
    float output;       // 输出基础速度
} SpeedPID_HandleTypeDef;

extern SpeedPID_HandleTypeDef *fl_pid;
extern SpeedPID_HandleTypeDef *fr_pid;
extern SpeedPID_HandleTypeDef *bl_pid;
extern SpeedPID_HandleTypeDef *br_pid;

/**
 * @brief  设置速度环目标速度（会自动限制在[SPEED_MIN_TARGET, SPEED_MAX_TARGET]）
 * @param  hspeed: 速度环结构体指针
 * @param  target: 目标速度
 */
void SpeedPID_SetTarget(SpeedPID_HandleTypeDef *hspeed, float target);

/**
 * @brief  速度环PID计算（需先通过SpeedPID_SetCurrent更新当前速度）
 * @param  hspeed: 速度环结构体指针
 * @retval float: 输出的基础速度
 */
float SpeedPID_Compute(SpeedPID_HandleTypeDef *hspeed);

/**
 * @brief  更新当前速度（需外部调用，传入编码器等反馈值）
 * @param  hspeed: 速度环结构体指针
 * @param  current: 当前实际速度
 */
void SpeedPID_SetCurrent(SpeedPID_HandleTypeDef *hspeed, float current);

#endif // SPEED_PID_H
