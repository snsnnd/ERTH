#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
#include "bee.h"

extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim1;

// 电机枚举类型定义
enum MOTOR
{
    motor_front_left,  // 0: 左前轮
    motor_front_right, // 1: 右前轮
    motor_back_left,   // 2: 左后轮
    motor_back_right,  // 3: 右后轮
    motor_all,         // 4: 所有电机
    motor_back,        // 5: 所有后轮
    motor_front        // 6: 所有前轮
};

#define PWM_MAX 100 // PWM最大
// （占空比 = CCR/(ARR+1)×100%）。

// 函数声明
void motor_init(void);
void motor_stop(void);
void motor_setspeed(enum MOTOR motor, int16_t counter);

void test_motor(void);

#endif
