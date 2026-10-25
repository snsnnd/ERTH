#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
// bee.h 已删：蜂鸣器模块随硬件方案舍弃

// 实际硬件（校赛原理图）：TB6612 双路差速驱动
//   PWMA = PB0 = TIM3_CH3（左轮 A 路），PWMB = PB1 = TIM3_CH4（右轮 B 路）
//   方向脚 AIN1/AIN2(BIN1/BIN2) 与 STBY 由 CubeMX 用户标签生成（见交付清单）
extern TIM_HandleTypeDef htim3; // PWM 输出定时器（CH3=左、CH4=右）

// 电机枚举类型定义（保留四轮枚举以兼容上层控制代码，
// 实际硬件为差速两驱：左/右各自合并为一路 TB6612 输出）
enum MOTOR
{
    motor_front_left,  // 0: 左前轮 → 合并到左（A）路
    motor_front_right, // 1: 右前轮 → 合并到右（B）路
    motor_back_left,   // 2: 左后轮 → 合并到左（A）路
    motor_back_right,  // 3: 右后轮 → 合并到右（B）路
    motor_all,         // 4: 两路同时
    motor_back,        // 5: 差速两驱下无后轮独立控制（不动作）
    motor_front        // 6: 差速两驱下无前轮独立控制（不动作）
};

#define PWM_MAX 100 // PWM最大
// （占空比 = CCR/(ARR+1)×100%，CubeMX TIM3 ARR 需设为 99）

// 函数声明
void motor_init(void);
void motor_stop(void);
void motor_setspeed(enum MOTOR motor, int16_t counter);

void test_motor(void);

#endif
