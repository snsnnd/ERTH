#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
// bee.h 已删：蜂鸣器模块随硬件方案舍弃

// 实际硬件（校赛原理图，已按参考车 mishiduobao_f103 的实车结论校正）：
//   PWMB = PB1 = TIM3_CH4 + BIN1/BIN2  → 左轮（B 路）
//   PWMA = PB0 = TIM3_CH3 + AIN1/AIN2  → 右轮（A 路）
//   方向脚 AIN1/AIN2(BIN1/BIN2) 与 STBY 由 CubeMX 用户标签生成（见交付清单）
// 注：原代码按"A=左、B=右"写，与实车接线相反，表现是"让它左转它右转"。
extern TIM_HandleTypeDef htim3; // PWM 输出定时器（CH3=右轮A路、CH4=左轮B路）

// 电机方向符号（与参考车 app_config.h 一致）：右轮是镜像安装，需取反
#define LEFT_MOTOR_SIGN 1   // 左轮反了就改成 -1
#define RIGHT_MOTOR_SIGN -1 // 右轮反了就改成 1

// 电机枚举类型定义（保留四轮枚举以兼容上层控制代码，
// 实际硬件为差速两驱：左/右各自合并为一路 TB6612 输出）
enum MOTOR
{
    motor_front_left,  // 0: 左前轮 → 合并到左（B）路
    motor_front_right, // 1: 右前轮 → 合并到右（A）路
    motor_back_left,   // 2: 左后轮 → 合并到左（B）路
    motor_back_right,  // 3: 右后轮 → 合并到右（A）路
    motor_all,         // 4: 两路同时
    motor_back,        // 5: 差速两驱下无后轮独立控制（不动作）
    motor_front        // 6: 差速两驱下无前轮独立控制（不动作）
};

// PWM 满量程：TIM3 ARR=999（1 kHz），占空比 = CCR/(999+1)
#define PWM_MAX 999

// 函数声明
void motor_init(void);
void motor_stop(void);
void motor_set_lr(int16_t left_pwm, int16_t right_pwm);
void motor_setspeed(enum MOTOR motor, int16_t counter);

void test_motor(void);

#endif
