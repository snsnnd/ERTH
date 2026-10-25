#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "sensor.h"
// arm_math.h 已删：STM32F103C8T6（Cortex-M3，无 FPU）工程不带 CMSIS-DSP，改用 math.h

// 机器人机械参数
#define WHEEL_BASE 0.30f    // 轴距(m)
#define TRACK_WIDTH 0.20f   // 轮距(m)
#define WHEEL_RADIUS 0.065f // 轮子半径(m)

// 速度结构体
typedef struct
{
    float vx;    // 沿x轴速度(m/s)
    float vy;    // 沿y轴速度(m/s)
    float omega; // 角速度(rad/s)
} RobotVelocity;

// 函数声明
void mecanum_inverse_kinematics(const RobotVelocity *robot_vel, ENCODER_SensorTypeDef *wheel_speeds);
void mecanum_forward_kinematics(const ENCODER_SensorTypeDef *wheel_speeds, RobotVelocity *robot_vel);
void kinematics_robot_to_global(float robot_dx, float robot_dy, float theta,
                                float *global_dx, float *global_dy);
void kinematics_global_to_robot(float global_dx, float global_dy, float theta,
                                float *robot_dx, float *robot_dy);

#endif // KINEMATICS_H
