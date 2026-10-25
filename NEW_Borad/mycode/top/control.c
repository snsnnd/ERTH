#include "control.h"
#include "sensor.h"
#include "motor.h"
#include "track_pid.h"
#include "speed_pid.h"
#include "BT.h"
#include <math.h>
#include <stdio.h>
#include "stdlib.h"

// 地图方向对应的目标角度（弧度，与 current_pose.theta 单位一致）
// 角度定义：0=向右(+x)，顺时针为正 → 上=-π/2、下=+π/2、左=±π（取 -π 与上/左负号方向一致）
#define MAP_UP_ANGLE (-1.5707963f)   // 向上（+y方向）：-90° = -π/2
#define MAP_RIGHT_ANGLE 0.0f         // 向右（+x方向）：0° = 0
#define MAP_DOWN_ANGLE 1.5707963f    // 向下（-y方向）：+90° = +π/2
#define MAP_LEFT_ANGLE (-3.1415927f) // 向左（-x方向）：-180° = -π（与 +π 等价，环绕归一化后等价）

// 弧度制常量定义（全局）
#define M_PI 3.1415926535f
#define TWO_PI (2 * M_PI)     // 2π 弧度（对应360°）
#define HALF_PI (M_PI / 2.0f) // π/2 弧度（对应90°，常用目标值）
// calculate_angle_diff 的环绕归一化依赖 PI：仓库内未定义 PI 时用本文件的 M_PI 兜底
#ifndef PI
#define PI M_PI
#endif
#define ANGLE_TOLERANCE_forward 0.349f // 前进容差：20° = 0.349 rad
#define TURN_AROUND_THRESHOLD 2.967f   // 掉头阈值：170° = 2.967 rad

#define TURN_SPEED 50

extern SpeedPID_HandleTypeDef *fl_pid;
extern SpeedPID_HandleTypeDef *fr_pid;
extern SpeedPID_HandleTypeDef *bl_pid;
extern SpeedPID_HandleTypeDef *br_pid;

/**
 * 计算当前角度与目标角度的差值（归一化到 -π ~ π 弧度）
 * 确保差值为最短路径（如：3π/2 与 -π/2 等价）
 */
static float calculate_angle_diff(float target_rad, float current_rad)
{
    float diff = target_rad - current_rad;
    // 环绕处理：映射到 [-π, π]
    if (diff > PI)
    {
        diff -= TWO_PI;
    }
    else if (diff < -PI)
    {
        diff += TWO_PI;
    }
    return diff;
}

// 1. 参数改为指针，确保结构体状态能被持续更新
void forward()
{
    ENCODER_SensorTypeDef encoder_instance;             // 结构体实例
    ENCODER_SensorTypeDef *encoder = &encoder_instance; // 指针指向实例
    sensor_encoder(encoder);                            // 传入指针，正确读取数据

    int track_left, track_right;
    float fl_pwm, fr_pwm, bl_pwm, br_pwm;

    // 1. 循迹控制：使用指针访问结构体，确保状态更新
    Track_MainControl(&track_left, &track_right);

    // 2. 速度闭环：每个电机独立使用自己的PID结构体和编码器反馈
    // 左前电机（假设独立编码器）
    SpeedPID_SetTarget(fl_pid, track_left);
    SpeedPID_SetCurrent(fl_pid, encoder->lf_speed); // 独立读取左前速度
    fl_pwm = SpeedPID_Compute(fl_pid);

    // 右前电机
    SpeedPID_SetTarget(fr_pid, track_right);
    SpeedPID_SetCurrent(fr_pid, encoder->rf_speed); // 独立读取右前速度
    fr_pwm = SpeedPID_Compute(fr_pid);

    // 左后电机
    SpeedPID_SetTarget(bl_pid, track_left);
    SpeedPID_SetCurrent(bl_pid, encoder->lb_speed); // 独立读取左后速度
    bl_pwm = SpeedPID_Compute(bl_pid);

    // 右后电机
    SpeedPID_SetTarget(br_pid, track_right);
    SpeedPID_SetCurrent(br_pid, encoder->rb_speed); // 独立读取右后速度
    br_pwm = SpeedPID_Compute(br_pid);

    // 3. 转换为整数PWM并输出
    int16_t fl_pwm_int = (int16_t)round(fl_pwm);
    int16_t fr_pwm_int = (int16_t)round(fr_pwm);
    int16_t bl_pwm_int = (int16_t)round(bl_pwm);
    int16_t br_pwm_int = (int16_t)round(br_pwm);

    motor_setspeed(motor_front_left, fl_pwm_int);
    motor_setspeed(motor_front_right, fr_pwm_int);
    motor_setspeed(motor_back_left, bl_pwm_int);
    motor_setspeed(motor_back_right, br_pwm_int);
}

int map_dir_to_robot_action(Direction map_dir, float *current_pose)
{
    if (current_pose == NULL)
    {
#if DEBUG_PRINT
        log_print("control->no pos\n");
#endif
        print_robot_action(ROBOT_ACTION_STOP);
        return -1;
    }

    // 1. 地图方向 → 目标角度（弧度）
    float target_angle_rad;
    switch (map_dir)
    {
    case DIRECTION_UP:
        target_angle_rad = MAP_UP_ANGLE;
        break;
    case DIRECTION_RIGHT:
        target_angle_rad = MAP_RIGHT_ANGLE;
        break;
    case DIRECTION_DOWN:
        target_angle_rad = MAP_DOWN_ANGLE;
        break;
    case DIRECTION_LEFT:
        target_angle_rad = MAP_LEFT_ANGLE;
        break;
    case DIRECTION_STOP:
        // 已到达目标，直接停止
        print_robot_action(ROBOT_ACTION_STOP);
        return 1;
    default:
        print_robot_action(ROBOT_ACTION_STOP); // 无效方向，返回停止
        return -1;
    }

    float current_angle_rad = *current_pose; // current_pose.theta 为弧度

    // 3. 计算角度差值（目标 - 当前，弧度，环绕归一化到 [-π, π]）
    float angle_diff = calculate_angle_diff(target_angle_rad, current_angle_rad);

    // 4. 根据差值确定动作（容差与掉头阈值均为弧度）
    if (fabsf(angle_diff) <= ANGLE_TOLERANCE_forward)
    {
        print_robot_action(ROBOT_ACTION_FORWARD); // 方向一致，前进
    }
    else if (angle_diff > 0.0f)
    {
        // 差值为正：左转（逆时针转）
        print_robot_action((angle_diff > TURN_AROUND_THRESHOLD) ? ROBOT_ACTION_TURN_AROUND : ROBOT_ACTION_TURN_LEFT);
    }
    else
    {
        // 差值为负：右转（顺时针转）
        print_robot_action((angle_diff < -TURN_AROUND_THRESHOLD) ? ROBOT_ACTION_TURN_AROUND : ROBOT_ACTION_TURN_RIGHT);
    }
    return 1;
}

void print_robot_action(RobotAction action)
{
    switch (action)
    {
    case ROBOT_ACTION_FORWARD:
#if DEBUG_PRINT
        log_print("Robot Action: Forward\r\n");
#endif
        forward();
        break;
    case ROBOT_ACTION_TURN_LEFT:
#if DEBUG_PRINT
        log_print("Robot Action: Turn Left (90°)\r\n");
#endif
        turn_left(TURN_SPEED);
        break;
    case ROBOT_ACTION_TURN_RIGHT:
#if DEBUG_PRINT
        log_print("Robot Action: Turn Right (90°)\r\n");
#endif
        turn_right(TURN_SPEED);
        break;
    case ROBOT_ACTION_TURN_AROUND:
#if DEBUG_PRINT
        log_print("Robot Action: Turn Around (180°)\r\n");
#endif
        turn_back(TURN_SPEED);
        break;
    case ROBOT_ACTION_STOP:
#if DEBUG_PRINT
        log_print("Robot Action: Stop\r\n");
#endif
        motor_stop();
        break;
    default:
#if DEBUG_PRINT
        log_print("Robot Action: Unknown\r\n");
#endif
    }
}

/**
 * @brief 控制机器人左转（原地左转，左右轮反向转动）
 * @param speed 转动速度（0~PWM_MAX），值越大转速越快
 */
void turn_left(int16_t speed)
{
    // 限制速度范围（仅取正值，方向由函数内部控制）
    if (speed < 0)
        speed = 0;
    if (speed > PWM_MAX)
        speed = PWM_MAX;

    // 左转逻辑：左轮后退，右轮前进
    motor_setspeed(motor_front_left, -speed); // 左前轮后退
    motor_setspeed(motor_back_left, -speed);  // 左后轮后退
    motor_setspeed(motor_front_right, speed); // 右前轮前进
    motor_setspeed(motor_back_right, speed);  // 右后轮前进
}

/**
 * @brief 控制机器人右转（原地右转，左右轮反向转动）
 * @param speed 转动速度（0~PWM_MAX），值越大转速越快
 */
void turn_right(int16_t speed)
{
    // 限制速度范围（仅取正值，方向由函数内部控制）
    if (speed < 0)
        speed = 0;
    if (speed > PWM_MAX)
        speed = PWM_MAX;

    // 右转逻辑：左轮前进，右轮后退
    motor_setspeed(motor_front_left, speed);   // 左前轮前进
    motor_setspeed(motor_back_left, speed);    // 左后轮前进
    motor_setspeed(motor_front_right, -speed); // 右前轮后退
    motor_setspeed(motor_back_right, -speed);  // 右后轮后退
}

/**
 * @brief 控制机器人后转（所有轮子后退）
 * @param speed 后退速度（0~PWM_MAX），值越大速度越快
 */
void turn_back(int16_t speed)
{
    // 限制速度范围（仅取正值，转为负值实现后退）
    if (speed < 0)
        speed = 0;
    if (speed > PWM_MAX)
        speed = PWM_MAX;
    int16_t back_speed = -speed; // 转为负值实现后退

    // 后转逻辑：所有轮子后退
    motor_setspeed(motor_front_left, back_speed);
    motor_setspeed(motor_front_right, back_speed);
    motor_setspeed(motor_back_left, back_speed);
    motor_setspeed(motor_back_right, back_speed);
}
