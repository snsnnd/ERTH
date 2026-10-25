#include "kinematics.h"
#include "arm_math.h"
#include <stdint.h>
#include "delay.h"
#include "BT.h"

// 预计算常量（差速驱动专用）
#define TRACK_WIDTH_HALF (TRACK_WIDTH / 2.0f) // 轮距的一半
#define HALF 0.5f                             // 1/2常量

/**
 * 差速驱动逆运动学：根据机器人速度计算四轮速度
 * 注：差速驱动无横向运动能力，vy输入无效（实际计算中忽略）
 */
void mecanum_inverse_kinematics(const RobotVelocity *robot_vel, ENCODER_SensorTypeDef *wheel_speeds)
{
    if (robot_vel == NULL || wheel_speeds == NULL)
    {
#if DEBUG_PRINT
        log_print("kinematics->params of inverse_kinematics is null\r\n");
#endif
        return;
    }
    if (isnan(robot_vel->vx) || isnan(robot_vel->vy) || isnan(robot_vel->omega))
    {
#if DEBUG_PRINT
        log_print("kinematics->input of inverse_kinematics is useless\n");
        log_print("vx=%.2f, vy=%.2f, omega=%.2f\r\n",
                  robot_vel->vx, robot_vel->vy, robot_vel->omega);
#endif
        return;
    }

    // 差速驱动：左右轮速度计算（左右两侧轮子速度分别相同）
    // 左轮速度 = vx - (轮距/2) * 角速度
    // 右轮速度 = vx + (轮距/2) * 角速度
    float left_speed = robot_vel->vx - TRACK_WIDTH_HALF * robot_vel->omega;
    float right_speed = robot_vel->vx + TRACK_WIDTH_HALF * robot_vel->omega;

    // 四轮差速：左前=左后，右前=右后
    wheel_speeds->lf_speed = left_speed;
    wheel_speeds->lb_speed = left_speed;
    wheel_speeds->rf_speed = right_speed;
    wheel_speeds->rb_speed = right_speed;
}

/**
 * 差速驱动正运动学：根据四轮速度计算机器人速度
 * 注：差速驱动vy始终为0
 */
void mecanum_forward_kinematics(const ENCODER_SensorTypeDef *wheel_speeds, RobotVelocity *robot_vel)
{
    if (wheel_speeds == NULL || robot_vel == NULL)
    {
#if DEBUG_PRINT
        log_print("kinematics->params of forward_kinematics is null\r\n");
#endif
        return;
    }
    if (isnan(wheel_speeds->lf_speed) || isnan(wheel_speeds->rf_speed) ||
        isnan(wheel_speeds->lb_speed) || isnan(wheel_speeds->rb_speed))
    {
#if DEBUG_PRINT
        log_print("kinematics->input of forward_kinematics is useless\n"); // 修正原笔误
        log_print("lf=%.2f, rf=%.2f, lb=%.2f, rb=%.2f\r\n",
                  wheel_speeds->lf_speed, wheel_speeds->rf_speed,
                  wheel_speeds->lb_speed, wheel_speeds->rb_speed);
#endif
        return;
    }

    // 计算左右轮平均速度（同一侧两轮取平均）
    float left_avg = (wheel_speeds->lf_speed + wheel_speeds->lb_speed) * HALF;
    float right_avg = (wheel_speeds->rf_speed + wheel_speeds->rb_speed) * HALF;

    // 差速驱动速度合成
    robot_vel->vx = (left_avg + right_avg) * HALF;           // 前进速度=左右轮平均
    robot_vel->vy = 0.0f;                                    // 差速驱动无横向速度
    robot_vel->omega = (right_avg - left_avg) / TRACK_WIDTH; // 角速度=(右轮速-左轮速)/轮距

#if DEBUG_PRINT
    log_print("robot_vel->vx:%f,robot_vel->vy:%f,robot_vel->omega:%f\n", robot_vel->vx, robot_vel->vy, robot_vel->omega);
#endif
}

// 坐标变换函数不变（与驱动类型无关）
void kinematics_robot_to_global(float robot_dx, float robot_dy, float theta,
                                float *global_dx, float *global_dy)
{
    if (global_dx == NULL || global_dy == NULL)
    {
#if DEBUG_PRINT
        log_print("kinematics->params of robot to global is null\r\n");
#endif
        return;
    }
    if (isnan(robot_dx) || isnan(robot_dy) || isnan(theta))
    {
#if DEBUG_PRINT
        log_print("kinematics->input of robot to global is useless\n");
        log_print("dx=%.2f, dy=%.2f, theta=%.2f\r\n",
                  robot_dx, robot_dy, theta);
#endif
        return;
    }
    float cos_theta = arm_cos_f32(theta);
    float sin_theta = arm_sin_f32(theta);
    *global_dx = robot_dx * cos_theta - robot_dy * sin_theta;
    *global_dy = robot_dx * sin_theta + robot_dy * cos_theta;
}

void kinematics_global_to_robot(float global_dx, float global_dy, float theta,
                                float *robot_dx, float *robot_dy)
{
    if (robot_dx == NULL || robot_dy == NULL)
    {
#if DEBUG_PRINT
        log_print("kinematics->params of global to robot is null\r\n");
#endif
        return;
    }
    if (isnan(global_dx) || isnan(global_dy) || isnan(theta))
    {
#if DEBUG_PRINT
        log_print("kinematics->input of global to robot is useless\n"); // 修正原笔误"sinput"
        log_print("dx=%.2f, dy=%.2f, theta=%.2f\r\n",
                  global_dx, global_dy, theta);
#endif
        return;
    }
    float cos_theta = arm_cos_f32(theta);
    float sin_theta = arm_sin_f32(theta);
    *robot_dx = global_dx * cos_theta + global_dy * sin_theta;
    *robot_dy = global_dy * cos_theta - global_dx * sin_theta;
}
