#include "control.h"
#include "sensor.h"
#include "motor.h"
#include "filter_algorithm.h"
#include "track_pid.h"
#include "speed_pid.h"
#include "time.h"
#include "BT.h"
#include <math.h>
#include <stdio.h>
#include "stdlib.h"

// 地图方向对应的目标角度（度，基于地图坐标系）
#define MAP_UP_ANGLE -90.0f  // 向上（+y方向）
#define MAP_RIGHT_ANGLE 0.0f // 向右（+x方向）
#define MAP_DOWN_ANGLE 90.0f // 向下（-y方向）
#define MAP_LEFT_ANGLE -0.0f // 向左（-x方向）

// 弧度制常量定义（全局）
#define M_PI 3.1415926535f
#define TWO_PI (2 * M_PI)     // 2π 弧度（对应360°）
#define HALF_PI (M_PI / 2.0f) // π/2 弧度（对应90°，常用目标值）
#define ANGLE_TOLERANCE_forward 20.0f

#define TURN_SPEED 50

// 角度PID结构体（统一为弧度制）
typedef struct
{
    float target_angle;  // 目标角度（弧度）
    float current_angle; // 当前角度（弧度，来自传感器）
    float kp;            // 比例系数
    float ki;            // 积分系数
    float kd;            // 微分系数
    float integral;      // 积分项
    float last_err;      // 上一次偏差（弧度）
    float max_output;    // 最大输出（速度缩放因子上限）
} AnglePID_HandleTypeDef;

// 初始化角度PID（弧度制参数）
AnglePID_HandleTypeDef angle_pid_ins = {
    .target_angle = 0.0f,  // 目标角度（弧度）
    .current_angle = 0.0f, // 当前角度（弧度）
    .kp = 0.12f,           // 增加比例增益，提高响应速度
    .ki = 0.06f,           // 减小积分增益，减少振荡
    .kd = 0.14f,           // 增加微分增益，抑制超调
    .max_output = 3.4f,    // 保持现有限幅
    .integral = 0.0f,
    .last_err = 0.0f,

};
AnglePID_HandleTypeDef *angle_pid = &angle_pid_ins;

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

// 偏航角向右未负
/**
 * 功能：差速四轮小车原地转向（闭环控制，全弧度制）
 * 参数：
 *   target_yaw：目标转向角度（弧度，正为顺时针，负为逆时针）
 *   base_speed：基础速度（影响转向快慢，如50cm/s）
 *   timeout：超时时间（ms，防止卡滞）
 * 返回值：0-成功，1-超时
 */
static uint8_t differential_rotate_in_place(float target_yaw,
                                            float base_speed,
                                            uint32_t timeout)
{
// 控制参数（弧度制）
#define ANGLE_THRESHOLD 0.017f // 控制精度（≈1°，1°=π/180≈0.017rad）
#define CONTROL_PERIOD_MS 50   // 控制周期（ms）
#define MAX_SPEED_SCALE 0.8f   // 速度缩放因子上限
#define PWM_MAX 100            // 电机最大PWM值

    // // 初始日志：打印目标角度（同时显示弧度和角度，便于对照）
    // log_print("[INIT] Rotate start. Target yaw: %.4f rad (%.1f°), Base speed: %.1f\n",
    //           target_yaw, target_yaw * 180 / PI, base_speed);

    ENCODER_SensorTypeDef encoder;
    uint32_t start_time = HAL_GetTick(); // 起始时间戳
    uint32_t last_time = start_time;     // 上一周期时间戳
    float start_angle = GetYawAngle();   // 初始偏航角（弧度）
    // log_print("[INIT] Start angle: %.4f rad, Timeout: %d ms\n", start_angle, timeout);

    // 初始化PID状态
    angle_pid->target_angle = target_yaw;
    angle_pid->integral = 0.0f;
    angle_pid->last_err = 0.0f;
    motor_stop(); // 确保电机初始停止

    // 闭环控制循环
    while (1)
    {
        // 1. 计算控制周期（秒）
        uint32_t current_time = HAL_GetTick();
        float dt = (current_time - last_time) / 1000.0f; // 转换为秒
        last_time = current_time;
        // 打印时间戳和控制周期（检查周期稳定性）
        log_print("\n[CYCLE] Timestamp: %d ms, dt: %.6f s\n", current_time - start_time, dt);

        // 2. 读取当前角度（原始角度和相对角度）
        float current_raw = GetYawAngle();               // 原始角度（弧度）
        float current_angle = current_raw - start_angle; // 相对初始位置的角度（弧度）
        log_print("[ANGLE] Raw: %.4f rad, Relative: %.4f rad\n", current_raw, current_angle);

        // 3. 角度归一化（映射到 [-π, π]）
        current_angle = fmod(current_angle, TWO_PI);
        if (current_angle > PI)
        {
            current_angle -= TWO_PI;
        }
        else if (current_angle < -PI)
        {
            current_angle += TWO_PI;
        }
        // log_print("[ANGLE] Normalized ([-π, π]): %.4f rad\n", current_angle);

        // 4. 计算角度偏差（核心PID输入）
        float angle_err = calculate_angle_diff(angle_pid->target_angle, current_angle);
        log_print("[ERROR] Target: %.4f rad, Current: %.4f rad, Error: %.4f rad\n",
                  angle_pid->target_angle, current_angle, angle_err);

        // 5. 角度PID计算（分拆各分项打印，便于分析参数影响）
        // 比例项（P）：直接与偏差成正比
        float p_term = angle_pid->kp * angle_err;
        // 积分项（I）：累积偏差（带限幅）
        angle_pid->integral += angle_err * dt;
        float max_integral = angle_pid->max_output / angle_pid->ki; // 积分限幅
        if (angle_pid->integral > max_integral)
        {
            angle_pid->integral = max_integral;
            // log_print("[I TERM] Integral clamped (upper): %.4f\n", angle_pid->integral);
        }
        else if (angle_pid->integral < -max_integral)
        {
            angle_pid->integral = -max_integral;
            // log_print("[I TERM] Integral clamped (lower): %.4f\n", angle_pid->integral);
        }
        float i_term = angle_pid->ki * angle_pid->integral;
        // 微分项（D）：基于偏差变化率（抑制震荡）
        float diff = (angle_err - angle_pid->last_err) / dt; // 偏差变化率
        float d_term = angle_pid->kd * diff;
        angle_pid->last_err = angle_err; // 保存当前偏差用于下一周期

        // 打印PID各分项值（关键调试数据）
        log_print("[PID TERMS] P: %.4f, I: %.4f, D: %.4f\n", p_term, i_term, d_term);

        // 总输出（速度缩放因子）及限幅
        float speed_scale = p_term + i_term + d_term;
        bool clamped = false; // 标记是否被限幅
        if (speed_scale > MAX_SPEED_SCALE)
        {
            speed_scale = MAX_SPEED_SCALE;
            clamped = true;
        }
        else if (speed_scale < -MAX_SPEED_SCALE)
        {
            speed_scale = -MAX_SPEED_SCALE;
            clamped = true;
        }
        // log_print("[SPEED SCALE] Total: %.4f, Clamped: %s\n", speed_scale, clamped ? "Yes" : "No");

        // 6. 计算四轮目标速度（同侧方向一致）
        float target_fl = -base_speed * speed_scale;
        float target_fr = base_speed * speed_scale;
        float target_bl = -base_speed * speed_scale;
        float target_br = base_speed * speed_scale;
        // log_print("[TARGET SPEEDS] FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f\n",
        //           target_fl, target_fr, target_bl, target_br);

        // 7. 读取编码器速度
        sensor_encoder(&encoder);
        encoder.lf_speed = -encoder.lf_speed; // 左前编码器反向
        encoder.rf_speed = -encoder.rf_speed; // 右前编码器反向
        encoder.lb_speed = encoder.lb_speed;
        encoder.rb_speed = -encoder.rb_speed; // 右后编码器反向
        // log_print("[ENCODER SPEEDS] FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f\n",
        //           encoder.lf_speed, encoder.rf_speed, encoder.lb_speed, encoder.rb_speed);

        // 8. 速度PID计算（打印每个电机的目标/当前/输出）
        // 左前电机
        SpeedPID_SetTarget(fl_pid, target_fl);
        SpeedPID_SetCurrent(fl_pid, encoder.lf_speed);
        float fl_pwm = SpeedPID_Compute(fl_pid);
        // 右前电机
        SpeedPID_SetTarget(fr_pid, target_fr);
        SpeedPID_SetCurrent(fr_pid, encoder.rf_speed);
        float fr_pwm = SpeedPID_Compute(fr_pid);
        // 左后电机
        SpeedPID_SetTarget(bl_pid, target_bl);
        SpeedPID_SetCurrent(bl_pid, encoder.lb_speed);
        float bl_pwm = SpeedPID_Compute(bl_pid);
        // 右后电机
        SpeedPID_SetTarget(br_pid, target_br);
        SpeedPID_SetCurrent(br_pid, encoder.rb_speed);
        float br_pwm = SpeedPID_Compute(br_pid);

        // log_print("[CALCULATED PWM] FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f\n",
        //           fl_pwm, fr_pwm, bl_pwm, br_pwm);

        // 9. PWM限幅与输出（先计算原始PWM）
        int16_t pwm_fl = (int16_t)round(fl_pwm);
        int16_t pwm_fr = (int16_t)round(fr_pwm);
        int16_t pwm_bl = (int16_t)round(bl_pwm);
        int16_t pwm_br = (int16_t)round(br_pwm);

        // -------------- 强制修正逻辑（两步法） --------------
        // 1. 第一步：修正FL和FR，确保与目标速度方向完全一致（解决基准错误）
        pwm_fl = abs(pwm_fl) * (target_fl > 0 ? 1 : -1); // FL必须与target_fl同符号
        pwm_fr = abs(pwm_fr) * (target_fr > 0 ? 1 : -1); // FR必须与target_fr同符号

        // 2. 第二步：修正BL和BR，严格跟随同侧的FL和FR（确保同侧一致）
        pwm_bl = abs(pwm_bl) * (pwm_fl > 0 ? 1 : -1); // BL与FL同符号
        pwm_br = abs(pwm_br) * (pwm_fr > 0 ? 1 : -1); // BR与FR同符号
        // ------------------------------------------------

        // 二次限幅（修正后可能超范围）
        pwm_fl = (pwm_fl > PWM_MAX) ? PWM_MAX : (pwm_fl < -PWM_MAX ? -PWM_MAX : pwm_fl);
        pwm_fr = (pwm_fr > PWM_MAX) ? PWM_MAX : (pwm_fr < -PWM_MAX ? -PWM_MAX : pwm_fr);
        pwm_bl = (pwm_bl > PWM_MAX) ? PWM_MAX : (pwm_bl < -PWM_MAX ? -PWM_MAX : pwm_bl);
        pwm_br = (pwm_br > PWM_MAX) ? PWM_MAX : (pwm_br < -PWM_MAX ? -PWM_MAX : pwm_br);

        // 打印修正后日志（验证效果）
        // log_print("[FINAL PWM AFTER CORRECTION] FL: %d, FR: %d, BL: %d, BR: %d (PWM_MAX: %d)\n",
        //           pwm_fl, pwm_fr, pwm_bl, pwm_br, PWM_MAX);

        // 输出到电机（确保修正后的值被实际使用）
        motor_setspeed(motor_front_left, pwm_fl);
        motor_setspeed(motor_front_right, pwm_fr);
        motor_setspeed(motor_back_left, pwm_bl);
        motor_setspeed(motor_back_right, pwm_br);

        // 10. 判断结束条件
        if (fabs(angle_err) <= ANGLE_THRESHOLD)
        {
            motor_stop();
            log_print("[SUCCESS] Reached target. Final error: %.4f rad\n",
                      angle_err, ANGLE_THRESHOLD);
            return 0;
        }

        if (current_time - start_time > timeout)
        {
            motor_stop();
            log_print("[TIMEOUT] Exceeded timeout. Current error: %.4f rad\n", angle_err);
            return 1;
        }

        // 控制周期延时（补偿执行时间）
        uint32_t elapsed = HAL_GetTick() - current_time;
        if (elapsed < CONTROL_PERIOD_MS)
        {
            HAL_Delay(CONTROL_PERIOD_MS - elapsed);
        }
        else
        {
            log_print("[WARNING] Cycle time exceeded: %d ms (> %d ms)\n", elapsed, CONTROL_PERIOD_MS);
        }
    }
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

// void left()
//{
//     // uint8_t result = differential_rotate_in_place(M_PI / 2, // 目标角度（顺时针）
//     //                                               6.0f,     // 基础速度
//     //                                               5000);    // 超时时间
//     turn_left();
//     if (result == 0)
//     {
// #if DEBUG_PRINT
//         log_print("turn left finished");
// #endif
//     }
//     else
//     {
// #if DEBUG_PRINT
//         log_print("turn left time out");
// #endif
//     }
// }

// void right()
//{
//     // uint8_t result = differential_rotate_in_place(-M_PI / 2, // 目标角度（顺时针）
//     //                                               5.0f,      // 基础速度
//     //                                               3000);     // 超时时间
//     turn_right();
//     if (result == 0)
//     {
// #if DEBUG_PRINT
//         log_print("turn right finished");
// #endif
//     }
//     else
//     {
// #if DEBUG_PRINT
//         log_print("turn right time out");
// #endif
//     }
// }

// static void back()
//{
//     // uint8_t result = differential_rotate_in_place(M_PI,  // 目标角度（顺时针）
//     //                                               10.0f, // 基础速度
//     //                                               3000); // 超时时间
//     turn_back();
//     if (result == 0)
//     {
// #if DEBUG_PRINT
//         log_print("turn back finished");
// #endif
//     }
//     else
//     {
// #if DEBUG_PRINT
//         log_print("turn back time out");
// #endif
//     }
// }

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

    // 1. 地图方向 → 目标角度（度）
    float target_angle_deg;
    switch (map_dir)
    {
    case DIRECTION_UP:
        target_angle_deg = MAP_UP_ANGLE;
        break;
    case DIRECTION_RIGHT:
        target_angle_deg = MAP_RIGHT_ANGLE;
        break;
    case DIRECTION_DOWN:
        target_angle_deg = MAP_DOWN_ANGLE;
        break;
    case DIRECTION_LEFT:
        target_angle_deg = MAP_LEFT_ANGLE;
        break;
    case DIRECTION_STOP:
        // 已到达目标，直接停止
        print_robot_action(ROBOT_ACTION_STOP);
        return 1;
    default:
        print_robot_action(ROBOT_ACTION_STOP); // 无效方向，返回停止
        return -1;
    }

    float current_angle_deg = *current_pose;

    // 3. 计算角度差值（目标 - 当前）
    float angle_diff = calculate_angle_diff(target_angle_deg, current_angle_deg);

    // 4. 根据差值确定动作
    if (fabsf(angle_diff) <= ANGLE_TOLERANCE_forward)
    {
        print_robot_action(ROBOT_ACTION_FORWARD); // 方向一致，前进
    }
    else if (angle_diff > 0.0f)
    {
        // 差值为正：左转（逆时针转）
        print_robot_action((angle_diff > 170.0f) ? ROBOT_ACTION_TURN_AROUND : ROBOT_ACTION_TURN_LEFT);
    }
    else
    {
        // 差值为负：右转（顺时针转）
        print_robot_action((angle_diff < -170.0f) ? ROBOT_ACTION_TURN_AROUND : ROBOT_ACTION_TURN_RIGHT);
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
