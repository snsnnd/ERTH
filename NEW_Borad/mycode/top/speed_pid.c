#include "speed_pid.h"
SpeedPID_HandleTypeDef fl_pid_instance = {
    .kp = 13.0f, // 增大比例系数：增强对误差的响应，提供更大加速PWM
    .ki = 3.2f,  // 适当增大积分：通过积分累积消除长期欠调（误差=20-实际值）
    .kd = 3.4f,
    .integral_max = 46.0f,

    // 速度：
    .target = 25.0f,
    .current = 0.0f,
    .error = 0.0f,
    .last_error = 0.0f,

    // pwm：
    .integral = 0.0f,
    .output = 0.0f};
SpeedPID_HandleTypeDef *fl_pid = &fl_pid_instance; // 空转很稳定

SpeedPID_HandleTypeDef fr_pid_instance = {
    .kp = 14.2f,           // 大幅降低比例（从15→10），减少波动中的推力，避免“越调越晃”
    .ki = 3.8f,            // 维持低积分，仅用于消除可能的长期微小偏差（当前偏差已很小）
    .kd = 3.5f,            // 激进提升微分（从3→8），强力抑制误差变化率，像“减震器”一样快速衰减震荡
    .integral_max = 40.0f, // 严格限制积分上限，防止积分在震荡中累积加剧波动
    .current = 0.0f,
    .error = 0.0f,
    .last_error = 0.0f,

    // pwm：
    .integral = 0.0f,
    .output = 0.0f};
SpeedPID_HandleTypeDef *fr_pid = &fr_pid_instance;

SpeedPID_HandleTypeDef bl_pid_instance = {
    .kp = 12.9f,           // 9.0            // 小比例：偏差=1.14时，p_term=3.42（弱驱动，避免超调）
    .ki = 5.3f,            // 4.8       // 小积分：缓慢累积正向/反向力，消除小幅稳态偏差（如19.03的欠调）
    .kd = 4.4f,            // 4.5  // 小微分：抑制波动（19.03→21.14），增强稳定性
    .integral_max = 46.0f, // 大幅提高积分上限：允许积分项提供最大驱动力

    // 速度：
    .current = 0.0f,
    .error = 0.0f,
    .last_error = 0.0f,

    // pwm：
    .integral = 0.0f,
    .output = 0.0f};
SpeedPID_HandleTypeDef *bl_pid = &bl_pid_instance;

SpeedPID_HandleTypeDef br_pid_instance = {
    .kp = 15.0f,           // 比例增益
    .ki = 5.6f,            // 积分增益
    .kd = 4.5f,            // 微分增益
    .integral_max = 47.0f, // 积分上限

    .current = 0.0f,
    .error = 0.0f,
    .last_error = 0.0f,

    // pwm：
    .integral = 0.0f,
    .output = 0.0f};
SpeedPID_HandleTypeDef *br_pid = &br_pid_instance;

void SpeedPID_SetTarget(SpeedPID_HandleTypeDef *hspeed, float target)
{
    // 限制目标速度在安全范围
    if (target > SPEED_MAX_TARGET)
    {
        hspeed->target = SPEED_MAX_TARGET;
    }
    else if (target < SPEED_MIN_TARGET)
    {
        hspeed->target = SPEED_MIN_TARGET;
    }
    else
    {
        hspeed->target = target;
    }
}

void SpeedPID_SetCurrent(SpeedPID_HandleTypeDef *hspeed, float current)
{
    hspeed->current = current; // 从外部（如编码器）获取当前速度
}

float SpeedPID_Compute(SpeedPID_HandleTypeDef *hspeed)
{
    // 计算偏差
    hspeed->last_error = hspeed->error;
    hspeed->error = hspeed->target - hspeed->current;

    // 比例项
    float p_term = hspeed->kp * hspeed->error;

    // 积分项（限幅）
    hspeed->integral += hspeed->ki * hspeed->error;
    if (hspeed->integral > hspeed->integral_max)
    {
        hspeed->integral = hspeed->integral_max;
    }
    else if (hspeed->integral < -hspeed->integral_max)
    {
        hspeed->integral = -hspeed->integral_max;
    }

    // 微分项
    float d_term = hspeed->kd * (hspeed->error - hspeed->last_error);

    // 输出限幅（基础速度不小于0）
    hspeed->output = p_term + hspeed->integral + d_term;
    if (hspeed->output > SPEED_OUTPUT_MAX)
    {
        hspeed->output = SPEED_OUTPUT_MAX;
    }
    if (hspeed->output < -SPEED_OUTPUT_MAX)
    {
        hspeed->output = -SPEED_OUTPUT_MAX;
    }

    return hspeed->output;
}
