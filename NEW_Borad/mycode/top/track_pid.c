#include "track_pid.h"
#include "motor.h" // PWM_MAX（TIM3 ARR=999 → 满量程 999）
#include "sensor.h"
#include "math.h"
#include "BT.h"

/* ==================== 循迹外环：参考车的"加权位置 + 差速目标"====================
 * 参考车 mishiduobao_f103 的 6 路权重（L3..R3）：-25, -18, -9, +11, +18, +26
 * 本车 8 路按同量级扩展成对称权重（中间两路为 0 交叉点）：
 *   bit0 最左 … bit7 最右
 * 算法：只统计"压到黑线"的通道，取权重平均作为偏差，
 *       再按 ±100 CPS/级 折算成左右轮目标速度差，最后按基础 PWM 映射成占空比。
 * 不丢线时 count==0 → 偏差 0 → 直行（与参考车一致）。
 */
static const int16_t sensor_weight[8] = {-25, -18, -11, -4, 4, 11, 18, 25};

static float constrain(float value, float min_val, float max_val)
{
    if (value < min_val)
    {
        return min_val;
    }
    else if (value > max_val)
    {
        return max_val;
    }
    else
    {
        return value;
    }
}

/* 保留的 PD 实例：当前循迹不走闭环（与参考车一致的开环差速），
 * 后续接 speed_pid 闭环时可复用。 */
TrackPI_HandleTypeDef pi_instance = {
    .kp = 9.5f, // 比例系数
    .ki = 1.0f, // 积分系数
    .kd = 2.8f, // 微分系数
    .output_max = 30.0f,
    .integral_max = 6.0f,
    .error = 0.0f,
    .last_error = 0.0f, // 初始化上一周期误差
    .integral = 0.0f,
    .derivative = 0.0f,
    .output = 0.0f};
TrackPI_HandleTypeDef *hpi = &pi_instance;

float Track_CalcError(uint16_t sensor_data)
{
    int16_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (sensor_data & (1u << i)) // 1 = 压黑线
        {
            sum += sensor_weight[i];
            count++;
        }
    }

    float error = 0.0f;
    if (count == 0)
    {
        error = 0.0f;
    }
    else
    {
        error = (float)sum / count;
    }

// 逐位打印 8 路状态：每控制周期 9 行，属刷屏级日志，默认关闭
#if DEBUG_PRINT_VERBOSE
    for (int i = 0; i < 8; i++)
    {
        uint8_t state = (sensor_data >> i) & 1;
        log_print("sensor%d: %d, ", i, state);
    }
    log_print("error=%.2f (sensor=%u, count=%u, sum=%d)\r\n", error, sensor_data, count, sum);
#endif

    return error;
}

float TrackPI_Compute(float target, float current)
{
    hpi->last_error = hpi->error; // 保存上一周期误差
    hpi->error = target - current;

    // 比例项
    float p_term = hpi->kp * hpi->error;

    // 积分项（保留之前的衰减逻辑）
    if (fabs(hpi->error) > 0.3f)
    {
        hpi->integral += hpi->ki * hpi->error;
        hpi->integral = constrain(hpi->integral, -hpi->integral_max, hpi->integral_max);
    }
    else
    {
        hpi->integral *= 0.7f;
    }
    float i_term = hpi->integral;

    // 微分项（基于误差变化率）
    hpi->derivative = hpi->error - hpi->last_error; // 误差变化量（简化版，也可除以周期时间）
    float d_term = hpi->kd * hpi->derivative;

    // 总输出
    hpi->output = p_term + i_term + d_term;
    hpi->output = constrain(hpi->output, -hpi->output_max, hpi->output_max);
    return hpi->output;
}

void Track_ControlMotor(float error, int *left_pwm, int *right_pwm)
{
    int32_t diff = (int32_t)(error * (float)LINE_DIFF_CPS_PER_LEVEL);

    if (diff > LINE_DIFF_CPS_MAX)
        diff = LINE_DIFF_CPS_MAX;
    if (diff < -LINE_DIFF_CPS_MAX)
        diff = -LINE_DIFF_CPS_MAX;

    int32_t target_left = (int32_t)LINE_BASE_CPS + diff;
    int32_t target_right = (int32_t)LINE_BASE_CPS - diff;

    if (target_left > LINE_TARGET_CPS_MAX)
        target_left = LINE_TARGET_CPS_MAX;
    if (target_left < LINE_TARGET_CPS_MIN)
        target_left = LINE_TARGET_CPS_MIN;
    if (target_right > LINE_TARGET_CPS_MAX)
        target_right = LINE_TARGET_CPS_MAX;
    if (target_right < LINE_TARGET_CPS_MIN)
        target_right = LINE_TARGET_CPS_MIN;

    // 目标速度 → 占空比（与参考车 DifferentialTargetToPwm() 同一映射）
    int32_t pwm_l = target_left * LINE_LEFT_BASE_PWM / LINE_BASE_CPS;
    int32_t pwm_r = target_right * LINE_RIGHT_BASE_PWM / LINE_BASE_CPS;

    if (pwm_l > PWM_MAX)
        pwm_l = PWM_MAX;
    if (pwm_r > PWM_MAX)
        pwm_r = PWM_MAX;

    *left_pwm = (int)pwm_l;
    *right_pwm = (int)pwm_r;

#if DEBUG_PRINT_VERBOSE
    log_print("trace: err=%.2f diff=%ld tl=%ld tr=%ld pwmL=%ld pwmR=%ld\r\n",
              error, (long)diff, (long)target_left, (long)target_right,
              (long)pwm_l, (long)pwm_r);
#endif
}

uint8_t Track_MainControl(int *left_pwm, int *right_pwm)
{
    uint16_t sensor_data = HD_Parallel_Read(); // 1 = 压黑线

    if (left_pwm == NULL || right_pwm == NULL)
    {
        return (uint8_t)sensor_data;
    }

    // 全白表示丢线：停车等待重新检测到黑线，避免无反馈继续盲走。
    if (sensor_data == TRACK_ALL_WHITE)
    {
        *left_pwm = 0;
        *right_pwm = 0;
        return (uint8_t)sensor_data;
    }

    // 全黑（校准点/横向色带）由 FSM 的校准状态机处理；其余情况继续循迹。
    float error = Track_CalcError(sensor_data);
    Track_ControlMotor(error, left_pwm, right_pwm);

    return (uint8_t)sensor_data;
}
