#include "track_pid.h"
#include "math.h"
#include "BT.h"

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

static const int8_t sensor_weight[8] = {-3, -2, -1, 0, 0, 1, 2, 3};

float Track_CalcError(uint16_t sensor_data)
{
    int16_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (sensor_data & (1 << i))
        {
            sum += sensor_weight[i];
            count++;
        }
    }
#if DEBUG_PRINT
    log_print("track_data:%u\n", sensor_data);
    for (int i = 0; i < 8; i++)
    {
        uint8_t state = (sensor_data >> i) & 1; // 提取每一位状态
        log_print("sensor%d: %d, ", i, state);  // 打印左侧到右侧的传感器状态
    }
#endif

    float error = 0.0f;
    if (count == 0)
    {
        error = 0.0f;
    }
    else
    {
        error = (float)sum / count;
    }

// 调试输出传感器数据和计算的偏差
#if DEBUG_PRINT
    log_print("count=%u, sum=%d, error=%.2f\n",
              sensor_data, count, sum, error);
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

void Track_ControlMotor(float pi_output, int *left_speed, int *right_speed)
{
    *left_speed = BASE_SPEED + pi_output;
    *right_speed = BASE_SPEED - pi_output;

    if (*left_speed > MAX_SPEED)
        *left_speed = MAX_SPEED;
    if (*left_speed < MIN_SPEED)
        *left_speed = MIN_SPEED;
    if (*right_speed > MAX_SPEED)
        *right_speed = MAX_SPEED;
    if (*right_speed < MIN_SPEED)
        *right_speed = MIN_SPEED;

// 调试输出电机速度控制参数
#if DEBUG_PRINT
    log_print("Track_ControlMotor: pi_output=%.2f, left_speed=%d, right_speed=%d\n",
              pi_output, *left_speed, *right_speed);
#endif
}

#include "motor.h"
uint8_t Track_MainControl(int *left_speed, int *right_speed)
{
    uint16_t sensor_data = HD_Serial_Read();
    if (sensor_data == 255)
    {
        motor_stop();
        return 0;
    }
    float error = Track_CalcError(sensor_data);
    float pi_output = TrackPI_Compute(0, error);
    Track_ControlMotor(pi_output, left_speed, right_speed);

    return sensor_data;
}
