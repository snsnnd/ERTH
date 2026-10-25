#include "utils.h"
#include <stdint.h>
#include <math.h> // isnan：C99 中为 math.h 宏，缺失会退化为隐式函数声明（链接隐患）
#include "BT.h"

// 快速平方根（Quake III算法，单精度）
float fast_sqrt(float x)
{
    if (x <= 0.0f)
    {
#if DEBUG_PRINT
        log_print("utils->Fast square root input invalid: %.2f\r\n", x);
#endif
        return 0.0f;
    }
    float xhalf = 0.5f * x;
    uint32_t i = *(uint32_t *)&x;
    i = 0x5f3759df - (i >> 1); // 使用统一魔术数
    x = *(float *)&i;
    x = x * (1.5f - xhalf * x * x);
    return x * x; // 修正为返回 sqrt(x)
}

// 低通滤波器初始化
void low_pass_filter_init(LowPassFilter *filter, float alpha)
{
    if (filter == NULL)
    {
#if DEBUG_PRINT
        log_print("utils->The pointer of the low-pass filter is empty\r\n");
#endif
        return;
    }
    filter->alpha = (alpha < 0.0f) ? 0.0f : (alpha > 1.0f) ? 1.0f
                                                           : alpha;
    filter->last_output = 0.0f;
}

// 低通滤波器更新
float low_pass_filter_update(LowPassFilter *filter, float input)
{
    if (filter == NULL)
    {
#if DEBUG_PRINT
        log_print("utils->The pointer of the low-pass filter is empty, and the input value is returned: %.2f\r\n", input);
#endif
        return input;
    }
    if (isnan(input))
    {
#if DEBUG_PRINT
        log_print("utils->Low-pass filter input invalid: NaN\r\n");
#endif
        return filter->last_output;
    }
    filter->last_output = filter->alpha * input + (1.0f - filter->alpha) * filter->last_output;
    return filter->last_output;
}

// 移动平均滤波器初始化
void moving_average_init(MovingAverageFilter *filter, float *buffer, uint16_t size)
{
    if (filter == NULL || buffer == NULL || size == 0)
    {
#if DEBUG_PRINT
        log_print("utils->The moving average filter parameter is invalid: filter=%p, buffer=%p, size=%d\r\n",
                  filter, buffer, size);
#endif
        return;
    }
    // 验证 size 是否为 2 的幂
    if ((size & (size - 1)) != 0)
    {
#if DEBUG_PRINT
        log_print("utils->The size of the moving average filter should be a power of 2: size=%d\r\n", size);
#endif
        return;
    }
    filter->buffer = buffer;
    filter->size = size;
    filter->index = 0;
    filter->sum = 0.0f;
    for (uint16_t i = 0; i < size; i++)
    {
        buffer[i] = 0.0f;
    }
}

// 移动平均滤波器更新
float moving_average_update(MovingAverageFilter *filter, float new_value)
{
    if (filter == NULL || filter->buffer == NULL || filter->size == 0)
    {
#if DEBUG_PRINT
        log_print("utils->The moving average filter parameter is invalid. Return the input value: %.2f\r\n", new_value);
#endif
        return new_value;
    }
    if (isnan(new_value))
    {
#if DEBUG_PRINT
        log_print("utils->The input of the moving average filter is invalid: NaN\r\n");
#endif
        return filter->sum / filter->size;
    }
    filter->sum -= filter->buffer[filter->index];
    filter->sum += new_value;
    filter->buffer[filter->index] = new_value;
    filter->index = (filter->index + 1) & (filter->size - 1);
    return filter->sum / (float)filter->size;
}

// 欧氏距离计算
float calculate_distance(float x1, float y1, float x2, float y2)
{
    if (isnan(x1) || isnan(y1) || isnan(x2) || isnan(y2))
    {
#if DEBUG_PRINT
        log_print("utils->The input for distance calculation is invalid: (%.2f, %.2f) -> (%.2f, %.2f)\r\n", x1, y1, x2, y2);
#endif
        return 0.0f;
    }
    float dx = x2 - x1;
    float dy = y2 - y1;
    // sqrtf（arm_sqrt_f32 已删：F103C8T6 无 FPU，工程不带 CMSIS-DSP）
    return sqrtf(dx * dx + dy * dy);
}
