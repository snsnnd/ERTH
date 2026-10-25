#ifndef UTILS_H
#define UTILS_H

#include "stm32f1xx.h"
#include <stdint.h>
// arm_math.h 已删：STM32F103C8T6（Cortex-M3，无 FPU）工程不带 CMSIS-DSP

// 地图比例尺：1 地图单位 = 0.01 米（即 1 厘米）
#define MAP_SCALE 0.01f

// 单精度数学常量
#define PI_F 3.14159265358979323846f
#define TWO_PI_F (2.0f * PI_F)
#define HALF_PI_F (0.5f * PI_F)

// 低通滤波器结构体
typedef struct
{
    float alpha;       // 滤波系数（0~1）
    float last_output; // 上一次输出值
} LowPassFilter;

// 移动平均滤波器结构体（缓冲区需4字节对齐）
typedef struct
{
    float *buffer;  // 数据缓冲区（4字节对齐）
    uint16_t size;  // 大小（需为2的幂：8/16/32）
    uint16_t index; // 当前索引
    float sum;      // 缓冲区总和
} MovingAverageFilter;

// 工具模块接口
float fast_sqrt(float x);
void low_pass_filter_init(LowPassFilter *filter, float alpha);
float low_pass_filter_update(LowPassFilter *filter, float input);
void moving_average_init(MovingAverageFilter *filter, float *buffer, uint16_t size);
float moving_average_update(MovingAverageFilter *filter, float new_value);
float calculate_distance(float x1, float y1, float x2, float y2);

static inline float real_to_map(float real_dist) { return real_dist / MAP_SCALE; }
static inline float map_to_real(float map_dist) { return map_dist * MAP_SCALE; }

#endif
