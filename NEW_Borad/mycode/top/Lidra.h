#ifndef LIDRA_H
#define LIDRA_H

#include "stm32f4xx.h"
#include <stdint.h>
#include "utils.h"

// 模块配置参数
#define RADAR_MIN_RANGE 0.1f     // 最小有效距离（米）
#define RADAR_MAX_RANGE 10.0f    // 最大有效距离（米）
#define ANGLE_STEP 0.8f          // 角度步进（度）
#define MAX_POINTS_PER_FRAME 450 // 单圈最大点数（360/0.8=450）
#define NOISE_FILTER_RADIUS 0.3f // 滤波半径（米）
#define NOISE_MIN_NEIGHBORS 3    // 最小邻居数
#define MIN_VALID_POINTS 5       // 质心计算最小点数

// 目标位置结构体（直角坐标）
typedef struct
{
    float x; // x坐标（米）
    float y; // y坐标（米）
} TargetPosition;

// 质心结果结构体
typedef struct
{
    float centroid_x;   // 质心x坐标（米）
    float centroid_y;   // 质心y坐标（米）
    uint16_t point_num; // 参与计算的点数
    // float rotation_speed; // 移除未使用字段，或保留待实现
} SingleMassPoint;

// 模块初始化函数
void FilterModule_Init(void);

// 主处理函数（输入距离/角度，输出质心）
// distance: 雷达距离（米）
// angle: 雷达角度（度）
// 返回值：0-成功；1-点云不足；2-无效距离；3-缓存溢出；4-未完成一圈
uint8_t FilterModule_Process(float distance, float angle, SingleMassPoint *mass_point);

#endif // LIDRA_H
