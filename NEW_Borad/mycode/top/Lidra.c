#include "Lidra.h"
#include <math.h>
#include "utils.h"
#include "arm_math.h"
#include "stdbool.h"
#include "BT.h"

// 静态全局变量
static float cos_table[MAX_POINTS_PER_FRAME];
static float sin_table[MAX_POINTS_PER_FRAME];
static TargetPosition frame_buffer[MAX_POINTS_PER_FRAME];
static uint16_t frame_count = 0;
static bool frame_valid = false; // 点云帧有效性标志

// 预计算三角函数表
static void PrecomputeTrigTables(void)
{
    for (uint16_t i = 0; i < MAX_POINTS_PER_FRAME; i++)
    {
        float angle_rad = (i * ANGLE_STEP) * PI_F / 180.0f;
        cos_table[i] = arm_cos_f32(angle_rad); // 使用CMSIS-DSP加速
        sin_table[i] = arm_sin_f32(angle_rad);
    }
}

// 极坐标转直角坐标
static inline void PolarToCartesian(float distance, float angle, TargetPosition *pos)
{
    if (pos == NULL || isnan(distance) || isnan(angle))
    {
#if DEBUG_PRINT
        log_print("Lidra->input of PolarToCartesian is useless\n");
        log_print("distance=%.2f, angle=%.2f\r\n", distance, angle);
#endif
        return;
    }
    uint16_t idx = (uint16_t)(angle / ANGLE_STEP) % MAX_POINTS_PER_FRAME;
    pos->x = distance * cos_table[idx];
    pos->y = distance * sin_table[idx];
}

// 点云滤波（基于DBSCAN的简单噪声滤波）
static inline void FilterPointCloud(TargetPosition *in, uint16_t in_len,
                                    TargetPosition *out, uint16_t *out_len)
{
    *out_len = 0;
    if (in == NULL || out == NULL || out_len == NULL || in_len < NOISE_MIN_NEIGHBORS)
    {
#if DEBUG_PRINT
        log_print("Lidra->parmas of FilterPointCloud is useless\n");
        log_print("in=%p, out=%p, out_len=%p, in_len=%d\r\n",
                  in, out, out_len, in_len);
#endif
        return;
    }

    const float radius_sq = NOISE_FILTER_RADIUS * NOISE_FILTER_RADIUS;

    for (uint16_t i = 0; i < in_len; i++)
    {
        const float cx = in[i].x;
        const float cy = in[i].y;
        uint8_t neighbors = 0;

        for (uint16_t j = 0; j < in_len; j++)
        {
            if (i == j)
                continue;
            const float dx = cx - in[j].x;
            const float dy = cy - in[j].y;
            const float dist_sq = dx * dx + dy * dy;
            if (dist_sq <= radius_sq)
            {
                neighbors++;
                if (neighbors >= NOISE_MIN_NEIGHBORS)
                    break;
            }
        }

        if (neighbors >= NOISE_MIN_NEIGHBORS)
        {
            out[(*out_len)++] = in[i];
        }
    }
}

// 质心计算
static inline uint8_t CalculateCentroid(TargetPosition *filtered, uint16_t len, SingleMassPoint *result)
{
    if (filtered == NULL || result == NULL)
    {
#if DEBUG_PRINT
        log_print("Lidra->parmas of CalculateCentroid is null\r\n");
#endif
        return 1;
    }
    if (len < MIN_VALID_POINTS)
    {
        result->point_num = 0;
#if DEBUG_PRINT
        log_print("Lidra->point_num is such few\n");
        log_print("len=%d, min=%d\r\n", len, MIN_VALID_POINTS);
#endif
        return 1;
    }

    float sum_x = 0.0f, sum_y = 0.0f;
    for (uint16_t i = 0; i < len; i++)
    {
        if (isnan(filtered[i].x) || isnan(filtered[i].y))
        {
#if DEBUG_PRINT
            log_print("Lidra->point_date is useless\n");
            log_print("index=%d, x=%.2f, y=%.2f\r\n", i, filtered[i].x, filtered[i].y);
#endif
            continue;
        }
        sum_x += filtered[i].x;
        sum_y += filtered[i].y;
    }

    const float inv_len = 1.0f / (float)len;
    result->centroid_x = sum_x * inv_len;
    result->centroid_y = sum_y * inv_len;
    result->point_num = len;
    // result->rotation_speed = 0.0f; // 待用户实现

    if (isnan(result->centroid_x) || isnan(result->centroid_y))
    {
#if DEBUG_PRINT
        log_print("Lidra->result of CalculateCentroid is useless\n");
        log_print("x=%.2f, y=%.2f\r\n", result->centroid_x, result->centroid_y);
#endif
        return 1;
    }
    return 0;
}

// 模块初始化
void FilterModule_Init(void)
{
    PrecomputeTrigTables();
    frame_count = 0;
    frame_valid = false;
    for (uint16_t i = 0; i < MAX_POINTS_PER_FRAME; i++)
    {
        frame_buffer[i].x = 0.0f;
        frame_buffer[i].y = 0.0f;
    }
}

// 主处理函数
uint8_t FilterModule_Process(float distance, float angle, SingleMassPoint *mass_point)
{
    if (mass_point == NULL)
    {
#if DEBUG_PRINT
        log_print("Lidra->mass_point is null\r\n");
#endif
        return 1;
    }

    // 过滤无效距离
    if (distance < RADAR_MIN_RANGE || distance > RADAR_MAX_RANGE || isnan(distance))
    {
#if DEBUG_PRINT
        log_print("Lidra->distance is useless\n");
        log_print("%.2f, min=%.2f, max=%.2f\r\n",
                  distance, RADAR_MIN_RANGE, RADAR_MAX_RANGE);
#endif
        return 2;
    }

    // 检查角度范围（0~360度）
    if (angle < 0.0f || angle >= 360.0f || isnan(angle))
    {
#if DEBUG_PRINT
        log_print("Lidra->angle is useless\n");
        log_print("%.2f\r\n", angle);
#endif
        return 2;
    }

    // 重置帧（新圈开始）
    if (angle < ANGLE_STEP && frame_valid)
    {
        frame_count = 0;
        frame_valid = false;
        for (uint16_t i = 0; i < MAX_POINTS_PER_FRAME; i++)
        {
            frame_buffer[i].x = 0.0f;
            frame_buffer[i].y = 0.0f;
        }
    }

    // 极坐标转直角坐标
    TargetPosition current_pos;
    PolarToCartesian(distance, angle, &current_pos);

    // 缓存点云
    if (frame_count < MAX_POINTS_PER_FRAME)
    {
        frame_buffer[frame_count++] = current_pos;
        frame_valid = true;
    }
    else
    {
        frame_count = 0;
        frame_valid = false;
#if DEBUG_PRINT
        log_print("Lidra->cloud_point is such more\r\n");
#endif
        return 3;
    }

    // 单圈点云收集完成
    if (frame_count == MAX_POINTS_PER_FRAME)
    {
        TargetPosition filtered_points[MAX_POINTS_PER_FRAME];
        uint16_t filtered_len = 0;

        FilterPointCloud(frame_buffer, MAX_POINTS_PER_FRAME, filtered_points, &filtered_len);
        uint8_t calc_ret = CalculateCentroid(filtered_points, filtered_len, mass_point);

        frame_count = 0;
        frame_valid = false;
        return calc_ret ? 1 : 0;
    }

    return 4; // 未完成一圈
}
