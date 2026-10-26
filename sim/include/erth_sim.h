#ifndef ERTH_SIM_H
#define ERTH_SIM_H

#include <stdint.h>

#include "erth_config.h"

/*
 * 闭环椭圆赛道 + 差速两驱 + 8 路数字灰度 + 正交编码器。
 *
 * 坐标约定：x 向右、y 向上、theta 逆时针为正（数学系，不是屏幕系）。
 * 灰度位序：bit0 = 最左探头（y 正方向），bit7 = 最右。
 *
 * 这个模型只用来产生教师数据与做 A/B，不追求替代实车；所有参数在
 * erth_config.h 里，标 ★ 的需要实测替换。
 */

#define ERTH_TRACK_POINTS 4096

typedef struct {
    float x;
    float y;
    float tx; /* 单位切向 */
    float ty;
} erth_track_pt_t;

typedef struct {
    /* 赛道 */
    erth_track_pt_t pts[ERTH_TRACK_POINTS];
    float total_len;

    /* 位姿与执行器状态 */
    float x, y, theta;
    float v_left, v_right;

    /* 编码器累计距离与上次计数 */
    float dist_left, dist_right;
    int32_t count_left, count_right;
    int32_t mark_left, mark_right;
    float counts_per_meter;

    /* 上次最近点索引，用于局部搜索 */
    int hint;

    /* 随机源（噪声/丢线注入），0 表示未播种 */
    uint32_t rng;
} erth_sim_t;

typedef struct {
    uint8_t mask;      /* 8 路数字量，bit i = 探头 i */
    float lateral_m;   /* 车心到线的横向距离，左正右负 */
    float heading_err; /* 车头与赛道切向的夹角 */
    float speed_mps;
} erth_obs_t;

typedef struct {
    uint8_t mask;
    float dt_s;
} erth_sensor_in_t;

void erth_sim_init(erth_sim_t *s, float x0, float y0, float theta0, uint32_t seed);

/* 把探头位置换算成 8 位数字量；noise 与 dropout 由 seed 驱动。 */
uint8_t erth_sim_sensors(erth_sim_t *s, float noise_m, float dropout_prob);

void erth_sim_read(erth_sim_t *s, erth_obs_t *out);

/* pwm 为 -ERTH_PWM_MAX..+ERTH_PWM_MAX。 */
void erth_sim_step(erth_sim_t *s, float pwm_left, float pwm_right, float dt_s);

/* 编码器：锁存当前计数（等价于固件「读取即清零」），再按窗口求速度。 */
void erth_sim_encoder_latch(erth_sim_t *s);
float erth_sim_encoder_left(const erth_sim_t *s, float window_s);
float erth_sim_encoder_right(const erth_sim_t *s, float window_s);

/* 沿赛道弧长的航向（用于构造初始位姿）。 */
void erth_track_pose_at(const erth_sim_t *s, float t_norm, float lateral_m,
                        float *x, float *y, float *theta);

#endif
