#include "erth_sim.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float wrap_pi(float a)
{
    while (a > (float)M_PI) {
        a -= 2.0f * (float)M_PI;
    }
    while (a < -(float)M_PI) {
        a += 2.0f * (float)M_PI;
    }
    return a;
}

static uint32_t next_rand(uint32_t *state)
{
    uint32_t x = *state;
    if (x == 0u) {
        x = 0x9e3779b9u;
    }
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static float rand_unit(uint32_t *state)
{
    return (float)(next_rand(state) >> 8) * (1.0f / 16777216.0f);
}

void erth_sim_init(erth_sim_t *s, float x0, float y0, float theta0, uint32_t seed)
{
    memset(s, 0, sizeof(*s));

    s->x = x0;
    s->y = y0;
    s->theta = theta0;
    s->rng = (seed != 0u) ? seed : 0x9e3779b9u;
    s->hint = 0;

    const float counts_per_rev =
        (float)ERTH_ENCODER_PPR * (float)ERTH_ENCODER_QUAD * ERTH_GEAR_RATIO;
    s->counts_per_meter = counts_per_rev /
                          (2.0f * (float)M_PI * ERTH_WHEEL_RADIUS_M);

    /* 椭圆折线：等参数采样，弧长由相邻点累加。 */
    float len = 0.0f;
    for (int i = 0; i < ERTH_TRACK_POINTS; ++i) {
        const float t = 2.0f * (float)M_PI * (float)i / (float)ERTH_TRACK_POINTS;
        s->pts[i].x = ERTH_SEMI_MAJOR_M * cosf(t);
        s->pts[i].y = ERTH_SEMI_MINOR_M * sinf(t);
    }
    for (int i = 0; i < ERTH_TRACK_POINTS; ++i) {
        const int nxt = (i + 1) % ERTH_TRACK_POINTS;
        const int prv = (i + ERTH_TRACK_POINTS - 1) % ERTH_TRACK_POINTS;
        float dx = s->pts[nxt].x - s->pts[prv].x;
        float dy = s->pts[nxt].y - s->pts[prv].y;
        const float norm = sqrtf(dx * dx + dy * dy);
        s->pts[i].tx = (norm > 1.0e-9f) ? dx / norm : 1.0f;
        s->pts[i].ty = (norm > 1.0e-9f) ? dy / norm : 0.0f;
        len += sqrtf((s->pts[i].x - s->pts[prv].x) * (s->pts[i].x - s->pts[prv].x) +
                     (s->pts[i].y - s->pts[prv].y) * (s->pts[i].y - s->pts[prv].y));
    }
    s->total_len = len;
}

void erth_track_pose_at(const erth_sim_t *s, float t_norm, float lateral_m,
                        float *x, float *y, float *theta)
{
    int idx = (int)(t_norm * (float)ERTH_TRACK_POINTS);
    idx %= ERTH_TRACK_POINTS;
    if (idx < 0) {
        idx += ERTH_TRACK_POINTS;
    }

    const erth_track_pt_t *p = &s->pts[idx];
    const float nx = -p->ty;
    const float ny = p->tx;

    if (x != NULL) {
        *x = p->x + lateral_m * nx;
    }
    if (y != NULL) {
        *y = p->y + lateral_m * ny;
    }
    if (theta != NULL) {
        *theta = atan2f(p->ty, p->tx);
    }
}

/* 局部最近点搜索：围绕上次索引，窗口足够覆盖探头到车心的距离。 */
static int nearest(const erth_sim_t *s, int hint, float px, float py, float *dist_out)
{
    const int window = 128;
    int best = hint;
    float best_d2 = 1.0e30f;

    for (int k = -window; k <= window; ++k) {
        int i = (hint + k) % ERTH_TRACK_POINTS;
        if (i < 0) {
            i += ERTH_TRACK_POINTS;
        }
        const float dx = px - s->pts[i].x;
        const float dy = py - s->pts[i].y;
        const float d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best = i;
        }
    }

    if (dist_out != NULL) {
        *dist_out = sqrtf(best_d2);
    }
    return best;
}

static float lateral_at(const erth_sim_t *s, int idx, float px, float py)
{
    const erth_track_pt_t *p = &s->pts[idx];
    const float vx = px - p->x;
    const float vy = py - p->y;
    /* 左法向 (-ty, tx)，正 = 偏左 */
    return vx * (-p->ty) + vy * p->tx;
}

uint8_t erth_sim_sensors(erth_sim_t *s, float noise_m, float dropout_prob)
{
    uint8_t mask = 0u;
    const float fwd_x = cosf(s->theta);
    const float fwd_y = sinf(s->theta);
    const float left_x = -fwd_y;
    const float left_y = fwd_x;
    const float half = 0.5f * ERTH_LINE_WIDTH_M;

    for (int i = 0; i < ERTH_SENSOR_COUNT; ++i) {
        const float offset = ERTH_SENSOR_SPACING_M *
                             (0.5f * (float)(ERTH_SENSOR_COUNT - 1) - (float)i);
        const float px = s->x + fwd_x * ERTH_SENSOR_TO_AXLE_M + left_x * offset;
        const float py = s->y + fwd_y * ERTH_SENSOR_TO_AXLE_M + left_y * offset;

        /* 探头沿车身方向搜索，索引取车心附近即可。 */
        const int idx = nearest(s, s->hint, px, py, NULL);
        const float d = lateral_at(s, idx, px, py);

        float d_noisy = d;
        if (noise_m > 0.0f) {
            /* Digital modules jitter around the switching point, so the noise
             * belongs on the measured distance, not on the threshold. */
            d_noisy += noise_m * (2.0f * rand_unit(&s->rng) - 1.0f);
        }

        int on_line = (fabsf(d_noisy) <= half) ? 1 : 0;
        if (dropout_prob > 0.0f && rand_unit(&s->rng) < dropout_prob) {
            on_line = 0;
        }

        /* The bit reports the raw level: 0 on the line, 1 off it. */
        const int level = (on_line != 0) ? ERTH_SENSOR_LEVEL_ON_LINE : 1;
        if (level != 0) {
            mask |= (uint8_t)(1u << i);
        }
    }

    return mask;
}

void erth_sim_read(erth_sim_t *s, erth_obs_t *out)
{
    s->hint = nearest(s, s->hint, s->x, s->y, NULL);

    out->mask = 0u;
    out->lateral_m = lateral_at(s, s->hint, s->x, s->y);
    out->heading_err = wrap_pi(atan2f(s->pts[s->hint].ty, s->pts[s->hint].tx) - s->theta);
    out->speed_mps = 0.5f * (s->v_left + s->v_right);
}

static float wheel_step(float v, float pwm, float gain, float dt_s)
{
    const float duty = pwm / (float)ERTH_PWM_MAX;
    const float mag = fabsf(duty);
    const float db = ERTH_DEADBAND;
    float drive = 0.0f;

    if (mag > db) {
        drive = (mag - db) / (1.0f - db);
        drive = (duty > 0.0f) ? drive : -drive;
    }

    const float target = gain * ERTH_V_MAX_MPS * drive;
    const float alpha = 1.0f - expf(-dt_s / ERTH_TAU_S);

    return v + alpha * (target - v);
}

void erth_sim_step(erth_sim_t *s, float pwm_left, float pwm_right, float dt_s)
{
    s->v_left = wheel_step(s->v_left, pwm_left, ERTH_GAIN_LEFT, dt_s);
    s->v_right = wheel_step(s->v_right, pwm_right, ERTH_GAIN_RIGHT, dt_s);

    const float v = 0.5f * (s->v_left + s->v_right);
    const float omega = (s->v_right - s->v_left) / ERTH_TRACK_WIDTH_M;

    s->x += v * cosf(s->theta) * dt_s;
    s->y += v * sinf(s->theta) * dt_s;
    s->theta = wrap_pi(s->theta + omega * dt_s);

    s->dist_left += s->v_left * dt_s;
    s->dist_right += s->v_right * dt_s;
    s->count_left = (int32_t)floorf(s->dist_left * s->counts_per_meter);
    s->count_right = (int32_t)floorf(s->dist_right * s->counts_per_meter);
}

float erth_sim_encoder_left(const erth_sim_t *s, float window_s)
{
    return (float)(s->count_left - s->mark_left) /
           (s->counts_per_meter * window_s);
}

float erth_sim_encoder_right(const erth_sim_t *s, float window_s)
{
    return (float)(s->count_right - s->mark_right) /
           (s->counts_per_meter * window_s);
}

void erth_sim_encoder_latch(erth_sim_t *s)
{
    s->mark_left = s->count_left;
    s->mark_right = s->count_right;
}
