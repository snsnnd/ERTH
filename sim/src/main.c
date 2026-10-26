#include "erth_sim.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * 现有循迹 PID 的对照台架。
 *
 * 控制律与固件 track_pid.c 一一对应：8 位数字量的加权均值误差 → PID →
 * 左右目标速度（BASE ± out，clamp 到 [MIN, MAX]）→ ×3 作为 PWM。
 * 这里只关心"换掉它之前，它到底有多好"，所以不做任何滤波或平滑。
 */

#define PWM_MAX ERTH_PWM_MAX

typedef struct {
    float err;
    float last_err;
    float integral;
    float out;
} pid_t;

/* 固件原值，见 track_pid.c 与 erth_config.h。 */
static const int8_t k_weight[8] = {-3, -2, -1, 0, 0, 1, 2, 3};

typedef struct {
    float rms;
    float max_lat;
    float mean_speed;
    float offline_frac;
    int samples;
} result_t;

static float track_error(uint8_t mask)
{
    int sum = 0;
    int count = 0;

    for (int i = 0; i < ERTH_SENSOR_COUNT; ++i) {
        if (mask & (1u << i)) {
            sum += k_weight[i];
            ++count;
        }
    }

    return (count == 0) ? 0.0f : (float)sum / (float)count;
}

static float pid_step(pid_t *pid)
{
    if (fabsf(pid->err) > 0.3f) {
        pid->integral += ERTH_PID_KI * pid->err;
        pid->integral = fmaxf(fminf(pid->integral, ERTH_PID_I_MAX), -ERTH_PID_I_MAX);
    } else {
        pid->integral *= 0.7f;
    }

    const float d = pid->err - pid->last_err;
    pid->last_err = pid->err;

    float out = ERTH_PID_KP * pid->err + pid->integral + ERTH_PID_KD * d;
    out = fmaxf(fminf(out, ERTH_PID_OUT_MAX), -ERTH_PID_OUT_MAX);
    pid->out = out;
    return out;
}

static void run(const char *label, float noise_m, float dropout,
                float x0, float y0, float theta0, uint32_t seed, result_t *res)
{
    erth_sim_t sim;
    erth_sim_init(&sim, x0, y0, theta0, seed);

    pid_t pid;
    memset(&pid, 0, sizeof(pid));

    const float dt = ERTH_DT_S;
    const int ticks = (int)(ERTH_DURATION_S / dt + 0.5f);

    float sum_sq = 0.0f;
    float sum_speed = 0.0f;
    float max_lat = 0.0f;
    int offline = 0;
    int counted = 0;

    for (int k = 0; k < ticks; ++k) {
        erth_obs_t obs;
        erth_sim_read(&sim, &obs);

        const uint8_t mask = erth_sim_sensors(&sim, noise_m, dropout);
        const bool count_it = k > (int)(0.5f / dt);

        if (mask == 0xFFu) {
            /* 全白 = 丢线；固件在这里停车，仿真同样停。 */
            if (count_it) {
                offline += 1;
                counted += 1;
            }
            erth_sim_step(&sim, 0.0f, 0.0f, dt);
            continue;
        }

        /* 固件：error = -mean(置位探头)，out = PID(error)。 */
        pid.err = -track_error(mask);
        const float out = pid_step(&pid);

        float left = ERTH_BASE_SPEED + out;
        float right = ERTH_BASE_SPEED - out;
        left = fmaxf(fminf(left, ERTH_MAX_SPEED), ERTH_MIN_SPEED);
        right = fmaxf(fminf(right, ERTH_MAX_SPEED), ERTH_MIN_SPEED);

        /* open-loop 增益把 10..30 映射成 30..90 的 PWM。 */
        erth_sim_step(&sim, left * ERTH_OPEN_LOOP_GAIN,
                      right * ERTH_OPEN_LOOP_GAIN, dt);

        if (count_it) {
            sum_sq += obs.lateral_m * obs.lateral_m;
            sum_speed += obs.speed_mps;
            max_lat = fmaxf(max_lat, fabsf(obs.lateral_m));
            counted += 1;
        }

        if (getenv("ERTH_TRACE") != NULL && k % 25 == 0 && k < 400) {
            fprintf(stderr, "k=%3d mask=%02x err=%+.2f out=%+.1f lat=%+.4f v=%.3f\n", k,
                    mask, (double)pid.err, (double)out, (double)obs.lateral_m,
                    (double)obs.speed_mps);
        }
    }

    res->rms = (counted > 0) ? sqrtf(sum_sq / (float)counted) : 0.0f;
    res->max_lat = max_lat;
    res->mean_speed = (counted > 0) ? sum_speed / (float)counted : 0.0f;
    res->offline_frac = (counted > 0) ? (float)offline / (float)counted : 0.0f;
    res->samples = counted;

    printf("%-14s rms=%.4f  max=%.4f  v=%.3f  off=%5.1f%%\n", label,
           (double)res->rms, (double)res->max_lat, (double)res->mean_speed,
           (double)(res->offline_frac * 100.0f));
}

int main(int argc, char **argv)
{
    bool check = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--check") == 0) {
            check = true;
        } else {
            fprintf(stderr, "usage: %s [--check]\n", argv[0]);
            return 2;
        }
    }

    erth_sim_t probe;
    erth_sim_init(&probe, 0.0f, 0.0f, 0.0f, 1u);

    printf("ellipse %.2f x %.2f m, length %.2f m, line %.0f mm, sensors %d x %.0f mm\n",
           (double)ERTH_SEMI_MAJOR_M, (double)ERTH_SEMI_MINOR_M,
           (double)probe.total_len, (double)(ERTH_LINE_WIDTH_M * 1000.0f),
           ERTH_SENSOR_COUNT, (double)(ERTH_SENSOR_SPACING_M * 1000.0f));
    printf("encoder %.0f counts/m (%.2f mm/count), pwm deadband %.3f\n",
           (double)probe.counts_per_meter,
           (double)(1000.0f / probe.counts_per_meter), (double)ERTH_DEADBAND);

    float x0 = 0.0f;
    float y0 = 0.0f;
    float th0 = 0.0f;
    erth_track_pose_at(&probe, 0.0f, 0.0f, &x0, &y0, &th0);

    result_t a;
    result_t b;
    result_t c;

    run("centred", 0.0f, 0.0f, x0, y0, th0, 1u, &a);

    erth_track_pose_at(&probe, 0.0f, 0.020f, &x0, &y0, &th0);
    run("offset 20mm", 0.0f, 0.0f, x0, y0, th0, 1u, &b);

    erth_track_pose_at(&probe, 0.0f, 0.0f, &x0, &y0, &th0);
    run("noise+dropout", 0.004f, 0.05f, x0, y0, th0, 7u, &c);

    if (check) {
        /* The baseline must at least hold the line; otherwise the plant or the
         * controller replica is wrong and the NN comparison would be moot. */
        int failures = 0;
        if (a.rms > 0.010f) {
            printf("FAIL: centred rms %.4f > 0.010\n", (double)a.rms);
            failures += 1;
        }
        if (b.rms > 0.015f) {
            printf("FAIL: offset rms %.4f > 0.015\n", (double)b.rms);
            failures += 1;
        }
        /* 5% 丢通道下，瞬时全白是预期现象，所以这里放宽到 5%。 */
        if (c.offline_frac > 0.05f) {
            printf("FAIL: offline %.1f%% > 5%%\n", (double)(c.offline_frac * 100.0f));
            failures += 1;
        }
        if (failures > 0) {
            printf("erth_sim: FAILED (%d)\n", failures);
            return 1;
        }
        printf("erth_sim: ok\n");
    }

    return 0;
}
