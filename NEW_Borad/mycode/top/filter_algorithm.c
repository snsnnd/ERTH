#include "filter_algorithm.h"
#include "utils.h"
#include <math.h>
#include <stdint.h>
#include "BT.h"
#include "mpu6050.h"

// 定义π相关常量（单精度浮点数）
#define PI_F32 3.14159265358979323846f
#define PI_HALF_F32 1.57079632679489661923f
#define PI_THREE_HALF 4.71238898038468985769f // 3π/2

static Attitude *g_attitude = NULL;

// 互补滤波初始化（接口不变，实现替换）
void ComplementaryFilter_Init(Attitude *attitude, const float *gyro_offset)
{
    if (attitude == NULL || gyro_offset == NULL)
    {
#if DEBUG_PRINT
        log_print("filter_algorithm->The complementary filter initialization parameters are empty.\r\n");
#endif
        return;
    }
    // 保存Attitude实例地址到全局指针
    g_attitude = attitude;

    attitude->roll = 0.0f;
    attitude->pitch = 0.0f;
    attitude->yaw = 0.0f;
    for (int i = 0; i < 3; i++)
    {
        if (isnan(gyro_offset[i]))
        {
#if DEBUG_PRINT
            log_print("filter_algorithm->Gyroscope zero bias invalid:\n");
            log_print("index=%d, value=%.2f\r\n", i, gyro_offset[i]);
#endif
            attitude->gyro_offset[i] = 0.0f;
        }
        else
        {
            attitude->gyro_offset[i] = gyro_offset[i];
        }
    }
}

// 互补滤波更新（接口不变，实现替换）
void ComplementaryFilter_Update(Attitude *attitude, const SensorData *sensor, float dt)
{
    if (attitude == NULL || sensor == NULL || dt <= 0.0f || isnan(dt))
    {
#if DEBUG_PRINT
        log_print("filter_algorithm->The complementary filter update parameters are invalid:\n");
        log_print("attitude=%p, sensor=%p, dt=%.2f\r\n",
                  attitude, sensor, dt);
#endif
        return;
    }
    if (isnan(sensor->acc_x) || isnan(sensor->acc_y) || isnan(sensor->acc_z) ||
        isnan(sensor->gyro_x) || isnan(sensor->gyro_y) || isnan(sensor->gyro_z))
    {
#if DEBUG_PRINT
        log_print("filter_algorithm->Sensor data invalid:\n");
        log_print("acc=(%.2f, %.2f, %.2f), gyro=(%.2f, %.2f, %.2f)\r\n",
                  sensor->acc_x, sensor->acc_y, sensor->acc_z,
                  sensor->gyro_x, sensor->gyro_y, sensor->gyro_z);
#endif
        return;
    }

    // 计算加速度计角度（atan2f：arm_atan2_f32 已删——F103C8T6 无 FPU，工程不带 CMSIS-DSP）
    float roll_acc, pitch_acc;
    roll_acc = atan2f(sensor->acc_y, sensor->acc_z);
    pitch_acc = atan2f(-sensor->acc_x, sqrtf(sensor->acc_y * sensor->acc_y + sensor->acc_z * sensor->acc_z));

    // 陀螺仪积分
    attitude->roll += (sensor->gyro_x - attitude->gyro_offset[0]) * dt;
    attitude->pitch += (sensor->gyro_y - attitude->gyro_offset[1]) * dt;
    attitude->yaw += (sensor->gyro_z - attitude->gyro_offset[2]) * dt;

    // 互补滤波融合
    attitude->roll = COMPLEMENTARY_ALPHA * attitude->roll + (1.0f - COMPLEMENTARY_ALPHA) * roll_acc;
    attitude->pitch = COMPLEMENTARY_ALPHA * attitude->pitch + (1.0f - COMPLEMENTARY_ALPHA) * pitch_acc;

    // 角度范围约束（-π 到 π）
    attitude->roll = fmodf(attitude->roll + PI_F, TWO_PI_F) - PI_F;
    attitude->pitch = fmodf(attitude->pitch + PI_F, TWO_PI_F) - PI_F;
    attitude->yaw = fmodf(attitude->yaw + PI_F, TWO_PI_F) - PI_F;
#if DEBUG_PRINT
    log_print("filter_algorithm->roll:%f,pitch:%f,yaw:%f\n", attitude->roll, attitude->pitch, attitude->yaw);
#endif
}

// 卡尔曼滤波初始化
void KalmanFilter_Init(KalmanFilter *kf, float init_angle)
{
    if (kf == NULL)
    {
#if DEBUG_PRINT
        log_print("filter_algorithm->The initialization parameter of the Kalman filter is empty\r\n");
#endif
        return;
    }
    if (isnan(init_angle))
    {
#if DEBUG_PRINT
        log_print("filter_algorithm->The initial Angle of the Kalman filter is invalid:\n");
        log_print("%.2f\r\n", init_angle);
#endif
        init_angle = 0.0f;
    }
    kf->angle = init_angle;
    kf->bias = 0.0f;
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
    kf->Q_angle = KALMAN_Q_ANGLE;
    kf->Q_gyro = KALMAN_Q_GYRO;
    kf->R_angle = KALMAN_R_ANGLE;
}

// 卡尔曼滤波更新（单轴）
float KalmanFilter_Update(KalmanFilter *kf, float new_angle, float new_gyro, float dt)
{
    if (kf == NULL)
    {
#if DEBUG_PRINT
        log_print("filter_algorithm->The update parameter of the Kalman filter is empty\r\n");
#endif
        return 0.0f;
    }
    if (isnan(new_angle) || isnan(new_gyro) || dt <= 0.0f || isnan(dt))
    {
#if DEBUG_PRINT
        log_print("filter_algorithm->Kalman filter input invalid:\n");
        log_print("angle=%.2f, gyro=%.2f, dt=%.2f\r\n",
                  new_angle, new_gyro, dt);
#endif
        return kf->angle;
    }

    // 预测步骤
    kf->angle += dt * (new_gyro - kf->bias);

    // 协方差预测（标准 angle-bias 卡尔曼形式；原 dt2 = dt*dt 为死变量已删）
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] = kf->P[0][1];
    kf->P[1][1] += kf->Q_gyro * dt;
#if DEBUG_PRINT
    log_print("Covariance prediction is finished\n");
#endif
    // 更新步骤
    float diff = new_angle - kf->angle;
    float y = fmodf(diff + PI_F, TWO_PI_F) - PI_F;

    float S = kf->P[0][0] + kf->R_angle;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    kf->angle += K0 * y;
    kf->bias += K1 * y;

    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    kf->P[0][0] -= K0 * P00;
    kf->P[0][1] -= K0 * P01;
    kf->P[1][0] -= K1 * P00;
    kf->P[1][1] -= K1 * P01;

    return kf->angle;
}

float GetYawAngle()
{
    // 检查Attitude实例是否已初始化
    if (g_attitude == NULL)
    {
#if DEBUG_PRINT
        log_print("filter_algorithm->GetYawAngle: Attitude not initialized!\r\n");
#endif
        return 0.0f; // 或返回NAN表示错误（需包含math.h）
    }
    // 返回当前偏航角（已在ComplementaryFilter_Update中做过范围约束）
    return g_attitude->yaw;
}
