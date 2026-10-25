#ifndef FILTER_ALGORITHM_H
#define FILTER_ALGORITHM_H

// 滤波器参数
#define COMPLEMENTARY_ALPHA 0.95f
#define KALMAN_Q_ANGLE 0.001f
#define KALMAN_Q_GYRO 0.03f
#define KALMAN_R_ANGLE 0.5f

// 传感器数据结构体
typedef struct
{
    float acc_x;  // 加速度计X轴 (m/s²)
    float acc_y;  // 加速度计Y轴
    float acc_z;  // 加速度计Z轴
    float gyro_x; // 陀螺仪X轴 (rad/s)
    float gyro_y; // 陀螺仪Y轴
    float gyro_z; // 陀螺仪Z轴
} SensorData;

// 姿态角结构体
typedef struct
{
    float roll;           // 横滚角 (rad)
    float pitch;          // 俯仰角 (rad)
    float yaw;            // 偏航角 (rad)
    float gyro_offset[3]; // 陀螺仪零偏 (rad/s)
} Attitude;

// 卡尔曼滤波器结构体（1D）
typedef struct
{
    float angle;   // 滤波后角度 (rad)
    float bias;    // 角速度偏差 (rad/s)
    float P[2][2]; // 协方差矩阵
    float Q_angle; // 角度过程噪声
    float Q_gyro;  // 角速度过程噪声
    float R_angle; // 测量噪声
} KalmanFilter;

// 函数声明
void ComplementaryFilter_Init(Attitude *attitude, const float *gyro_offset);
void ComplementaryFilter_Update(Attitude *attitude, const SensorData *sensor, float dt);

void KalmanFilter_Init(KalmanFilter *kf, float init_angle);
float KalmanFilter_Update(KalmanFilter *kf, float new_angle, float new_gyro, float dt);

float GetYawAngle(void);

#endif // FILTER_ALGORITHM_H
