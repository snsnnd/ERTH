#ifndef POSITION_H
#define POSITION_H

#include "stdbool.h"
#include "kinematics.h"
#include "sensor.h"
#include "utils.h"
#include "filter_algorithm.h"
#include "time.h"

// 定位数据结构体
typedef struct
{
    float x;     // 全局坐标系X坐标 (m)
    float y;     // 全局坐标系Y坐标 (m)
    float theta; // 全局坐标系下的角度 (rad)，0为正右方向
    float vx;    // X方向速度 (m/s)
    float vy;    // Y方向速度 (m/s)
    float omega; // 角速度 (rad/s)
} Pose;

// 校准点结构体
typedef struct
{
    uint8_t node_id;
    float x; // 校准点X坐标 (m)
    float y; // 校准点Y坐标 (m)
} CalibrationPoint;

// 定位模块结构体
typedef struct
{
    // kinematics：
    RobotVelocity robot_vel; // 机器人速度

    // Sensor：
    ENCODER_SensorTypeDef wheel_speeds; // 轮速
    MPU_SensorTypeDef mpu_data;         // MPU6050数据
    Track_SensorTypeDef track_data;     // 循迹传感器数据

    // filter_algorithm：
    Attitude yaw_attitude; // 偏航角互补滤波姿态结构体（新增）
    float gyro_offset[3];  // 存储陀螺仪X/Y/Z轴零偏（rad/s）

    LowPassFilter vx_filter;    // X方向速度滤波器
    LowPassFilter vy_filter;    // Y方向速度滤波器
    LowPassFilter omega_filter; // 角速度滤波器

    // position：
    Pose current_pose; // 当前位姿

    CalibrationPoint current_calib; // 当前待匹配的校准点（来自globalmap）
    CalibrationPoint last_calib;    // 上一次成功匹配的校准点
    bool has_last_calib;            // 是否存在上一次校准点（初始为false）

    uint32_t last_update_time; // 上一次更新时间 (ms)
    uint32_t last_calib_time;  // 上次校准时间 (ms)，用于防跳变

    uint8_t is_calibrated;  // 是否已完成校准（0: 未校准, 1: 已校准）
    uint8_t calib_detected; // 是否检测到校准点（0: 未检测, 1: 已检测）

} LocalizationModule;

// 初始化定位模块
void localization_init(LocalizationModule *loc,
                       uint8_t init_id, float init_x, float init_y, float init_theta,
                       float track_interval, float lp_alpha);

// 更新定位数据
void localization_update(LocalizationModule *loc);

// 获取当前位姿
void localization_get_pose(LocalizationModule *loc, Pose *pose);

#endif // POSITION_H
