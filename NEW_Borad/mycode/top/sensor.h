#ifndef SENSOR_H
#define SENSOR_H

#include "stdint.h"
#include "hd.h"
#include "encoder.h"
#include "Lidra_driver.h"
#include "mpu6050.h"

/***************************  循迹  ********************************/

// 循迹场景类型定义
typedef enum
{
    TRACK_SCENE_UNKNOWN = 0, // 未知场景
    TRACK_SCENE_STRAIGHT,    // 直线
    TRACK_CHECK_POINT,       // 校准点
    TRACK_SCENE_FULL_WHITE   // 全白
} Track_SceneTypeDef;

// 循迹传感器结构体
typedef struct
{
    uint16_t sensor; // 传感器原始数据（bit0~bit7对应8路传感器，0=黑，1=白）
} Track_SensorTypeDef;

// 定义允许的最大偏差,校准点判断
#define MAX_DIFF 1

/***************************  编码器  ********************************/

typedef struct
{
    float lf_speed;      // 左前
    float lb_speed;      // 左后
    float rf_speed;      // 右前
    float rb_speed;      // 右后
} ENCODER_SensorTypeDef; // 线速度

/***************************  MPU  ********************************/
typedef struct
{
    int16_t acc_raw_x; // 加速度（x,y,z轴，单位：m/s²）
    int16_t acc_raw_y;
    int16_t acc_raw_z;
    int16_t gyro_raw_x; // 角速度（x,y,z轴，单位：rad/s）
    int16_t gyro_raw_y;
    int16_t gyro_raw_z;
} MPU_RAW; // 原始数据

typedef struct
{
    float acc_x; // 加速度
    float acc_y;
    float acc_z;
    float gyro_x; // 角速度
    float gyro_y;
    float gyro_z;
} MPU_TRA; // 物理数据

typedef struct
{
    MPU_RAW RAW;
    MPU_TRA TRA;
} MPU_SensorTypeDef;

/***************************  编码器  ********************************/
void sensor_encoder(ENCODER_SensorTypeDef *encoder);

/***************************  mpu6050  ********************************/
void sensor_mpu_raw(MPU_SensorTypeDef *mpu);
void sensor_mpu_tra(MPU_SensorTypeDef *mpu);
void sensor_mpu_acc_raw(MPU_SensorTypeDef *mpu);
void sensor_mpu_gyro_raw(MPU_SensorTypeDef *mpu);
void sensor_mpu_acc_tra(MPU_SensorTypeDef *mpu);
void sensor_mpu_gyro_tra(MPU_SensorTypeDef *mpu);

/***************************  循迹传感器  ********************************/
int get_digitaldata(uint8_t *data);
uint8_t sensor_hd_read_digital(Track_SensorTypeDef *track);
Track_SceneTypeDef sensor_hd_judge_scene(Track_SensorTypeDef *track);

// *************************  激光雷达  *********************************/
float sensor_get_distance(void);       // 获取单个点的距离（单位：m）
float sensor_get_angle(void);          // 获取单个点的角度（单位：°，顺时针，0°朝右）
float sensor_get_rotation_speed(void); // 获取雷达转速（单位：Hz，可选）

#endif
