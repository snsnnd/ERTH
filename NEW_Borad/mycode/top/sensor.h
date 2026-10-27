#ifndef SENSOR_H
#define SENSOR_H

#include "stdint.h"
#include <stdbool.h>
#include "hd.h"
#include "encoder.h"
// Lidra_driver.h 已删（雷达模块舍弃）
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
    // 8 路数字量：bit0 = 座子第 1 脚（最左侧），bit7 = 第 8 脚（最右侧）
    // 极性约定：1 = 该路压黑线，0 = 白底。
    // 依据本工程既有场景表：0x18 = 中间两路黑 → 直线；0xF8 = 左侧 5 路黑；
    // 0x1F = 右侧 5 路黑；0xFF = 全黑 → 校准点。（原注释“0=黑,1=白”与代码相反，已更正）
    uint16_t sensor;
} Track_SensorTypeDef;

// 循迹极性 / 场景常量（唯一事实来源）
// 实车若发现黑白相反，只改 TRACK_BIT_IS_BLACK 并把两个场景常量互换即可。
#define TRACK_BIT_IS_BLACK 1
#define TRACK_ALL_BLACK 0xFFu // 8 路全黑：校准点 / 横向色带
#define TRACK_ALL_WHITE 0x00u // 8 路全白：丢线

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
bool sensor_mpu_raw(MPU_SensorTypeDef *mpu);
bool sensor_mpu_tra(MPU_SensorTypeDef *mpu);
bool sensor_mpu_acc_raw(MPU_SensorTypeDef *mpu);
bool sensor_mpu_gyro_raw(MPU_SensorTypeDef *mpu);
void sensor_mpu_acc_tra(MPU_SensorTypeDef *mpu);
void sensor_mpu_gyro_tra(MPU_SensorTypeDef *mpu);

/***************************  循迹传感器  ********************************/
uint8_t sensor_hd_read_digital(Track_SensorTypeDef *track);
Track_SceneTypeDef sensor_hd_judge_scene(Track_SensorTypeDef *track);

// 激光雷达接口（sensor_get_distance/angle/rotation_speed）已删除：
// 雷达模块（N10/USART5 + 点云聚类）随硬件方案舍弃

#endif
