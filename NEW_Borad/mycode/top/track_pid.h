#ifndef TRACK_PID_H
#define TRACK_PID_H

#include "stm32f4xx_hal.h"
#include "hd.h"

// 传感器通道定义（8路，从左到右）
#define CHANNEL_0 0 // 最左侧
#define CHANNEL_1 1
#define CHANNEL_2 2
#define CHANNEL_3 3
#define CHANNEL_4 4
#define CHANNEL_5 5
#define CHANNEL_6 6
#define CHANNEL_7 7 // 最右侧

// 电机速度参数（根据硬件调整）
#define BASE_SPEED 15 // 基础速度（PWM值，0~1000为例）
#define MAX_SPEED 30  // 最大速度限制
#define MIN_SPEED 10  // 最小速度限制

typedef struct
{
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数（新增）
    float output_max;   // 输出最大值
    float integral_max; // 积分最大值
    float error;        // 当前误差
    float last_error;   // 上一周期误差（用于计算微分）
    float integral;     // 积分项
    float derivative;   // 微分项（新增）
    float output;       // 输出控制量
} TrackPI_HandleTypeDef;

extern TrackPI_HandleTypeDef *hpi;

/**
 * @brief  计算循迹偏差（基于8路传感器）
 * @param  sensor_data: 8路传感器数据
 * @retval float: 偏差值（负值偏左，正值偏右）
 */
float Track_CalcError(uint16_t sensor_data);

/**
 * @brief  执行PID计算（增加微分环节）
 * @param  target: 目标值（通常为0，即中线）
 * @param  current: 当前偏差（从Track_CalcError获取）
 * @retval float: PID输出值
 */
float TrackPI_Compute(float target, float current);

/**
 * @brief  根据PID输出控制左右电机速度
 * @param  pi_output: PID输出值
 * @param  left_speed: 左电机速度（输出参数）
 * @param  right_speed: 右电机速度（输出参数）
 */
void Track_ControlMotor(float pi_output, int *left_speed, int *right_speed);

/**
 * @brief  循迹控制主函数（整合读取-偏差-PID-电机）
 * @param  left_speed: 左电机速度（输出）
 * @param  right_speed: 右电机速度（输出）
 * @retval uint8_t: 传感器原始数据
 */
uint8_t Track_MainControl(int *left_speed, int *right_speed);

#endif // TRACK_PID_H
