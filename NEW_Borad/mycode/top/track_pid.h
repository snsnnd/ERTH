#ifndef TRACK_PID_H
#define TRACK_PID_H

#include "stm32f1xx_hal.h" // 实际 MCU：STM32F103C8T6
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

/* ==================== 循迹差速参数 ====================
 * 全部对齐参考车 D:\efw_studio\examples\zhixianxunji\mishiduobao_f103
 * （同款硬件、能正常跑直线的车）：
 *   app.c:  police/thief base_cps = 2000
 *           line_diff_per_level = 100 CPS/级, line_diff_max = 1000 CPS
 *           target_cps_min = 180, target_cps_max = 2300
 *           left_speed_base_pwm = 292, right_speed_base_pwm = 260
 *   line_controller.c: 差速目标后按 PWM = 目标CPS × 基础PWM ÷ 2000 映射
 * 含义：直行时左右占空比 292/999≈29%、260/999≈26%（右轮略弱，补偿机械差异）；
 *       满偏时左 335/2300≈33.5%、右 130/2000≈13%，即最大占空比差约 ±15%。
 * 原工程是"PD 输出 ×3"→ 直行 45%、满偏差 43%，比参考车猛约 3 倍，是跑不直的主因之一。
 */
#define LINE_BASE_CPS 2000        // 直行目标速度（CPS，仅作为差速基准）
#define LINE_DIFF_CPS_PER_LEVEL 100 // 每 1 级权重产生多少 CPS 差速
#define LINE_DIFF_CPS_MAX 1000    // 单侧最大差速修正（CPS）
#define LINE_TARGET_CPS_MIN 180   // 单侧目标下限
#define LINE_TARGET_CPS_MAX 2300  // 单侧目标上限
#define LINE_LEFT_BASE_PWM 292    // 左轮直行基础 PWM（满量程 999）
#define LINE_RIGHT_BASE_PWM 260   // 右轮直行基础 PWM（右轮略弱，补偿机械差异）

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
 * @brief  计算循迹偏差（基于8路传感器，1=压黑线）
 * @param  sensor_data: 8路传感器数据（bit=1 表示该路压黑线）
 * @retval float: 偏差值（负值线偏左，正值线偏右），无黑线时返回 0
 */
float Track_CalcError(uint16_t sensor_data);

/**
 * @brief  闭环 PD 计算（当前循迹用的是参考车的开环差速，本函数保留供后续接速度环）
 * @param  target: 目标值（通常为0，即中线）
 * @param  current: 当前偏差（从Track_CalcError获取）
 * @retval float: PD 输出值
 */
float TrackPI_Compute(float target, float current);

/**
 * @brief  偏差 → 左右 PWM（参考车的差速映射）
 * @param  error: Track_CalcError 的返回值
 * @param  left_pwm: 左轮 PWM 输出（0~PWM_MAX）
 * @param  right_pwm: 右轮 PWM 输出（0~PWM_MAX）
 */
void Track_ControlMotor(float error, int *left_pwm, int *right_pwm);

/**
 * @brief  循迹控制主函数（读取 → 偏差 → 差速 → 左右 PWM）
 * @param  left_pwm: 左轮 PWM（输出）
 * @param  right_pwm: 右轮 PWM（输出）
 * @retval uint8_t: 8 路循迹原始值（1=压黑线）
 */
uint8_t Track_MainControl(int *left_pwm, int *right_pwm);

#endif // TRACK_PID_H
