#ifndef BT_H
#define BT_H

#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <inttypes.h> // 定义 PRIu16 宏

// 调试日志控制
#define DEBUG_PRINT 1

// PID数据 (格式: PID:电机ID,实际值,目标值)
// 位置数据(POS:x,y)
// 敌人位置数据(ENEMY:x,y)
// 偏航角数据 (格式: YAW:实际值,目标值)
// 循迹数据(track_data:)

// 外部声明UART句柄（由CubeMX生成，在usart.c中定义）
extern UART_HandleTypeDef huart2;

/**
 * @brief  蓝牙模块初始化（含UART和模块参数配置）
 */
void bt_init(void);

/**
 * @brief  蓝牙打印函数（类似printf，支持格式化输出）
 * @param  format：格式化字符串
 * @param  ...：可变参数
 */
void bt_printf(const char *format, ...);

#define log_print(...) bt_printf(__VA_ARGS__)

/**
 * @brief  蓝牙模块测试函数
 */
void bt_test(void);

#endif
