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
// 实际硬件：蓝牙 SPP 接原理图 H7 串口（PB10/PB11）= USART3
extern UART_HandleTypeDef huart3;

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

// ---------------- 对战协议（SPP 蓝牙行协议，115200 8N1，ASCII 行以 '\n' 结尾） ----------------

// 单行最大长度（含结尾 '\0'），与协议单行 <40 字节留足余量
#define BT_LINE_MAX 128

/**
 * @brief  取出一整行接收数据（自动去掉结尾 '\r'/'\n'，并 NUL 结尾）
 * @note   主循环上下文调用（非中断）；无数据时返回 0
 * @param  out: 输出缓冲区
 * @param  max: out 可用字节数（含 '\0'）
 * @retval 本行有效长度（不含 '\0'）
 */
int bt_read_line(char *out, int max);

/**
 * @brief  发送一行文本（若未以 '\n' 结尾则自动补 '\n'）
 * @note   阻塞发送（约 3.5ms/40字节@115200），禁止在中断/临界区内调用
 */
void bt_send_line(const char *s);

/**
 * @brief  蓝牙模块测试函数
 */
void bt_test(void);

// 说明：全工程唯一的 HAL_UART_RxCpltCallback 强定义位于 BT.c，
//       唯一下行通道 USART3 → 蓝牙行缓冲（原 USART5→n10 雷达分支已删）

#endif
