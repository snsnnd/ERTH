#ifndef BT_H
#define BT_H

#include "stm32f1xx_hal.h" // 实际 MCU：STM32F103C8T6
#include <stdarg.h>
#include <inttypes.h> // 定义 PRIu16 宏

// 调试日志控制
// DEBUG_PRINT        ：主循环上下文的调试打印总开关
// DEBUG_PRINT_VERBOSE：逐节点/逐轮/逐周期的刷屏级打印（默认关，否则主循环会被串口拖慢十几倍）
// BT_ALLOW_ISR_PRINT ：是否允许在中断上下文里做阻塞串口发送
//                      默认 0：TIM1 中断（10/20ms）里的 log_print/bt_send_line 直接丢弃。
//                      原因：115200 下 1 字节约 87us，20ms 节拍里几百字节的打印会让
//                      中断永不返回；且 HAL_UART_Transmit 不可重入，会切断主循环正在
//                      发送的 POS/RPOS 协议行。仅在单步排查中断代码时临时置 1。
#define DEBUG_PRINT 1
#define DEBUG_PRINT_VERBOSE 0
#define BT_ALLOW_ISR_PRINT 0

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
// 控车命令：DRV:<左轮PWM>,<右轮PWM>（有符号，范围 -999..999）；STOP；ESTOP；AUTO；START。
// DRV 是直控/按住控制，约 400ms 无续发则自动停车；ESTOP 锁存，START 清除并启动导航。

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
