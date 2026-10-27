#ifndef KEY_H
#define KEY_H

#include "main.h"
#include <stdint.h>

/* ==================== 按键布局（低电平按下，内部上拉） ====================
 * 与参考车 mishiduobao_f103 的四键位置一致（同款硬件）：
 *   PC14 = 开始/继续     （参考车 KEY_START）
 *   PC15 = 暂停          （参考车 KEY_PAUSE）
 *   PC13 = 急停（锁存）  （参考车 KEY_ROLE，这里改作急停更好用）
 *   PA2  = 复位          （参考车 KEY_RESET，软复位 MCU）
 * 四根脚在 gpio.c 里都已配成 GPIO_MODE_INPUT + PULLUP。
 * 若板上只有 PC13 一个键，见 FSM.c 的 START_REQUIRE_KEY 开关。
 */

#define KEY_EV_START 0x01u // 开始/继续
#define KEY_EV_PAUSE 0x02u // 暂停
#define KEY_EV_ESTOP 0x04u // 急停（锁存，必须再按"开始"才解除）
#define KEY_EV_RESET 0x08u // 软件复位

/**
 * @brief  非阻塞按键扫描（10ms 节流 + 连续 2 次确认去抖）
 * @retval 本次扫描中新产生的"按下"事件位（KEY_EV_* 的组合），无事件返回 0
 * @note   在主循环里反复调用即可，不会阻塞；10ms 内重复调用只扫描一次
 */
uint8_t key_scan(void);

/**
 * @brief  查询某个按键当前是否仍处于按下状态
 * @param  ev 单个 KEY_EV_* 位
 * @retval 1=按下，0=松开
 */
uint8_t key_is_down(uint8_t ev);

#endif // KEY_H
