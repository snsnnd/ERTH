#ifndef __TIME_H
#define __TIME_H

#include "stm32f1xx_hal.h" // 实际 MCU：STM32F103C8T6

/**
 * @brief  获取系统启动到当前的毫秒数
 * @retval uint32_t: 毫秒数（范围0~4294967295，约49.7天溢出）
 */
uint32_t timer_get_ms(void);

/**
 * @brief  计算两个时间点的毫秒差值（处理溢出）
 * @param  start: 起始时间（GetTickMs()返回值）
 * @param  end: 结束时间（GetTickMs()返回值）
 * @retval uint32_t: 时间差值（end - start，自动处理溢出）
 */
uint32_t GetTickDiffMs(uint32_t start, uint32_t end);

#endif
