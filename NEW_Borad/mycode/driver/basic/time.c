#include "time.h"

/**
 * @brief  获取当前毫秒数（直接调用HAL库函数）
 */
uint32_t timer_get_ms(void)
{
	return HAL_GetTick(); // 调用HAL库自带函数，返回系统启动后的毫秒数
}

/**
 * @brief  计算时间差值（处理溢出）
 * 例：若start=4294967290，end=5，实际差值为11（而非负数）
 */
uint32_t GetTickDiffMs(uint32_t start, uint32_t end)
{
	if (end >= start)
	{
		return end - start; // 无溢出时直接减
	}
	else
	{
		// 溢出时：(最大值 - start) + end + 1（补偿溢出进位）
		return (0xFFFFFFFF - start) + end + 1;
	}
}
