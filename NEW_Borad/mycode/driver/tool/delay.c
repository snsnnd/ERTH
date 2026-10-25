#include "delay.h"

/**
 * @brief  微秒级延时（基于 SysTick 定时器）
 * @param  us：需要延时的微秒数
 * @note   1. 依赖系统时钟（SystemCoreClock），需确保 SystemCoreClock 已正确配置
 *         2. 若 SysTick 时钟源为 HCLK/8（默认），代码已自动适配
 *         3. 节拍计数使用 64 位：原 uint32_t 在 us 较大时
 *            ticks = us * (SystemCoreClock/1000000) 会溢出回绕，导致延时严重偏短；
 *            现以 (uint64_t)us 参与运算，消除该溢出（SysTick 24 位回绕由累计逻辑处理）
 */
void delay_us(uint32_t us)
{
    uint64_t ticks;      // 需要的总节拍数（64 位，防乘法溢出）
    uint32_t told, tnow; // 记录 SysTick 当前值
    uint64_t tcnt = 0;   // 累计节拍数（64 位，与 ticks 同宽比较）
    uint32_t reload;     // SysTick 装载值

    // 1. 获取 SysTick 配置参数
    reload = SysTick->LOAD;                          // 读取 SysTick 最大装载值
    ticks = (uint64_t)us * (SystemCoreClock / 1000000U); // 计算需要的节拍数（SystemCoreClock 单位：Hz）

    // 2. 记录当前 SysTick 计数器值
    told = SysTick->VAL;

    while (1)
    {
        tnow = SysTick->VAL;

        // 3. 计算累计节拍数（处理计数器递减和溢出）
        if (tnow != told)
        {
            // 若当前值 < 上一次值：未溢出，直接累加差值
            if (tnow < told)
                tcnt += told - tnow;
            // 若当前值 > 上一次值：已溢出，累加（装载值 - 当前值 + 上一次值）
            else
                tcnt += reload - tnow + told;

            told = tnow;

            // 4. 累计节拍数达标，退出延时
            if (tcnt >= ticks)
                break;
        }
    }
}
