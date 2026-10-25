#include "key.h"
#include "delay.h" // delay_ms 宏（= HAL_Delay）

// #define key_Pin
// #define key_GPIO_Port

/**
 * @brief  按键读取（低电平有效：按键按下时引脚读到 RESET/低电平）
 * @param  a：保留参数，当前逻辑未使用（维持接口兼容）
 * @param  b：按键模式状态指针，每次有效按键在 0 与 3 之间翻转
 * @retval 1 = 检测到一次有效按键（已翻转 *b），0 = 无按键动作
 * @note   时序：检测按下（低电平）→ 延时去抖 → 复检仍为低电平 → 翻转 *b → 等待释放。
 *         修正：原“等待释放”条件写反为 while (ReadPin)——按下态（低电平）时该循环
 *         立即退出，一次长按会被主循环反复触发；现改为 while (!ReadPin)，
 *         即按下期间（低电平）自锁，松开（高电平）后才返回。
 */
uint8_t key_read(uint8_t *a, uint8_t *b)
{
    (void)a; // 保留参数：当前逻辑未使用

    if (!HAL_GPIO_ReadPin(key_GPIO_Port, key_Pin)) // 有效电平判断：读到低电平 = 按下
    {
        delay_ms(10); // 去抖延时
        if (!HAL_GPIO_ReadPin(key_GPIO_Port, key_Pin)) // 复检：仍为低电平才认定有效按下
        {
            if (*b == 3)
            {
                *b = 0;
            }
            else
            {
                *b = 3;
            }

            // 等待释放：按下期间（低电平）一直自锁，松开（高电平）后返回
            while (!HAL_GPIO_ReadPin(key_GPIO_Port, key_Pin))
                ;
            return 1;
        }
    }

    return 0;
}
