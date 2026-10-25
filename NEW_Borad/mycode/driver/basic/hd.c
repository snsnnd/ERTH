#include "hd.h"
#include "delay.h"
#define HD_CLK_PIN GPIO_PIN_10
#define HD_CLK_PORT GPIOC
#define HD_DAT_PIN GPIO_PIN_11
#define HD_DAT_PORT GPIOC

/**
 * @brief  按串行时序读取8位数据（高电平锁存，低电平读取）
 * @note   时序要求：CLK高电平≥5μs，时钟间隔＜1ms
 * @retval uint8_t：8位数据（1=高电平输入，0=低电平输入）
 */
uint8_t HD_Serial_Read(void)
{
    uint8_t data = 0;

    for (uint8_t i = 0; i < 8; i++) // 循环读取8位（第1位→第8位）
    {
        // 1. CLK拉高：锁存当前位数据到DAT
        HAL_GPIO_WritePin(HD_CLK_PORT, HD_CLK_PIN, GPIO_PIN_SET);
        delay_us(5);
        // 若HAL_Delay精度不足，可用空循环：for(int d=0;d<100;d++);

        // 2. 读取DAT数据（左移一位，存入新数据）
        data <<= 1;
        if (HAL_GPIO_ReadPin(HD_DAT_PORT, HD_DAT_PIN) == GPIO_PIN_SET)
        {
            data |= 0x01; // 当前位为高电平
        }

        // 3. CLK拉低：准备下一位读取
        HAL_GPIO_WritePin(HD_CLK_PORT, HD_CLK_PIN, GPIO_PIN_RESET);
        delay_us(5);
    }

    return data;
}
