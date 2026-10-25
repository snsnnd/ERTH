#include "hd.h"

// 实际硬件（校赛原理图「循迹模块 iic+io」8 针座）：8 路并行数字输出。
// 座子 1..8 脚依次接 PA11 / PA10 / PA9 / PA8 / PB15 / PB14 / PB13 / PB12。
// 位映射：bit(n-1) ← 座子第 n 脚（bit0 = 第1脚 = PA11 = 通道0 最左侧）。
// 若实车左右通道相反，只需颠倒下表顺序即可，上层协议不变。
// 原 PC10(CLK)/PC11(DAT) 软串行时序已删除：座子上没有这两根脚。
static const struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} hd_pin[8] = {
    {GPIOA, GPIO_PIN_11}, // bit0 / 通道0（最左）
    {GPIOA, GPIO_PIN_10}, // bit1 / 通道1
    {GPIOA, GPIO_PIN_9},  // bit2 / 通道2
    {GPIOA, GPIO_PIN_8},  // bit3 / 通道3
    {GPIOB, GPIO_PIN_15}, // bit4 / 通道4
    {GPIOB, GPIO_PIN_14}, // bit5 / 通道5
    {GPIOB, GPIO_PIN_13}, // bit6 / 通道6
    {GPIOB, GPIO_PIN_12}, // bit7 / 通道7（最右）
};

/**
 * @brief  并行读取循迹模块 8 路数字量（一次读 8 个 GPIO 电平）
 * @note   8 脚须在 CubeMX 中配置为 GPIO_Input（见交付清单）
 * @retval uint8_t：8位数据（bit0~bit7 对应座子 1~8 脚）
 */
uint8_t HD_Parallel_Read(void)
{
    uint8_t data = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (HAL_GPIO_ReadPin(hd_pin[i].port, hd_pin[i].pin) == GPIO_PIN_SET)
        {
            data |= (uint8_t)(1u << i);
        }
    }

    return data;
}
