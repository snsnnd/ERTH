#include "hd.h"

// 实际硬件（校赛原理图「循迹模块 iic+io」8 针座）：8 路并行数字输出。
// 座子 1..8 脚依次接 PA11 / PA10 / PA9 / PA8 / PB15 / PB14 / PB13 / PB12。
// 位映射：bit(n-1) ← 座子第 n 脚（默认 bit0 = 第1脚 = PA11 = 通道0 最左侧）。
// 原 PC10(CLK)/PC11(DAT) 软串行时序已删除：座子上没有这两根脚。

/* ==================== 极性 / 顺序两个开关（上车一次性确认） ====================
 * 极性：参考车 mishiduobao_f103 的 app_config.h 里
 *       LINE_ACTIVE_STATE = GPIO_PIN_RESET，即“压到黑线时模块输出低电平”。
 *       本函数读到高电平记 1（=白底），所以必须取反才是“1=压黑线”。
 *       上层判据（0x18 直线 / 0xF8 偏左 / 0x1F 偏右 / 0xFF 全黑 / 0x00 全白）
 *       全部按“1=黑线”编写，因此这里默认取反。
 * 顺序：若实车原地转向与预期相反（该左转却右转），把 HD_REVERSE_BITS 改成 1。
 */
#define HD_BLACK_IS_LOW 1
#define HD_REVERSE_BITS 0

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

static uint8_t reverse_bits8(uint8_t v)
{
    uint8_t r = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (v & (1u << i))
        {
            r |= (uint8_t)(1u << (7 - i));
        }
    }
    return r;
}

/**
 * @brief  并行读取循迹模块 8 路数字量
 * @note   8 脚须在 CubeMX 中配置为 GPIO_Input（见交付清单）
 * @retval uint8_t：bit0~bit7 对应座子 1~8 脚，**1 = 该路压到黑线**，0 = 白底
 */
uint8_t HD_Parallel_Read(void)
{
    uint8_t raw = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (HAL_GPIO_ReadPin(hd_pin[i].port, hd_pin[i].pin) == GPIO_PIN_SET)
        {
            raw |= (uint8_t)(1u << i);
        }
    }

#if HD_BLACK_IS_LOW
    raw = (uint8_t)(~raw & 0xFFu); // 取反：1 = 压黑线
#endif
#if HD_REVERSE_BITS
    raw = reverse_bits8(raw); // 通道左右互换
#endif

    return raw;
}
