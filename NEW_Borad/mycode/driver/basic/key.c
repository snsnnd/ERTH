#include "key.h"
#include "time.h" // timer_get_ms / GetTickDiffMs

/* 按键物理映射：低电平 = 按下。
 * 直接写端口/引脚，避免改 CubeMX 生成的 main.h（重新生成会被覆盖）。 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t ev; // 对应的 KEY_EV_* 位
} key_pin_t;

static const key_pin_t keys[4] = {
    {GPIOC, GPIO_PIN_14, KEY_EV_START}, // 开始/继续
    {GPIOC, GPIO_PIN_15, KEY_EV_PAUSE}, // 暂停
    {GPIOC, GPIO_PIN_13, KEY_EV_ESTOP}, // 急停
    {GPIOA, GPIO_PIN_2, KEY_EV_RESET},  // 复位
};

#define KEY_COUNT 4u
#define KEY_SCAN_PERIOD_MS 10u // 扫描节流
#define KEY_DEBOUNCE_COUNT 2u  // 连续 2 次（≈20ms）状态一致才认

static uint8_t s_pressed[KEY_COUNT]; // 1 = 已确认为按下
static uint8_t s_debounce[KEY_COUNT];
static uint32_t s_last_scan_ms;

uint8_t key_scan(void)
{
    uint32_t now = timer_get_ms();
    uint8_t events = 0;

    if (GetTickDiffMs(s_last_scan_ms, now) < KEY_SCAN_PERIOD_MS)
    {
        return 0; // 未到扫描周期
    }
    s_last_scan_ms = now;

    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        uint8_t pressed = (HAL_GPIO_ReadPin(keys[i].port, keys[i].pin) == GPIO_PIN_RESET) ? 1u : 0u;

        if (pressed == s_pressed[i])
        {
            s_debounce[i] = 0; // 状态稳定，清计数
            continue;
        }

        if (++s_debounce[i] >= KEY_DEBOUNCE_COUNT)
        {
            s_debounce[i] = 0;
            s_pressed[i] = pressed;
            if (pressed)
            {
                events |= keys[i].ev; // 只上报"按下"沿
            }
        }
    }

    return events;
}

uint8_t key_is_down(uint8_t ev)
{
    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        if ((keys[i].ev & ev) != 0u)
        {
            return (HAL_GPIO_ReadPin(keys[i].port, keys[i].pin) == GPIO_PIN_RESET) ? 1u : 0u;
        }
    }
    return 0;
}
