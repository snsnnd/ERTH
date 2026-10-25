#include "led.h"

// #define yellow_Pin GPIO_PIN_5
// #define yellow_GPIO_Port GPIOE

// #define red_Pin GPIO_PIN_6
// #define red_GPIO_Port GPIOE

// #define green_Pin GPIO_PIN_9
// #define green_GPIO_Port GPIOB

// #define blue_Pin GPIO_PIN_2
// #define blue_GPIO_Port GPIOG

// // 初始化LED
// void led_init(void)
// {

//     // 初始化时关闭所有LED
//     led_off();
// }

// // 打开指定的LED
// void led_on(void)
// {
//     // switch (led)
//     // {
//     // case blue:
//     //     HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_SET);
//     //     break;
//     // case yellow:
//     //     HAL_GPIO_WritePin(yellow_GPIO_Port, yellow_Pin, GPIO_PIN_SET);
//     //     break;
//     // case green:
//     //     HAL_GPIO_WritePin(green_GPIO_Port, green_Pin, GPIO_PIN_SET);
//     //     break;
//     // case red:
//     //     HAL_GPIO_WritePin(red_GPIO_Port, red_Pin, GPIO_PIN_SET);
//     //     break;
//     // default:
//     //     // 处理无效的LED
//     //     break;
//     // }
//     HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
// }

// // 关闭指定的LED
// void led_off(void)
// {
//     // switch (led)
//     // {
//     // case blue:
//     //     HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_RESET);
//     //     break;
//     // case yellow:
//     //     HAL_GPIO_WritePin(yellow_GPIO_Port, yellow_Pin, GPIO_PIN_RESET);
//     //     break;
//     // case green:
//     //     HAL_GPIO_WritePin(green_GPIO_Port, green_Pin, GPIO_PIN_RESET);
//     //     break;
//     // case red:
//     //     HAL_GPIO_WritePin(red_GPIO_Port, red_Pin, GPIO_PIN_RESET);
//     //     break;
//     // default:
//     //     // 处理无效的LED
//     //     break;
//     // }
//     HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
// }

// void test_led()
// {
//     led_on();
//     HAL_Delay(1000);
//     led_off();
// }

#define green_Pin
#define red_Pin
#define blue_Pin
#define yellow_Pin

#define led_green_Port
#define led_red_Port
#define led_blue_Port
#define led_yellow_Port

void led_yellow(uint8_t a)
{
    if (a == 1)
    {
        HAL_GPIO_WritePin(led_yellow_Port, yellow_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(led_yellow_Port, yellow_Pin, GPIO_PIN_SET);
    }
}
void led_red(uint8_t a)
{
    if (a == 1)
    {
        HAL_GPIO_WritePin(led_red_Port, red_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(led_red_Port, red_Pin, GPIO_PIN_SET);
    }
}
void led_blue(uint8_t a)
{
    if (a == 1)
    {
        HAL_GPIO_WritePin(led_blue_Port, blue_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(led_blue_Port, blue_Pin, GPIO_PIN_SET);
    }
}
void led_green(uint8_t a)
{
    if (a == 1)
    {
        HAL_GPIO_WritePin(led_green_Port, green_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(led_green_Port, green_Pin, GPIO_PIN_SET);
    }
}
