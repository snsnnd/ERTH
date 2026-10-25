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

// 修复说明：原文件此处存在一组空宏（宏体为空的 yellow_Pin/red_Pin/green_Pin/blue_Pin
// 与 led_yellow_Port/led_red_Port/led_blue_Port/led_green_Port），使
// HAL_GPIO_WritePin(led_yellow_Port, yellow_Pin, ...) 展开成非法参数列表，调用无效且无法编译。
// 经全工程 grep 确认 led_yellow/led_red/led_blue/led_green 零调用者（仅 led.c 定义、led.h 声明），
// 且仓库内没有可信的引脚/端口配置（CubeMX 生成代码不在仓库中，无法恢复“真实实现”），
// 故按“删除宏及其调用点”处理：移除空宏与四个无调用者的 LED 函数，文件保留。
