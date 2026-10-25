#include "BT.h"
#include <string.h>
#include <stdio.h>

// 模块内部宏定义
#define BT_USART_HANDLE &huart2 // 指向CubeMX生成的UART句柄
#define BT_BAUDRATE 115200
#define PRINT_BUF_SIZE 256 // 打印缓冲区大小

// 模块内部函数声明
static void bt_module_config(void); // 蓝牙模块AT指令配置

/**
 * @brief  蓝牙模块初始化
 */
void bt_init(void)
{
    // UART初始化由CubeMX生成的MX_USART2_UART_Init()完成
    // 此处只需进行模块参数配置
    bt_module_config();
    log_print("bt init finished\r\n");
}

/**
 * @brief  蓝牙打印函数实现
 */
void bt_printf(const char *format, ...)
{
    va_list args;
    char buf[PRINT_BUF_SIZE];
    uint16_t len;

    va_start(args, format);
    // 格式化字符串（支持中文）
    len = vsnprintf(buf, PRINT_BUF_SIZE, format, args);
    va_end(args);

    // 发送数据（使用HAL库UART发送函数）
    if (len > 0 && len < PRINT_BUF_SIZE)
    {
        HAL_UART_Transmit(BT_USART_HANDLE, (uint8_t *)buf, len, 0xFFFF);
    }
}

/**
 * @brief  蓝牙模块AT指令配置（内部使用）
 */
static void bt_module_config(void)
{
    HAL_Delay(1000); // 等待模块上电稳定（HAL库延时函数）

    // 配置蓝牙名称：STM32F407
    bt_printf("AT+NAMESTM32F407\r\n");
    HAL_Delay(500);

    // 配置配对密码：0000
    bt_printf("AT+PIN0000\r\n");
    HAL_Delay(500);

    // 配置波特率：115200（HC-05模块指令）
    bt_printf("AT+BAUD=115200\r\n");
    HAL_Delay(500);

    // 配置角色：从机模式（0-从机，1-主机）
    bt_printf("AT+ROLE=0\r\n");
    HAL_Delay(500);

    // 重启模块使配置生效
    bt_printf("AT+RESET\r\n");
    HAL_Delay(1000);
}

/**
 * @brief  蓝牙模块测试函数
 */
void bt_test(void)
{
    log_print("Bluetooth test: Hello from STM32F407\r\n");
    log_print("Current baud rate: %d\r\n", BT_BAUDRATE);
}
