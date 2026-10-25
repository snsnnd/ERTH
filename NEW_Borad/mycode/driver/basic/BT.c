#include "BT.h"
#include "Lidra_driver.h" // 供 HAL_UART_RxCpltCallback 分发 huart5 → n10_rx_byte()
#include <string.h>
#include <stdio.h>

// 模块内部宏定义
#define BT_USART_HANDLE &huart2 // 指向CubeMX生成的UART句柄
#define BT_BAUDRATE 115200
#define PRINT_BUF_SIZE 256 // 打印缓冲区大小

// ---------------- 对战协议下行接收（UART2 单字节中断 → 完整行队列） ----------------
#define BT_RX_LINE_QUEUE 5 // 完整行队列槽数（单生产者单消费者，有效容量 = 槽数-1 = 4）

// 模块内部函数声明
static void bt_module_config(void); // 蓝牙模块AT指令配置

// 接收状态（全部由 UART2 中断上下文写入装配缓冲，主循环只读成品行队列）
static uint8_t bt_rx_byte;                         // HAL_UART_Receive_IT 单字节缓冲
static volatile char bt_asm_buf[BT_LINE_MAX];      // 行装配缓冲（中断上下文）
static volatile uint16_t bt_asm_len;               // 当前装配长度
static volatile uint8_t bt_asm_discard;            // 溢出丢弃模式：丢掉本行余下字节直到 '\n'（重新同步）
static volatile char bt_line_q[BT_RX_LINE_QUEUE][BT_LINE_MAX]; // 成品行队列（NUL 结尾）
static volatile uint8_t bt_line_head;              // 生产者（中断）写入位置
static volatile uint8_t bt_line_tail;              // 消费者（主循环）读取位置
static volatile uint16_t bt_line_dropped;          // 因队列满/行超长被丢弃的行数（调试用）

/**
 * @brief  中断上下文：把装配好的一行推入成品行队列（SPSC，无锁）
 */
static void bt_push_line(void)
{
    uint8_t next = (uint8_t)((bt_line_head + 1) % BT_RX_LINE_QUEUE);
    if (next == bt_line_tail)
    {
        bt_line_dropped++; // 队列满：丢弃新行（主循环消费过慢）
        return;
    }
    memcpy((void *)bt_line_q[bt_line_head], (const void *)bt_asm_buf, bt_asm_len);
    bt_line_q[bt_line_head][bt_asm_len] = '\0'; // bt_asm_len 最大 BT_LINE_MAX-1，必留 '\0' 空间
    __DMB();                                    // 保证行数据先于索引更新可见
    bt_line_head = next;
}

/**
 * @brief  中断上下文：UART2 单字节行装配（含溢出保护与重新同步）
 * 规则：'\n' 为行界；'\r' 忽略；超长则丢弃本行并进入丢弃模式直到下一个 '\n'
 */
static void bt_rx_byte_handler(uint8_t byte)
{
    if (bt_asm_discard)
    {
        if (byte == '\n')
        {
            bt_asm_discard = 0; // 本行丢弃完毕，重新同步到下一行
        }
        return;
    }

    if (byte == '\n')
    {
        if (bt_asm_len > 0)
        {
            bt_push_line();
        }
        bt_asm_len = 0;
        return;
    }

    if (byte == '\r')
    {
        return; // 兼容 "\r\n"，行界只认 '\n'
    }

    if (bt_asm_len >= BT_LINE_MAX - 1)
    {
        bt_asm_len = 0;
        bt_asm_discard = 1;
        bt_line_dropped++; // 溢出：丢弃本行，等待行界重新同步
        return;
    }

    bt_asm_buf[bt_asm_len++] = byte;
}

/**
 * @brief  全工程唯一的 UART 接收完成回调（强定义）
 * 按 huart->Instance 分发：
 *   USART2 → 蓝牙 SPP 行协议（对战协议 ORP/ERR 下行）
 *   USART5 → N10 雷达（转发给从 Lidra_driver.c 抽出的 n10_rx_byte）
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance == NULL)
    {
        return;
    }

    if (huart->Instance == USART2)
    {
        bt_rx_byte_handler(bt_rx_byte);
        // 重启下 1 字节接收（HAL 需手动重启；禁止在其中发送数据）
        HAL_UART_Receive_IT(BT_USART_HANDLE, &bt_rx_byte, 1);
    }
    else if (huart->Instance == USART5)
    {
        n10_rx_byte(); // Lidra_driver.c 中抽取的处理 + 接收重启
    }
}

/**
 * @brief  蓝牙模块初始化
 */
void bt_init(void)
{
    // UART初始化由CubeMX生成的MX_USART2_UART_Init()完成
    // 此处只需进行模块参数配置
    bt_module_config();

    // 启动对战协议下行接收（单字节中断，回调内重启）
    bt_asm_len = 0;
    bt_asm_discard = 0;
    bt_line_head = 0;
    bt_line_tail = 0;
    bt_line_dropped = 0;
    HAL_UART_Receive_IT(BT_USART_HANDLE, &bt_rx_byte, 1);

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
 * @brief  发送一行文本（未以 '\n' 结尾则自动补）
 * @note   阻塞发送，禁止在中断/临界区内调用
 */
void bt_send_line(const char *s)
{
    if (s == NULL)
    {
        return;
    }

    size_t len = strlen(s);
    if (len > 0)
    {
        HAL_UART_Transmit(BT_USART_HANDLE, (uint8_t *)s, (uint16_t)len, 0xFFFF);
    }
    if (len == 0 || s[len - 1] != '\n')
    {
        HAL_UART_Transmit(BT_USART_HANDLE, (uint8_t *)"\n", 1, 0xFFFF);
    }
}

/**
 * @brief  取出一整行接收数据（去 '\r'/'\n'），主循环上下文调用
 * @retval 行长度，0 表示无数据
 */
int bt_read_line(char *out, int max)
{
    if (out == NULL || max <= 0)
    {
        return 0;
    }
    if (bt_line_tail == bt_line_head)
    {
        return 0; // 队列空
    }
    __DMB();

    const char *line = (const char *)bt_line_q[bt_line_tail];
    int n = 0;
    while (n < max - 1)
    {
        char c = line[n];
        if (c == '\0' || c == '\r' || c == '\n')
        {
            break;
        }
        out[n] = c;
        n++;
    }
    out[n] = '\0';

    // 无论是否截断，整行按一个槽位消费（协议行 <40B << max，正常不会截断）
    __DMB();
    bt_line_tail = (uint8_t)((bt_line_tail + 1) % BT_RX_LINE_QUEUE);
    return n;
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
