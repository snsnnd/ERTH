#include "BT.h"
#include <string.h>
#include <stdio.h>

// 模块内部宏定义
// 实际硬件（校赛原理图 H7 串口）：蓝牙 SPP 模块接 PB10/PB11 = USART3，
// 故句柄由原 huart2 改为 huart3（CubeMX 需同步开启 USART3，见交付清单）
#define BT_USART_HANDLE &huart3 // 指向CubeMX生成的UART句柄
#define BT_BAUDRATE 115200
#define PRINT_BUF_SIZE 256 // 打印缓冲区大小

// 模组 AT 配置开关
// 默认 0（关闭）：HC-05 类模组必须在“AT 模式”（KEY 拉高、38400 波特）下配置，
// 在数据透传通道（本工程 115200）上发 AT 指令无效，而且这 5 行会直接灌到
// 蓝牙对端（上位机 final.py），既白等约 4s 启动时间，又给协议行添乱。
// 若确实需要上电自动配置，把这里置 1 并按模组手册改 bt_module_config() 的指令/波特率。
#define BT_AT_CONFIG 0

// UART 发送互斥标志：HAL_UART_Transmit 不可重入，主循环与中断若同时发送，
// 字节流会交织、把一条协议行切断。此处用“忙则丢弃”的简单策略保证行完整性。
static volatile uint8_t bt_tx_busy;

// ---------------- 对战协议下行接收（UART3 单字节中断 → 完整行队列） ----------------
#define BT_RX_LINE_QUEUE 5 // 完整行队列槽数（单生产者单消费者，有效容量 = 槽数-1 = 4）

// 模块内部函数声明
static void bt_module_config(void); // 蓝牙模块AT指令配置

// 接收状态（全部由 UART3 中断上下文写入装配缓冲，主循环只读成品行队列）
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
 * @brief  中断上下文：UART3 单字节行装配（含溢出保护与重新同步）
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
 * 唯一下行通道：USART3（蓝牙 SPP 行协议，对战协议 ORP/ERR 下行）
 * （原按 Instance 分发的 USART5 → N10 雷达分支已随雷达模块一并删除）
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance == NULL)
    {
        return;
    }

    if (huart->Instance == USART3)
    {
        bt_rx_byte_handler(bt_rx_byte);
        // 重启下 1 字节接收（HAL 需手动重启；禁止在其中发送数据）
        HAL_UART_Receive_IT(BT_USART_HANDLE, &bt_rx_byte, 1);
    }
}

/**
 * @brief  蓝牙模块初始化
 */
void bt_init(void)
{
    // UART初始化由CubeMX生成的MX_USART3_UART_Init()完成
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
 * @brief  尝试独占 UART 发送权（忙则返回 0，调用方丢弃本条消息）
 * @note   只锁“判忙+置忙”的几条指令，真正的发送过程不关中断
 */
static uint8_t bt_tx_try_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t locked = 0;

    __disable_irq();
    if (!bt_tx_busy)
    {
        bt_tx_busy = 1;
        locked = 1;
    }
    if (!primask)
    {
        __enable_irq();
    }
    return locked;
}

/**
 * @brief  整段发送（不可重入保护 + 忙则丢弃），保证协议行不被字节交织切断
 */
static void bt_tx_write(const char *p, uint16_t len)
{
    if (p == NULL || len == 0 || !bt_tx_try_lock())
    {
        return;
    }
    HAL_UART_Transmit(BT_USART_HANDLE, (uint8_t *)p, len, 0xFFFF);
    bt_tx_busy = 0; // 释放：单个字节赋值，中断侧读到 0 即可重试
}

/**
 * @brief  蓝牙打印函数实现
 */
void bt_printf(const char *format, ...)
{
    va_list args;
    char buf[PRINT_BUF_SIZE];
    int len;

    // 中断上下文禁止阻塞发送（会拖垮 10/20ms 节拍并与主循环抢 UART）
    if (!BT_ALLOW_ISR_PRINT && __get_IPSR() != 0U)
    {
        return;
    }
    if (format == NULL)
    {
        return;
    }

    va_start(args, format);
    len = vsnprintf(buf, PRINT_BUF_SIZE, format, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }
    if (len >= PRINT_BUF_SIZE)
    {
        len = PRINT_BUF_SIZE - 1; // 超长截断发送（原实现整条静默丢弃）
    }
    bt_tx_write(buf, (uint16_t)len);
}

/**
 * @brief  发送一行文本（未以 '\n' 结尾则自动补）
 * @note   阻塞发送，禁止在中断/临界区内调用（中断内默认被 BT_ALLOW_ISR_PRINT 拦掉）
 *         文本与结尾 '\n' 合并成一次发送，避免两条消息交织
 */
void bt_send_line(const char *s)
{
    char buf[BT_LINE_MAX + 2];
    size_t len;

    if (s == NULL)
    {
        return;
    }
    if (!BT_ALLOW_ISR_PRINT && __get_IPSR() != 0U)
    {
        return;
    }

    len = strlen(s);
    if (len > BT_LINE_MAX)
    {
        len = BT_LINE_MAX;
    }
    memcpy(buf, s, len);
    if (len == 0 || buf[len - 1] != '\n')
    {
        buf[len++] = '\n';
    }
    bt_tx_write(buf, (uint16_t)len);
}

/**
 * @brief  取出一整行接收数据（去 '\r'/'\n'），主循环上下文调用
 * @retval 行长度，0 表示无数据
 * @note   共享索引 bt_line_head/bt_line_tail 已声明 volatile；
 *         主循环侧读取/更新用临界区快照（中断侧写入单字节索引本身原子，无需再进临界区）
 */
int bt_read_line(char *out, int max)
{
    uint8_t tail, head;

    if (out == NULL || max <= 0)
    {
        return 0;
    }

    // 临界区快照共享索引：避免读取 head/tail 时被中断的写入打断（只包最小片段）
    // 用 PRIMASK 保存/恢复，而不是无条件 __enable_irq()（否则在临界区内调用会提前开中断）
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    tail = bt_line_tail;
    head = bt_line_head;
    if (!primask)
    {
        __enable_irq();
    }

    if (tail == head)
    {
        return 0; // 队列空
    }
    __DMB();

    // 读取快照槽位：中断只会写 head 槽（head != tail），不会动 tail 槽，故拷贝期间无需关中断
    const char *line = (const char *)bt_line_q[tail];
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
    // tail 的读-改-写在临界区内完成，防止中断在此期间读到中间状态
    primask = __get_PRIMASK();
    __disable_irq();
    bt_line_tail = (uint8_t)((bt_line_tail + 1) % BT_RX_LINE_QUEUE);
    if (!primask)
    {
        __enable_irq();
    }
    return n;
}

/**
 * @brief  蓝牙模块AT指令配置（内部使用，默认编译关闭，见文件头 BT_AT_CONFIG）
 * @note   HC-05/HC-06 类模组：KEY/EN 拉高进入 AT 模式后波特率固定 38400，
 *         且指令是 AT+NAME/AT+PSWD/AT+UART=波特率,停止位,校验/AT+ROLE/AT+RESET。
 *         本工程数据通道是 115200，此函数若开启需先把 BT_USART_HANDLE 切到 AT 模式。
 */
static void bt_module_config(void)
{
#if BT_AT_CONFIG
    HAL_Delay(1000); // 等待模块上电稳定

    bt_printf("AT+NAME=STM32F103\r\n"); // 设备名
    HAL_Delay(500);

    bt_printf("AT+PSWD=0000\r\n"); // 配对码
    HAL_Delay(500);

    bt_printf("AT+UART=115200,0,0\r\n"); // 波特率,停止位,校验
    HAL_Delay(500);

    bt_printf("AT+ROLE=0\r\n"); // 0=从机
    HAL_Delay(500);

    bt_printf("AT+RESET\r\n"); // 复位使配置生效
    HAL_Delay(1000);
#else
    // 保持模组原有配置：不占用启动时间，也不向对端灌 AT 字符串
#endif
}

/**
 * @brief  蓝牙模块测试函数
 */
void bt_test(void)
{
    log_print("Bluetooth test: Hello from STM32F103\r\n");
    log_print("Current baud rate: %d\r\n", BT_BAUDRATE);
}
