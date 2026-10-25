#include "Lidra_driver.h"
#include <string.h>
#include <stdio.h>
#include "BT.h"

// -------------------------- 模块内部宏定义 --------------------------
#define N10_BAUDRATE 230400
#define RING_BUF_SIZE 256
#define N10_FRAME_LEN 58
#define N10_CRC_CALC_LEN 57

// -------------------------- 模块内部全局变量 --------------------------
static RingBufferTypeDef n10_ring_buf; // 环形缓冲区
static uint8_t rx_temp_byte;           // 接收临时字节

// 电机控制指令
static const uint8_t N10_MotorStopCmd[] = {
    0xA5, 0x5A, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xFA, 0xFB};

static const uint8_t N10_MotorStartCmd[] = {
    0xA5, 0x5A, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0xFA, 0xFB};

// -------------------------- 模块内部函数声明 --------------------------
static void n10_ringbuf_init(RingBufferTypeDef *rb);
static uint8_t n10_ringbuf_write(RingBufferTypeDef *rb, uint8_t data);
static uint8_t n10_ringbuf_read(RingBufferTypeDef *rb, uint8_t *data);
static uint16_t n10_ringbuf_get_count(RingBufferTypeDef *rb);
static uint8_t n10_calculate_crc(uint8_t *data, uint16_t len);
static float n10_parse_speed(uint8_t speed_h, uint8_t speed_l);
static float n10_parse_angle(uint8_t angle_h, uint8_t angle_l);

// -------------------------- 对外接口实现 --------------------------
/**
 * @brief 雷达模块初始化（HAL库版本）
 * 注：外设（UART/GPIO）初始化由CubeMX自动生成，此处仅初始化缓冲区
 */
void n10_init(void)
{
    n10_ringbuf_init(&n10_ring_buf);

    // 启动UART接收中断（HAL库需手动开启）
    HAL_UART_Receive_IT(&huart5, &rx_temp_byte, 1);

#if DEBUG_PRINT
    printf("N10 radar initialized successfully!\r\n");
#endif
}

/**
 * @brief 启动雷达电机（使用HAL库发送指令）
 */
void n10_start_motor(void)
{
    HAL_UART_Transmit(&huart5, N10_MotorStartCmd, sizeof(N10_MotorStartCmd), 500);
#if DEBUG_PRINT
    printf("N10 motor start!\r\n");
#endif
}

/**
 * @brief 停止雷达电机（使用HAL库发送指令）
 */
void n10_stop_motor(void)
{
    HAL_UART_Transmit(&huart5, N10_MotorStopCmd, sizeof(N10_MotorStopCmd), 500);
#if DEBUG_PRINT
    printf("N10 motor stop!\r\n");
#endif
}

/**
 * @brief 处理接收数据并解析点云帧（核心逻辑不变）
 */
uint8_t n10_process_frame(N10_PointCloud_FrameTypeDef *frame)
{
    uint8_t frame_buf[N10_FRAME_LEN];
    uint16_t read_cnt = 0;

    if (n10_ringbuf_get_count(&n10_ring_buf) < N10_FRAME_LEN)
        return 0;

    // 读取完整帧数据
    while (read_cnt < N10_FRAME_LEN)
    {
        n10_ringbuf_read(&n10_ring_buf, &frame_buf[read_cnt]);
        read_cnt++;
    }

    // 帧头校验
    if (frame_buf[0] != 0xA5 || frame_buf[1] != 0x5A)
    {
#if DEBUG_PRINT
        printf("Frame header error! Drop 1 byte:0x%02X\r\n", frame_buf[0]);
#endif
        // 回写剩余数据
        for (uint16_t i = 1; i < N10_FRAME_LEN; i++)
        {
            n10_ringbuf_write(&n10_ring_buf, frame_buf[i]);
        }
        return 0;
    }

    // CRC校验
    uint8_t crc_calc = n10_calculate_crc(frame_buf, N10_CRC_CALC_LEN);
    if (crc_calc != frame_buf[N10_FRAME_LEN - 1])
    {
#if DEBUG_PRINT
        printf("CRC error! Calc:0x%02X, Recv:0x%02X\r\n", crc_calc, frame_buf[N10_FRAME_LEN - 1]);
#endif
        return 0;
    }

    // 解析帧数据
    memcpy(frame, frame_buf, N10_FRAME_LEN);
    return 1;
}

/**
 * @brief  获取解析后的转速
 * @param  frame: 点云帧数据
 * @retval 转速值（Hz）
 */
float n10_get_speed(N10_PointCloud_FrameTypeDef *frame)
{
#if DEBUG_PRINT
    float speed = n10_parse_speed(frame->speed_h, frame->speed_l);
    log_print("N10 speed:%f\r\n", speed);
#endif
    return n10_parse_speed(frame->speed_h, frame->speed_l);
}

/**
 * @brief  获取解析后的起始角度
 * @param  frame: 点云帧数据
 * @retval 起始角度（度）
 */
float n10_get_start_angle(N10_PointCloud_FrameTypeDef *frame)
{
#if DEBUG_PRINT
    float angle = n10_parse_angle(frame->start_angle_h, frame->start_angle_l);
    log_print("N10 start angle:%f\r\n", angle);
#endif
    return n10_parse_angle(frame->start_angle_h, frame->start_angle_l);
}

/**
 * @brief  获取解析后的结束角度
 * @param  frame: 点云帧数据
 * @retval 结束角度（度）
 */
float n10_get_stop_angle(N10_PointCloud_FrameTypeDef *frame)
{
#if DEBUG_PRINT
    float angle = n10_parse_angle(frame->stop_angle_h, frame->stop_angle_l);
    log_print("N10 end angle:%f\r\n", angle);
#endif
    return n10_parse_angle(frame->stop_angle_h, frame->stop_angle_l);
}

/**
 * @brief 获取当前帧中所有点的角度和距离
 * @details 解析一帧完整数据，计算16个点的角度（平均分布在起始和结束角度之间）
 *          距离单位转换为米（原始数据为毫米）
 */
int16_t n10_get_angles_and_distances(float *angles, float *distances, uint8_t max_count)
{
    // 参数有效性检查
    if (angles == NULL || distances == NULL || max_count < 16)
    {
#if DEBUG_PRINT
        log_print("n10_get_angles_and_distances: invalid parameters\r\n");
#endif
        return -1;
    }

    N10_PointCloud_FrameTypeDef frame;
    // 尝试解析一帧数据
    if (n10_process_frame(&frame) != 1)
    {
#if DEBUG_PRINT
        log_print("n10_get_angles_and_distances: no valid frame\r\n");
#endif
        return 0;
    }

    // 获取起始和结束角度（度）
    float start_angle = n10_get_start_angle(&frame);
    float stop_angle = n10_get_stop_angle(&frame);

    // 计算角度步长（16个点有15个间隔）
    float angle_step = 0.0f;
    if (stop_angle >= start_angle)
    {
        angle_step = (stop_angle - start_angle) / 15.0f;
    }
    else
    {
        // 处理角度跨360度的情况（如350度到10度）
        angle_step = (stop_angle + 360.0f - start_angle) / 15.0f;
    }

    // 提取每个点的角度和距离
    for (uint8_t i = 0; i < 16; i++)
    {
        // 计算当前点角度（处理跨360度的情况）
        float current_angle = start_angle + i * angle_step;
        if (current_angle >= 360.0f)
        {
            current_angle -= 360.0f;
        }
        angles[i] = current_angle;

        // 距离转换：毫米 -> 米
        distances[i] = frame.point_cloud[i].distance / 1000.0f;

#if DEBUG_PRINT
        log_print("Point %d: angle=%.2f°, distance=%.3fm\r\n",
                  i, angles[i], distances[i]);
#endif
    }

    return 16; // 每帧固定16个点
}

/**
 * @brief 测试函数（延时替换为HAL_Delay）
 */
void n10_test()
{
    N10_PointCloud_FrameTypeDef frame;
    uint8_t frame_count = 0;

#if DEBUG_PRINT
    printf("===== Lidar Test Start =====");
#endif

    n10_start_motor();
    HAL_Delay(1000); // HAL库延时函数

    while (1)
    {
        if (n10_process_frame(&frame) == 1)
        {
            frame_count++;
#if DEBUG_PRINT
            printf("\r\n----- Valid Frame %d -----", frame_count);
            printf("Speed: %.2f Hz", n10_get_speed(&frame));
            // ...（其余打印同原代码）
#endif
        }
        HAL_Delay(100);
    }
}

// -------------------------- 模块内部实现 --------------------------
// 环形缓冲区操作（保持不变）
static void n10_ringbuf_init(RingBufferTypeDef *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    memset(rb->buf, 0, RING_BUF_SIZE);
}

static uint8_t n10_ringbuf_write(RingBufferTypeDef *rb, uint8_t data)
{
    if (rb->count >= RING_BUF_SIZE)
        return 0;
    rb->buf[rb->head] = data;
    rb->head = (rb->head + 1) % RING_BUF_SIZE;
    rb->count++;
    return 1;
}

/**
 * @brief  读取环形缓冲区
 */
static uint8_t n10_ringbuf_read(RingBufferTypeDef *rb, uint8_t *data)
{
    if (rb->count == 0)
        return 0;

    *data = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUF_SIZE;
    rb->count--;
    return 1;
}

/**
 * @brief  获取缓冲区数据长度
 */
static uint16_t n10_ringbuf_get_count(RingBufferTypeDef *rb)
{
    return rb->count;
}

// CRC计算、转速/角度解析函数（保持不变）
static uint8_t n10_calculate_crc(uint8_t *data, uint16_t len)
{
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

static float n10_parse_speed(uint8_t speed_h, uint8_t speed_l)
{
    uint16_t speed_us = (speed_h << 8) | speed_l;
    uint32_t circle_time_us = speed_us * 24;
    return 1000000.0f / circle_time_us;
}

static float n10_parse_angle(uint8_t angle_h, uint8_t angle_l)
{
    uint16_t angle_raw = (angle_h << 8) | angle_l;
    return (float)angle_raw / 100.0f;
}

// -------------------------- HAL中断回调函数 --------------------------
/**
 * @brief UART接收完成回调（HAL库机制）
 * 每次接收1字节后触发，写入环形缓冲区
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5)
    {
        n10_ringbuf_write(&n10_ring_buf, rx_temp_byte);
        // 重新开启中断接收（HAL库需手动重启）
        HAL_UART_Receive_IT(&huart5, &rx_temp_byte, 1);
    }
}
