#ifndef LIDRA_DRIVER_H
#define LIDRA_DRIVER_H

#include "stm32f4xx_hal.h" // 替换为HAL库头文件
#include <stdint.h>

// -------------------------- 数据结构定义 --------------------------
/**
 * @brief 环形缓冲区结构体（保持不变）
 */
typedef struct
{
    uint8_t buf[256]; // 缓冲区数组
    uint16_t head;    // 写入索引
    uint16_t tail;    // 读取索引
    uint16_t count;   // 有效数据长度
} RingBufferTypeDef;

/**
 * @brief 点云帧结构体（保持不变）
 */
typedef struct __attribute__((packed))
{
    uint8_t frame_header[2]; // 帧头（0xA5,0x5A）
    uint8_t frame_length;    // 帧长度
    uint8_t speed_h;         // 转速高位
    uint8_t speed_l;         // 转速低位
    uint8_t start_angle_h;   // 起始角度高位
    uint8_t start_angle_l;   // 起始角度低位
    struct __attribute__((packed))
    {
        uint16_t distance; // 距离（mm）
        uint8_t intensity; // 强度
    } point_cloud[16];     // 16个点云数据
    uint8_t stop_angle_h;  // 结束角度高位
    uint8_t stop_angle_l;  // 结束角度低位
    uint8_t crc;           // CRC校验值
} N10_PointCloud_FrameTypeDef;

// 外部引用CubeMX生成的UART句柄（需在CubeMX中配置UART5）
extern UART_HandleTypeDef huart5;

// -------------------------- 对外接口声明 --------------------------
void n10_init(void);
void n10_start_motor(void);
void n10_stop_motor(void);
uint8_t n10_process_frame(N10_PointCloud_FrameTypeDef *frame);
float n10_get_speed(N10_PointCloud_FrameTypeDef *frame);
float n10_get_start_angle(N10_PointCloud_FrameTypeDef *frame);
float n10_get_stop_angle(N10_PointCloud_FrameTypeDef *frame);
int16_t n10_get_angles_and_distances(float *angles, float *distances, uint8_t max_count);
void n10_test(void);

#endif
