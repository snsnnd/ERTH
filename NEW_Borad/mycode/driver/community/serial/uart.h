#ifndef UART_H
#define UART_H

#include "main.h"
#include <stdint.h>
#include <stdlib.h>

// 串口实例结构体（每个串口对应一个实例）
typedef struct
{
    UART_HandleTypeDef *huart; // 串口句柄（如&huart1、&huart2）
    uint8_t *rx_buf;           // 接收缓冲区（动态分配或静态数组）
    uint16_t rx_buf_size;      // 缓冲区大小
    uint16_t rx_len;           // 当前接收长度
    uint8_t frame_tail;        // 帧尾标记（用于帧同步，如'#'）
} UartComm_HandleTypeDef;

/**
 * @brief 初始化串口通信实例
 * @param handle：串口实例指针
 * @param huart：HAL库串口句柄（如&huart1）
 * @param rx_buf：接收缓冲区（外部提供，可静态数组或动态分配）
 * @param rx_buf_size：接收缓冲区大小
 * @param frame_tail：帧尾标记（如'#'）
 * @return 0=成功，-1=参数无效
 */
int8_t uart_comm_init(UartComm_HandleTypeDef *handle,
                      UART_HandleTypeDef *huart,
                      uint8_t *rx_buf,
                      uint16_t rx_buf_size,
                      uint8_t frame_tail);

/**
 * @brief 发送数据（阻塞模式）
 * @param handle：串口实例指针
 * @param data：待发送数据
 * @param len：数据长度
 * @param timeout：超时时间(ms)
 * @return HAL状态（HAL_OK/HAL_ERROR等）
 */
HAL_StatusTypeDef uart_comm_send(UartComm_HandleTypeDef *handle,
                                 const uint8_t *data,
                                 uint16_t len,
                                 uint32_t timeout);

/**
 * @brief 串口接收中断回调（由HAL库触发，需传入对应实例）
 * @param handle：串口实例指针
 * @param data：本次接收的字节
 */
void uart_comm_rx_callback(UartComm_HandleTypeDef *handle, uint8_t data);

/**
 * @brief 检查是否接收到完整帧（根据帧尾判断）
 * @param handle：串口实例指针
 * @return 1=帧完整，0=未完整
 */
uint8_t uart_comm_is_frame_complete(UartComm_HandleTypeDef *handle);

/**
 * @brief 获取当前接收缓冲区数据及长度
 * @param handle：串口实例指针
 * @param buf：输出缓冲区指针
 * @param len：输出数据长度
 */
void uart_comm_get_rx_buf(UartComm_HandleTypeDef *handle,
                          uint8_t **buf,
                          uint16_t *len);

/**
 * @brief 重置接收缓冲区（清空数据，准备下次接收）
 * @param handle：串口实例指针
 */
void uart_comm_reset_rx_buf(UartComm_HandleTypeDef *handle);

#endif
