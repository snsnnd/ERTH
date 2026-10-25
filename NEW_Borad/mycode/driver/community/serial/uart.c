#include "uart.h"

/**
 * @brief 初始化串口通信实例
 */
int8_t uart_comm_init(UartComm_HandleTypeDef *handle,
                      UART_HandleTypeDef *huart,
                      uint8_t *rx_buf,
                      uint16_t rx_buf_size,
                      uint8_t frame_tail)
{
    // 参数合法性检查
    if (handle == NULL || huart == NULL || rx_buf == NULL || rx_buf_size == 0)
    {
        return -1;
    }

    // 初始化实例参数
    handle->huart = huart;
    handle->rx_buf = rx_buf;
    handle->rx_buf_size = rx_buf_size;
    handle->rx_len = 0;
    handle->frame_tail = frame_tail;

    // 启动中断接收（每次1字节）
    if (HAL_UART_Receive_IT(handle->huart, handle->rx_buf, 1) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

/**
 * @brief 发送数据
 */
HAL_StatusTypeDef uart_comm_send(UartComm_HandleTypeDef *handle,
                                 const uint8_t *data,
                                 uint16_t len,
                                 uint32_t timeout)
{
    if (handle == NULL || data == NULL || len == 0)
    {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit(handle->huart, data, len, timeout);
}

/**
 * @brief 接收中断回调（每接收1字节触发）
 */
void uart_comm_rx_callback(UartComm_HandleTypeDef *handle, uint8_t data)
{
    if (handle == NULL)
    {
        return;
    }

    // 缓冲区溢出保护：超过大小则重置
    if (handle->rx_len >= handle->rx_buf_size - 1)
    {
        uart_comm_reset_rx_buf(handle);
    }

    // 存储接收数据
    handle->rx_buf[handle->rx_len++] = data;

    // 继续接收下一字节（注意：HAL_UART_Receive_IT的buf参数是"下次接收的存储地址"）
    HAL_UART_Receive_IT(handle->huart, &handle->rx_buf[handle->rx_len], 1);
}

/**
 * @brief 检查帧是否完整（最后一个字节是否为帧尾）
 */
uint8_t uart_comm_is_frame_complete(UartComm_HandleTypeDef *handle)
{
    if (handle == NULL || handle->rx_len == 0)
    {
        return 0;
    }
    return (handle->rx_buf[handle->rx_len - 1] == handle->frame_tail) ? 1 : 0;
}

/**
 * @brief 获取接收缓冲区信息
 */
void uart_comm_get_rx_buf(UartComm_HandleTypeDef *handle,
                          uint8_t **buf,
                          uint16_t *len)
{
    if (handle == NULL)
    {
        return;
    }
    if (buf != NULL)
    {
        *buf = handle->rx_buf;
    }
    if (len != NULL)
    {
        *len = handle->rx_len;
    }
}

/**
 * @brief 重置接收缓冲区
 */
void uart_comm_reset_rx_buf(UartComm_HandleTypeDef *handle)
{
    if (handle == NULL)
    {
        return;
    }
    handle->rx_len = 0;
    // 可选：清空缓冲区（非必须，后续写入会覆盖）
    // memset(handle->rx_buf, 0, handle->rx_buf_size);
}
