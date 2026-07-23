#include "bsp_uart.h"

#include "usart.h"

/* 将 BSP 逻辑编号映射到 CubeMX 生成的 HAL UART 句柄。 */
static UART_HandleTypeDef *BSP_UartGetHandle(bsp_uart_t uart)
{
    switch (uart)
    {
    case BSP_UART_1: return &huart1;
    case BSP_UART_2: return &huart2;
    case BSP_UART_3: return &huart3;
    case BSP_UART_4: return &huart4;
    case BSP_UART_5: return &huart5;
    case BSP_UART_6: return &huart6;
    default:         return NULL;
    }
}

static bsp_status_t BSP_UartMapStatus(HAL_StatusTypeDef status)
{
    /* 隔离 HAL 状态类型，避免上层模块依赖 STM32 HAL。 */
    switch (status)
    {
    case HAL_OK:      return BSP_STATUS_OK;
    case HAL_BUSY:    return BSP_STATUS_BUSY;
    case HAL_TIMEOUT: return BSP_STATUS_TIMEOUT;
    default:          return BSP_STATUS_ERROR;
    }
}

bsp_status_t BSP_UartSend(bsp_uart_t uart,
                          const uint8_t *data,
                          uint16_t length,
                          uint32_t timeout_ms)
{
    UART_HandleTypeDef *handle = BSP_UartGetHandle(uart);

    /* 无效 UART、空指针和零长度都不能形成有效发送。 */
    if ((handle == NULL) || (data == NULL) || (length == 0U))
    {
        return BSP_STATUS_ERROR;
    }

    return BSP_UartMapStatus(HAL_UART_Transmit(handle,
                                               (uint8_t *)data,
                                               length,
                                               timeout_ms));
}

bsp_status_t BSP_UartSendDma(bsp_uart_t uart,
                             const uint8_t *data,
                             uint16_t length)
{
    UART_HandleTypeDef *handle = BSP_UartGetHandle(uart);

    /* HAL DMA 发送不会复制数据，发送完成前上层必须保留原缓冲区。 */
    if ((handle == NULL) || (data == NULL) || (length == 0U))
    {
        return BSP_STATUS_ERROR;
    }

    return BSP_UartMapStatus(HAL_UART_Transmit_DMA(handle,
                                                   (uint8_t *)data,
                                                   length));
}

bsp_status_t BSP_UartStartReceiveToIdleDma(bsp_uart_t uart,
                                           uint8_t *buffer,
                                           uint16_t size)
{
    UART_HandleTypeDef *handle = BSP_UartGetHandle(uart);
    HAL_StatusTypeDef status;

    if ((handle == NULL) || (buffer == NULL) || (size == 0U))
    {
        return BSP_STATUS_ERROR;
    }

    status = HAL_UARTEx_ReceiveToIdle_DMA(handle, buffer, size);
    if ((status == HAL_OK) && (handle->hdmarx != NULL))
    {
        /* 这里只关心空闲事件/接收完成事件，不使用 DMA 半传输事件。 */
        __HAL_DMA_DISABLE_IT(handle->hdmarx, DMA_IT_HT);
    }

    return BSP_UartMapStatus(status);
}

uint8_t BSP_UartTxReady(bsp_uart_t uart)
{
    UART_HandleTypeDef *handle = BSP_UartGetHandle(uart);

    /* gState 只表示发送状态；接收 DMA 可以同时处于工作状态。 */
    return (handle != NULL) && (handle->gState == HAL_UART_STATE_READY);
}

uint8_t BSP_UartMatches(bsp_uart_t uart, const void *hal_uart)
{
    /* HAL 的公共回调通过句柄区分事件来自哪个物理 UART。 */
    return BSP_UartGetHandle(uart) == (const UART_HandleTypeDef *)hal_uart;
}
