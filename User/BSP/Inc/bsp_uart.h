#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

/*
 * UART 通用板级接口
 *
 * 上层使用 BSP_UART_1~6，不直接引用 huart1~huart6。
 * 调用发送/接收函数前，必须完成对应 UART 和 DMA 的 CubeMX 初始化。
 */

/** BSP 对 HAL 返回状态的统一表示。 */
typedef enum
{
    BSP_STATUS_OK = 0,  /**< 操作已成功启动或完成。 */
    BSP_STATUS_BUSY,    /**< 外设正忙，本次操作未启动。 */
    BSP_STATUS_TIMEOUT, /**< 阻塞操作等待超时。 */
    BSP_STATUS_ERROR    /**< 参数无效或 HAL 返回其他错误。 */
} bsp_status_t;

/** UART 逻辑编号，与 USART1~3、UART4~5、USART6 一一对应。 */
typedef enum
{
    BSP_UART_1 = 0,  /**< USART1。 */
    BSP_UART_2,      /**< USART2。 */
    BSP_UART_3,      /**< USART3。 */
    BSP_UART_4,      /**< UART4。 */
    BSP_UART_5,      /**< UART5。 */
    BSP_UART_6       /**< USART6。 */
} bsp_uart_t;

/**
 * @brief 以阻塞方式发送一段字节数据。
 * @param uart       UART 逻辑编号。
 * @param data       待发送数据首地址。
 * @param length     发送字节数，必须大于 0。
 * @param timeout_ms 最大阻塞时间，单位为毫秒。
 * @return BSP_STATUS_OK 表示发送完成，其他值表示忙、超时或错误。
 */
bsp_status_t BSP_UartSend(bsp_uart_t uart,
                          const uint8_t *data,
                          uint16_t length,
                          uint32_t timeout_ms);

/**
 * @brief 使用 DMA 启动一次非阻塞发送。
 * @param uart   UART 逻辑编号。
 * @param data   待发送数据首地址。
 * @param length 发送字节数，必须大于 0。
 * @return BSP_STATUS_OK 只表示 DMA 已成功启动，不代表数据已经发送完毕。
 * @note  DMA 完成前，data 指向的缓冲区必须保持有效且不能被修改。
 */
bsp_status_t BSP_UartSendDma(bsp_uart_t uart,
                             const uint8_t *data,
                             uint16_t length);

/**
 * @brief 启动 UART DMA 接收，并在总线空闲时触发接收事件回调。
 * @param uart   UART 逻辑编号。
 * @param buffer 接收缓冲区首地址。
 * @param size   缓冲区总字节数，必须大于 0。
 * @return BSP_STATUS_OK 表示接收已启动，其他值表示忙或错误。
 * @note  收到数据后 HAL 会调用 HAL_UARTEx_RxEventCallback()；回调处理完后需再次调用本函数。
 * @note  接收期间 buffer 必须一直有效，通常应使用全局或 static 数组。
 */
bsp_status_t BSP_UartStartReceiveToIdleDma(bsp_uart_t uart,
                                           uint8_t *buffer,
                                           uint16_t size);

/**
 * @brief 查询 UART 是否可以开始新的发送。
 * @param uart UART 逻辑编号。
 * @return 1=发送状态空闲，0=UART 无效或正在发送。
 */
uint8_t BSP_UartTxReady(bsp_uart_t uart);

/**
 * @brief 判断 HAL 回调传入的句柄是否属于指定 UART。
 * @param uart     UART 逻辑编号。
 * @param hal_uart HAL 回调传入的 UART_HandleTypeDef 指针；使用 void* 避免头文件暴露 HAL 类型。
 * @return 1=匹配，0=不匹配。
 */
uint8_t BSP_UartMatches(bsp_uart_t uart, const void *hal_uart);

#endif /* BSP_UART_H */
