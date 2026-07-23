#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdint.h>

/*
 * SPI1 板级接口
 *
 * 当前硬件绑定为 SPI1：PA5=SCK，PA6=MISO，PA7=MOSI，PE8=CS。
 * SPI1 使用模式 0、8 位数据、MSB 先行，时钟约 5.25 MHz。
 * 上层设备驱动不直接引用 hspi1、GPIO 和 STM32 HAL。
 */

/** BSP 对 HAL SPI 返回状态的统一表示。 */
typedef enum
{
    BSP_SPI_STATUS_OK = 0,  /**< 传输完成。 */
    BSP_SPI_STATUS_BUSY,    /**< SPI 外设正忙。 */
    BSP_SPI_STATUS_TIMEOUT, /**< 阻塞等待超时。 */
    BSP_SPI_STATUS_ERROR    /**< 参数无效或其他通信错误。 */
} bsp_spi_status_t;

/**
 * @brief 通过 SPI1 向指定的 7 位寄存器地址连续写入数据。
 * @param register_address 设备内部寄存器地址，最高位由 BSP 自动清零为写命令。
 * @param data             待写入数据首地址。
 * @param length           写入长度，必须大于 0。
 * @param timeout_ms       每次阻塞传输的最大等待时间，单位为毫秒。
 * @return BSP_SPI_STATUS_OK 表示写入完成，其他值表示忙、超时或错误。
 * @note  BSP 会在整次传输期间拉低 PE8，结束后重新拉高；本接口不使用 DMA。
 */
bsp_spi_status_t BSP_Spi1WriteRegister(uint8_t register_address,
                                       const uint8_t *data,
                                       uint16_t length,
                                       uint32_t timeout_ms);

/**
 * @brief 通过 SPI1 从指定的 7 位寄存器地址开始连续读取数据。
 * @param register_address 设备内部寄存器地址，最高位由 BSP 自动置一为读命令。
 * @param data             接收缓冲区首地址。
 * @param length           读取长度，必须大于 0。
 * @param timeout_ms       每次阻塞传输的最大等待时间，单位为毫秒。
 * @return BSP_SPI_STATUS_OK 表示读取完成，其他值表示忙、超时或错误。
 * @note  连续读取要求设备已经启用寄存器地址自动递增。
 */
bsp_spi_status_t BSP_Spi1ReadRegister(uint8_t register_address,
                                      uint8_t *data,
                                      uint16_t length,
                                      uint32_t timeout_ms);

#endif /* BSP_SPI_H */
