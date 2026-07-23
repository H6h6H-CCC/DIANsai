#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdint.h>

/*
 * I2C1 板级接口
 *
 * 当前硬件绑定为 I2C1：PB6=SCL，PB7=SDA。
 * 上层设备驱动只使用 7 位设备地址，不直接引用 hi2c1 和 STM32 HAL。
 */

/** BSP 对 HAL I2C 返回状态的统一表示。 */
typedef enum
{
    BSP_I2C_STATUS_OK = 0,  /**< 发送完成。 */
    BSP_I2C_STATUS_BUSY,    /**< I2C 总线或外设正忙。 */
    BSP_I2C_STATUS_TIMEOUT, /**< 阻塞等待超时。 */
    BSP_I2C_STATUS_ERROR    /**< 参数无效、设备无应答或其他错误。 */
} bsp_i2c_status_t;

/**
 * @brief 通过 I2C1 以阻塞方式发送“前缀字节 + 数据”。
 * @param address_7bit I2C 设备的 7 位地址，例如 OLED 为 0x3C。
 * @param prefix       设备协议要求的数据前缀，例如 OLED 的命令前缀为 0x00。
 * @param data         待发送数据首地址。
 * @param length       数据长度，必须大于 0。
 * @param timeout_ms   最大阻塞时间，单位为毫秒。
 * @return BSP_I2C_STATUS_OK 表示发送完成，其他值表示忙、超时或错误。
 * @note  本接口不使用 DMA；等待发送期间，中断仍可正常响应。
 */
bsp_i2c_status_t BSP_I2c1Write(uint8_t address_7bit,
                               uint8_t prefix,
                               const uint8_t *data,
                               uint16_t length,
                               uint32_t timeout_ms);

/**
 * @brief 通过 I2C1 从指定的 8 位寄存器地址开始连续读取数据。
 * @param address_7bit I2C 设备的 7 位地址，例如 IMU660RC 的 SA0 接地时为 0x6A。
 * @param register_address 设备内部的 8 位寄存器地址。
 * @param data         接收缓冲区首地址。
 * @param length       读取长度，必须大于 0。
 * @param timeout_ms   最大阻塞时间，单位为毫秒。
 * @return BSP_I2C_STATUS_OK 表示读取完成，其他值表示忙、超时或错误。
 * @note  本接口不使用 DMA；设备必须支持以寄存器地址作为子地址的 I2C 访问方式。
 */
bsp_i2c_status_t BSP_I2c1Read(uint8_t address_7bit,
                              uint8_t register_address,
                              uint8_t *data,
                              uint16_t length,
                              uint32_t timeout_ms);

#endif /* BSP_I2C_H */
