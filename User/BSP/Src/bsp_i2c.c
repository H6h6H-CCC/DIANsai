#include "bsp_i2c.h"

#include "i2c.h"

static bsp_i2c_status_t BSP_I2cMapStatus(HAL_StatusTypeDef status)
{
    switch (status)
    {
    case HAL_OK:      return BSP_I2C_STATUS_OK;
    case HAL_BUSY:    return BSP_I2C_STATUS_BUSY;
    case HAL_TIMEOUT: return BSP_I2C_STATUS_TIMEOUT;
    default:          return BSP_I2C_STATUS_ERROR;
    }
}

bsp_i2c_status_t BSP_I2c1Write(uint8_t address_7bit,
                               uint8_t prefix,
                               const uint8_t *data,
                               uint16_t length,
                               uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    if ((address_7bit > 0x7FU) || (data == NULL) || (length == 0U))
    {
        return BSP_I2C_STATUS_ERROR;
    }

    /* HAL 接口需要左移一位后的地址；prefix 作为设备协议的首字节发送。 */
    status = HAL_I2C_Mem_Write(&hi2c1,
                               (uint16_t)address_7bit << 1U,
                               prefix,
                               I2C_MEMADD_SIZE_8BIT,
                               (uint8_t *)data,
                               length,
                               timeout_ms);

    return BSP_I2cMapStatus(status);
}

bsp_i2c_status_t BSP_I2c1Read(uint8_t address_7bit,
                              uint8_t register_address,
                              uint8_t *data,
                              uint16_t length,
                              uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    if ((address_7bit > 0x7FU) || (data == NULL) || (length == 0U))
    {
        return BSP_I2C_STATUS_ERROR;
    }

    /* HAL 会先发送寄存器地址，再产生重复起始条件并连续读取数据。 */
    status = HAL_I2C_Mem_Read(&hi2c1,
                              (uint16_t)address_7bit << 1U,
                              register_address,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              length,
                              timeout_ms);

    return BSP_I2cMapStatus(status);
}
