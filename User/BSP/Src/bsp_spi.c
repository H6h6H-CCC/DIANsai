#include "bsp_spi.h"

#include "main.h"
#include "spi.h"

#include <stddef.h>
#include <string.h>

#define BSP_SPI_READ_BIT   0x80U
#define BSP_SPI_ADDRESS_MASK 0x7FU

static bsp_spi_status_t BSP_SpiMapStatus(HAL_StatusTypeDef status)
{
    switch (status)
    {
    case HAL_OK:      return BSP_SPI_STATUS_OK;
    case HAL_BUSY:    return BSP_SPI_STATUS_BUSY;
    case HAL_TIMEOUT: return BSP_SPI_STATUS_TIMEOUT;
    default:          return BSP_SPI_STATUS_ERROR;
    }
}

static void BSP_Spi1Select(void)
{
    HAL_GPIO_WritePin(spics_GPIO_Port, spics_Pin, GPIO_PIN_RESET);
}

static void BSP_Spi1Deselect(void)
{
    HAL_GPIO_WritePin(spics_GPIO_Port, spics_Pin, GPIO_PIN_SET);
}

bsp_spi_status_t BSP_Spi1WriteRegister(uint8_t register_address,
                                       const uint8_t *data,
                                       uint16_t length,
                                       uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;
    uint8_t command;

    if ((register_address > BSP_SPI_ADDRESS_MASK) ||
        (data == NULL) || (length == 0U))
    {
        return BSP_SPI_STATUS_ERROR;
    }

    command = register_address & BSP_SPI_ADDRESS_MASK;
    BSP_Spi1Select();
    status = HAL_SPI_Transmit(&hspi1, &command, 1U, timeout_ms);
    if (status == HAL_OK)
    {
        status = HAL_SPI_Transmit(&hspi1, data, length, timeout_ms);
    }
    BSP_Spi1Deselect();

    return BSP_SpiMapStatus(status);
}

bsp_spi_status_t BSP_Spi1ReadRegister(uint8_t register_address,
                                      uint8_t *data,
                                      uint16_t length,
                                      uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;
    uint8_t command;

    if ((register_address > BSP_SPI_ADDRESS_MASK) ||
        (data == NULL) || (length == 0U))
    {
        return BSP_SPI_STATUS_ERROR;
    }

    command = register_address | BSP_SPI_READ_BIT;
    BSP_Spi1Select();
    status = HAL_SPI_Transmit(&hspi1, &command, 1U, timeout_ms);
    if (status == HAL_OK)
    {
        /* 主机接收数据时必须同时发送占位字节，以产生 SPI 时钟。 */
        memset(data, 0xFF, length);
        status = HAL_SPI_Receive(&hspi1, data, length, timeout_ms);
    }
    BSP_Spi1Deselect();

    return BSP_SpiMapStatus(status);
}
