#include "imu660rc.h"

#include "app_config.h"
#include "bsp_time.h"

#if APP_IMU660RC_BUS == APP_IMU660RC_BUS_I2C
#include "bsp_i2c.h"
#elif APP_IMU660RC_BUS == APP_IMU660RC_BUS_SPI
#include "bsp_spi.h"
#else
#error "APP_IMU660RC_BUS has an invalid value"
#endif

#include <stddef.h>

#define IMU660RC_I2C_ADDRESS          0x6AU
#define IMU660RC_EXPECTED_ID          0x70U
#define IMU660RC_BUS_TIMEOUT_MS       20U

#define IMU660RC_REG_WHO_AM_I         0x0FU
#define IMU660RC_REG_CTRL1            0x10U
#define IMU660RC_REG_CTRL2            0x11U
#define IMU660RC_REG_CTRL3            0x12U
#define IMU660RC_REG_CTRL6            0x15U
#define IMU660RC_REG_CTRL8            0x17U
#define IMU660RC_REG_OUT_TEMP_L       0x20U

#define IMU660RC_CTRL3_DEFAULT        0x44U
#define IMU660RC_CTRL3_SW_RESET       0x01U
#define IMU660RC_ODR_240_HZ           0x07U
#define IMU660RC_GYRO_FS_2000_DPS     0x04U
#define IMU660RC_ACCEL_FS_8_G         0x02U

#define IMU660RC_ACCEL_G_PER_LSB      0.000244f
#define IMU660RC_GYRO_DPS_PER_LSB     0.070f
#define IMU660RC_TEMP_LSB_PER_C       256.0f
#define IMU660RC_TEMP_ZERO_C          25.0f

static imu660rc_status_t IMU660RC_BusWriteRegister(uint8_t register_address,
                                                    uint8_t value)
{
#if APP_IMU660RC_BUS == APP_IMU660RC_BUS_I2C
    bsp_i2c_status_t status;

    status = BSP_I2c1Write(IMU660RC_I2C_ADDRESS,
                           register_address,
                           &value,
                           1U,
                           IMU660RC_BUS_TIMEOUT_MS);

    return (status == BSP_I2C_STATUS_OK) ? IMU660RC_STATUS_OK
                                         : IMU660RC_STATUS_BUS_ERROR;
#else
    bsp_spi_status_t status;

    status = BSP_Spi1WriteRegister(register_address,
                                   &value,
                                   1U,
                                   IMU660RC_BUS_TIMEOUT_MS);

    return (status == BSP_SPI_STATUS_OK) ? IMU660RC_STATUS_OK
                                         : IMU660RC_STATUS_BUS_ERROR;
#endif
}

static imu660rc_status_t IMU660RC_BusReadRegisters(uint8_t register_address,
                                                    uint8_t *data,
                                                    uint16_t length)
{
#if APP_IMU660RC_BUS == APP_IMU660RC_BUS_I2C
    bsp_i2c_status_t status;

    status = BSP_I2c1Read(IMU660RC_I2C_ADDRESS,
                          register_address,
                          data,
                          length,
                          IMU660RC_BUS_TIMEOUT_MS);

    return (status == BSP_I2C_STATUS_OK) ? IMU660RC_STATUS_OK
                                         : IMU660RC_STATUS_BUS_ERROR;
#else
    bsp_spi_status_t status;

    status = BSP_Spi1ReadRegister(register_address,
                                  data,
                                  length,
                                  IMU660RC_BUS_TIMEOUT_MS);

    return (status == BSP_SPI_STATUS_OK) ? IMU660RC_STATUS_OK
                                         : IMU660RC_STATUS_BUS_ERROR;
#endif
}

static int16_t IMU660RC_BytesToInt16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8U));
}

imu660rc_status_t IMU660RC_ReadDeviceId(uint8_t *device_id)
{
    if (device_id == NULL)
    {
        return IMU660RC_STATUS_BUS_ERROR;
    }

    return IMU660RC_BusReadRegisters(IMU660RC_REG_WHO_AM_I,
                                     device_id,
                                     1U);
}

imu660rc_status_t IMU660RC_Init(void)
{
    imu660rc_status_t status;
    uint8_t device_id;
    uint8_t ctrl3;
    uint8_t retry;

    /* 上电后先留出时间，让传感器完成内部启动。 */
    BSP_DelayMs(20U);

    status = IMU660RC_ReadDeviceId(&device_id);
    if (status != IMU660RC_STATUS_OK)
    {
        return status;
    }
    if (device_id != IMU660RC_EXPECTED_ID)
    {
        return IMU660RC_STATUS_ID_ERROR;
    }

    status = IMU660RC_BusWriteRegister(IMU660RC_REG_CTRL3,
                                       IMU660RC_CTRL3_DEFAULT |
                                       IMU660RC_CTRL3_SW_RESET);
    if (status != IMU660RC_STATUS_OK)
    {
        return status;
    }

    /* SW_RESET 会由芯片自动清零；最多等待 100 ms。 */
    for (retry = 0U; retry < 100U; retry++)
    {
        if (IMU660RC_BusReadRegisters(IMU660RC_REG_CTRL3,
                                      &ctrl3,
                                      1U) != IMU660RC_STATUS_OK)
        {
            return IMU660RC_STATUS_BUS_ERROR;
        }
        if ((ctrl3 & IMU660RC_CTRL3_SW_RESET) == 0U)
        {
            break;
        }
        BSP_DelayMs(1U);
    }
    if (retry == 100U)
    {
        return IMU660RC_STATUS_RESET_TIMEOUT;
    }

    /* BDU 保证高低字节来自同一采样，IF_INC 允许连续读取 14 字节。 */
    status = IMU660RC_BusWriteRegister(IMU660RC_REG_CTRL3,
                                       IMU660RC_CTRL3_DEFAULT);
    if (status != IMU660RC_STATUS_OK)
    {
        return status;
    }

    status = IMU660RC_BusWriteRegister(IMU660RC_REG_CTRL8,
                                       IMU660RC_ACCEL_FS_8_G);
    if (status != IMU660RC_STATUS_OK)
    {
        return status;
    }

    status = IMU660RC_BusWriteRegister(IMU660RC_REG_CTRL6,
                                       IMU660RC_GYRO_FS_2000_DPS);
    if (status != IMU660RC_STATUS_OK)
    {
        return status;
    }

    status = IMU660RC_BusWriteRegister(IMU660RC_REG_CTRL1,
                                       IMU660RC_ODR_240_HZ);
    if (status != IMU660RC_STATUS_OK)
    {
        return status;
    }

    status = IMU660RC_BusWriteRegister(IMU660RC_REG_CTRL2,
                                       IMU660RC_ODR_240_HZ);
    if (status != IMU660RC_STATUS_OK)
    {
        return status;
    }

    /* 等待若干采样周期，使输出寄存器得到有效数据。 */
    BSP_DelayMs(20U);
    return IMU660RC_STATUS_OK;
}

imu660rc_status_t IMU660RC_ReadRaw(imu660rc_raw_data_t *data)
{
    uint8_t buffer[14];

    if (data == NULL)
    {
        return IMU660RC_STATUS_BUS_ERROR;
    }

    if (IMU660RC_BusReadRegisters(IMU660RC_REG_OUT_TEMP_L,
                                  buffer,
                                  sizeof(buffer)) != IMU660RC_STATUS_OK)
    {
        return IMU660RC_STATUS_BUS_ERROR;
    }

    data->temperature = IMU660RC_BytesToInt16(buffer[0], buffer[1]);
    data->gyro_x = IMU660RC_BytesToInt16(buffer[2], buffer[3]);
    data->gyro_y = IMU660RC_BytesToInt16(buffer[4], buffer[5]);
    data->gyro_z = IMU660RC_BytesToInt16(buffer[6], buffer[7]);
    data->accel_x = IMU660RC_BytesToInt16(buffer[8], buffer[9]);
    data->accel_y = IMU660RC_BytesToInt16(buffer[10], buffer[11]);
    data->accel_z = IMU660RC_BytesToInt16(buffer[12], buffer[13]);

    return IMU660RC_STATUS_OK;
}

imu660rc_status_t IMU660RC_Read(imu660rc_data_t *data)
{
    imu660rc_raw_data_t raw;
    imu660rc_status_t status;

    if (data == NULL)
    {
        return IMU660RC_STATUS_BUS_ERROR;
    }

    status = IMU660RC_ReadRaw(&raw);
    if (status != IMU660RC_STATUS_OK)
    {
        return status;
    }

    data->temperature_c = IMU660RC_TEMP_ZERO_C +
                          (float)raw.temperature / IMU660RC_TEMP_LSB_PER_C;
    data->gyro_x_dps = (float)raw.gyro_x * IMU660RC_GYRO_DPS_PER_LSB;
    data->gyro_y_dps = (float)raw.gyro_y * IMU660RC_GYRO_DPS_PER_LSB;
    data->gyro_z_dps = (float)raw.gyro_z * IMU660RC_GYRO_DPS_PER_LSB;
    data->accel_x_g = (float)raw.accel_x * IMU660RC_ACCEL_G_PER_LSB;
    data->accel_y_g = (float)raw.accel_y * IMU660RC_ACCEL_G_PER_LSB;
    data->accel_z_g = (float)raw.accel_z * IMU660RC_ACCEL_G_PER_LSB;

    return IMU660RC_STATUS_OK;
}
