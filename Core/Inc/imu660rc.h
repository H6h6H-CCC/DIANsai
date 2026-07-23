#ifndef IMU660RC_H
#define IMU660RC_H

#include <stdint.h>

/*
 * IMU660RC 六轴传感器驱动
 *
 * 模块内部芯片为 LSM6DSV16X，通过 app_config.h 选择通信方式：
 *   I2C：CS 接 3.3V，SA0 接 GND，SCL 接 PB6，SDA 接 PB7；
 *   SPI：CS 接 PE8，SCK 接 PA5，MISO 接 PA6，MOSI 接 PA7。
 * 两种方式均使用阻塞通信，不占用 DMA，对外调用接口完全相同。
 *
 * 默认配置：加速度计 240 Hz / +/-8 g，陀螺仪 240 Hz / +/-2000 dps。
 */

/** 驱动函数的返回状态。 */
typedef enum
{
    IMU660RC_STATUS_OK = 0,       /**< 操作成功。 */
    IMU660RC_STATUS_BUS_ERROR,    /**< I2C/SPI 通信失败或传入空指针。 */
    IMU660RC_STATUS_ID_ERROR,     /**< WHO_AM_I 不是芯片规定的 0x70。 */
    IMU660RC_STATUS_RESET_TIMEOUT /**< 软件复位位未在规定时间内自动清零。 */
} imu660rc_status_t;

/** 传感器寄存器中未经单位换算的有符号原始值。 */
typedef struct
{
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} imu660rc_raw_data_t;

/** 已换算为常用物理单位的数据。 */
typedef struct
{
    float temperature_c; /**< 温度，单位为摄氏度。 */
    float gyro_x_dps;    /**< X 轴角速度，单位为度每秒。 */
    float gyro_y_dps;    /**< Y 轴角速度，单位为度每秒。 */
    float gyro_z_dps;    /**< Z 轴角速度，单位为度每秒。 */
    float accel_x_g;     /**< X 轴加速度，单位为重力加速度 g。 */
    float accel_y_g;     /**< Y 轴加速度，单位为重力加速度 g。 */
    float accel_z_g;     /**< Z 轴加速度，单位为重力加速度 g。 */
} imu660rc_data_t;

/**
 * @brief 检查芯片 ID、软件复位并启动加速度计和陀螺仪。
 * @return IMU660RC_STATUS_OK 表示初始化完成，其他值表示失败原因。
 * @note  调用前必须按 app_config.h 的选择执行 MX_I2C1_Init() 或 MX_SPI1_Init()。
 */
imu660rc_status_t IMU660RC_Init(void);

/**
 * @brief 读取芯片固定 ID。
 * @param device_id 用于保存 ID 的地址，正常应读到 0x70。
 * @return IMU660RC_STATUS_OK 表示读取成功。
 */
imu660rc_status_t IMU660RC_ReadDeviceId(uint8_t *device_id);

/**
 * @brief 一次连续读取温度、三轴角速度和三轴加速度原始值。
 * @param data 用于保存原始数据的结构体地址。
 * @return IMU660RC_STATUS_OK 表示读取成功。
 */
imu660rc_status_t IMU660RC_ReadRaw(imu660rc_raw_data_t *data);

/**
 * @brief 读取并换算温度、角速度和加速度。
 * @param data 用于保存物理量数据的结构体地址。
 * @return IMU660RC_STATUS_OK 表示读取成功。
 */
imu660rc_status_t IMU660RC_Read(imu660rc_data_t *data);

#endif /* IMU660RC_H */
