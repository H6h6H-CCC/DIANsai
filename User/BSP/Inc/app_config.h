#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * 应用资源配置
 *
 * 这里决定同一个硬件资源在本次固件中交给哪个设备使用。
 * 这些都是编译期配置：修改后必须重新编译并下载，运行时不会自动切换。
 * 每组只修改标有“当前选择”的一行，选项对应的数字定义不要修改。
 */

/* USART2：普通数据接收，或 ATK 激光测距模块。 */
#define APP_USART2_MODE_DATA     1
#define APP_USART2_MODE_ATK_TOF  2
/* 当前选择：APP_USART2_MODE_DATA / APP_USART2_MODE_ATK_TOF。 */
#define APP_USART2_MODE          APP_USART2_MODE_DATA

/* UART5：Emm_V5 步进电机，或 DOJI 总线舵机。 */
#define APP_UART5_MODE_EMM_V5  1
#define APP_UART5_MODE_DOJI    2
/* 当前选择：APP_UART5_MODE_EMM_V5 / APP_UART5_MODE_DOJI。 */
#define APP_UART5_MODE         APP_UART5_MODE_EMM_V5

/* UART4：每秒发送一次调试报告，或连接 JY61P IMU。 */
#define APP_UART4_MODE_DEBUG  1
#define APP_UART4_MODE_JY61P  2
/* 当前选择：APP_UART4_MODE_DEBUG / APP_UART4_MODE_JY61P。 */
#define APP_UART4_MODE        APP_UART4_MODE_DEBUG

/* TIM1 CH1~4：moter 四路电机，或 BTN 两组双 PWM 电机。 */
#define APP_TIM1_MODE_MOTER  1
#define APP_TIM1_MODE_BTN    2
/* 当前选择：APP_TIM1_MODE_MOTER / APP_TIM1_MODE_BTN。 */
#define APP_TIM1_MODE        APP_TIM1_MODE_MOTER

/* IMU660RC：与 OLED 共用 I2C1，或使用独立的 SPI1 和 PE8 片选。 */
#define APP_IMU660RC_BUS_I2C  1
#define APP_IMU660RC_BUS_SPI  2
/* 当前选择：APP_IMU660RC_BUS_I2C / APP_IMU660RC_BUS_SPI。 */
#define APP_IMU660RC_BUS      APP_IMU660RC_BUS_I2C

/* UART4 为 DEBUG 模式时，选择一条报告中包含哪些字段：1=发送，0=不发送。 */
#define APP_REPORT_VISION   1  /* 视觉信息 */
#define APP_REPORT_ADC      0  /* ADC4/ADC5 电压 */
#define APP_REPORT_ENCODER  0  /* 编码器计数 */

#endif /* APP_CONFIG_H */
