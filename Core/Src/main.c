/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
/* USB CDC暂时停用：需要恢复时取消本行和MX_USB_DEVICE_Init()的注释。 */
// #include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc3_uart_report.h"
#include "jy61p.h"
#include "atk_ms53l0m.h"
#include "moter.h"
#include "btn.h"  /* TIM1 switches between moter and BTN in app_config.h. */
#include "encoder.h"
#include "shijue.h"
#include "doji.h"
#include "Emm_V5.h"
#include "gray.h"
#include "TJC_SCREEN.h"
#include "app_config.h"
#include "bsp_btn.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "ball_control.h"
#include "state.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PACKET_TIMEOUT 100  
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t count;
uint8_t count1;
uint8_t count2;
uint8_t rxBuffer1[256]= {0};
uint8_t rxBuffer4[256];
uint8_t rxBuffer5[256];
uint8_t rxBuffer2[256];
uint8_t rxBuffer3[256];
volatile uint16_t g_rx2_size = 0U;
volatile uint32_t g_jy61_rx_event_count = 0U;
volatile uint32_t g_jy61_rx_byte_count = 0U;
volatile uint32_t g_vision_rx_event_count = 0U;
volatile uint32_t g_vision_rx_byte_count = 0U;
volatile uint32_t g_vision_rx_restart_fail_count = 0U;
volatile uint32_t g_vision_uart_error_count = 0U;
volatile uint32_t g_vision_uart_last_error = 0U;
volatile uint32_t g_debug_rx_restart_fail_count = 0U;
volatile uint32_t g_debug_uart_error_count = 0U;
volatile uint32_t g_debug_uart_last_error = 0U;
uint8_t shijue[10];
char displayBuffer[20];
uint16_t atk_id = 0;
uint16_t atk_distance = 0;
uint8_t atk_ready = 0;
uint8_t atk_last_err = ATK_MS53L0M_ERROR;
volatile uint8_t g_roll_flag_vel = 0;
volatile uint8_t g_roll_flag_pos = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Main_AppInit(void)
{
    /* 启动周期定时器和各串口的空闲中断 DMA 接收。 */
    BSP_TimeStartPeriodic();
    (void)BSP_UartStartReceiveToIdleDma(BSP_UART_1, rxBuffer1, sizeof(rxBuffer1));
    if (BSP_UartStartReceiveToIdleDma(BSP_UART_2,
                                      rxBuffer2,
                                      sizeof(rxBuffer2)) != BSP_STATUS_OK)
    {
        g_debug_rx_restart_fail_count++;
    }
    (void)BSP_UartStartReceiveToIdleDma(BSP_UART_3, rxBuffer3, sizeof(rxBuffer3));
    (void)BSP_UartStartReceiveToIdleDma(BSP_UART_4, rxBuffer4, sizeof(rxBuffer4));
    (void)BSP_UartStartReceiveToIdleDma(BSP_UART_5, rxBuffer5, sizeof(rxBuffer5));

    /* 初始化当前固件配置需要使用的设备。 */
    BSP_ServoInit();
    BallControl_Init();
    Encoder3_Init();
    Encoder4_Init();

    /* 默认不启用激光测距，避免占用当前调试输出。 */
    atk_ready = 0U;
    atk_last_err = ATK_MS53L0M_ERROR;

#if APP_UART4_MODE == APP_UART4_MODE_JY61P
    /* Recover the required packet types first, then switch to 100 Hz. */
    JY61P_Unlock();
    BSP_DelayMs(200U);
    JY61P_WriteRegister(0x02U, 0x000EU);
    BSP_DelayMs(200U);
    JY61P_WriteRegister(0x03U, 0x09);
    BSP_DelayMs(200U);
    JY61P_Save();
    BSP_DelayMs(200U);
#endif

#if APP_TIM1_MODE == APP_TIM1_MODE_MOTER
    Moter_Init();
#elif APP_TIM1_MODE == APP_TIM1_MODE_BTN
    BTN_Init();
#endif

#if APP_REPORT_ADC
    Adc3UartReport_Init();
    (void)Adc3UartReport_Start();
#endif

    /* 设置电机和舵机的上电初始输出。 */
#if APP_TIM1_MODE == APP_TIM1_MODE_MOTER
    /* 上电时保持四路电机停止。 */
    Moter_A(0);
    Moter_B(0);
    Moter_C(0);
    Moter_D(0);
#endif
    /* OLED、按键和题目逻辑统一由 state 模块管理。 */
    State_Init();
    /* PA15 舵机由 BallControl 管理，默认保持 1750 us 机械中位。 */
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (BSP_UartMatches(BSP_UART_4, huart))
    {
#if APP_UART4_MODE == APP_UART4_MODE_JY61P
      uint16_t i;
      g_jy61_rx_event_count++;
      g_jy61_rx_byte_count += Size;
      for (i = 0U; i < Size; i++)
      {
        ProcessReceivedData(rxBuffer4[i]);
      }
#elif APP_UART4_MODE == APP_UART4_MODE_ENCODER
      /* UART4 当前只发送编码器速度，收到的数据暂不解析也不回发。 */
      (void)Size;
#endif
      (void)BSP_UartStartReceiveToIdleDma(BSP_UART_4,
                                          rxBuffer4,
                                          sizeof(rxBuffer4));
    }
    else if (BSP_UartMatches(BSP_UART_1, huart))
    {
      g_vision_rx_event_count++;
      g_vision_rx_byte_count += Size;
      Shijue_ProcessRxBuffer(rxBuffer1, Size);
      if (BSP_UartStartReceiveToIdleDma(BSP_UART_1,
                                        rxBuffer1,
                                        sizeof(rxBuffer1)) != BSP_STATUS_OK)
      {
        g_vision_rx_restart_fail_count++;
      }
    }
    else if (BSP_UartMatches(BSP_UART_2, huart))
    {
        g_rx2_size = Size;
#if APP_USART2_MODE == APP_USART2_MODE_ATK_TOF
        atk_ms53l0m_uart_rx_event(rxBuffer2, Size);
#else
        State_DebugRx(rxBuffer2, Size);
#endif
        if (BSP_UartStartReceiveToIdleDma(BSP_UART_2,
                                          rxBuffer2,
                                          sizeof(rxBuffer2)) != BSP_STATUS_OK)
        {
            g_debug_rx_restart_fail_count++;
        }
    }
    else if (BSP_UartMatches(BSP_UART_3, huart))
    {
        TJC_OnRx(rxBuffer3, Size);
        (void)BSP_UartStartReceiveToIdleDma(BSP_UART_3,
                                            rxBuffer3,
                                            sizeof(rxBuffer3));
    }
    else if (BSP_UartMatches(BSP_UART_5, huart))
    {
        (void)Size;
        (void)BSP_UartStartReceiveToIdleDma(BSP_UART_5,
                                            rxBuffer5,
                                            sizeof(rxBuffer5));
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (BSP_UartMatches(BSP_UART_1, huart))
    {
        /* USART1 DMA遇到线路错误后会停止，立即重启以恢复视觉数据流。 */
        g_vision_uart_error_count++;
        g_vision_uart_last_error = huart->ErrorCode;
        if (BSP_UartStartReceiveToIdleDma(BSP_UART_1,
                                          rxBuffer1,
                                          sizeof(rxBuffer1)) != BSP_STATUS_OK)
        {
            g_vision_rx_restart_fail_count++;
        }
    }
    else if (BSP_UartMatches(BSP_UART_2, huart))
    {
        /* USART2 线路错误会终止 DMA 接收，立即重启以保留在线调参能力。 */
        g_debug_uart_error_count++;
        g_debug_uart_last_error = huart->ErrorCode;
        if (BSP_UartStartReceiveToIdleDma(BSP_UART_2,
                                          rxBuffer2,
                                          sizeof(rxBuffer2)) != BSP_STATUS_OK)
        {
            g_debug_rx_restart_fail_count++;
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_TIM8_Init();
  MX_ADC3_Init();
  MX_TIM9_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USB CDC暂时停用，需要使用USB视觉通信时取消注释。 */
  // MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
    Main_AppInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* 临时舵机调试：PA15/PB3/PB10/PB11 四路均固定为1750 us。 */

    while (1)
    {
      State_RunCurrent();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM9)
  {
    static uint8_t encoder_tick = 0U;

    count++;
    encoder_tick++;

    if (encoder_tick >= 10U)
    {
      encoder_tick = 0U;
      Encoder3_Update10ms();
      Encoder4_Update10ms();
    }

    if (count >= 1000U)
    {
      count = 0;
      count1++;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
