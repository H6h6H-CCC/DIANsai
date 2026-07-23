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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc3_uart_report.h"
#include "oled.h"
#include "imu660rc.h"
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
#include "bsp_input.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  MAIN_STATE_TRACK = 1,
  MAIN_STATE_STOP = 2
} MainState_t;

typedef enum
{
  MAIN_MODE_NONE = 0,
  MAIN_MODE_TRACK = 1
} MainMode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PACKET_TIMEOUT 100  
#define TRACK_BASE_PWM 350
#define TRACK_KP 12
#define TRACK_STOP_BLACK_COUNT 3U
#define TRACK_FIRST_STOP_MS 1000U
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
uint8_t txBuffer4[256];
uint8_t rxBuffer5[256];
uint8_t rxBuffer2[256];
uint8_t rxBuffer3[256];
static uint8_t vision_report_buffer[256];
volatile uint16_t g_rx2_size = 0U;
uint8_t shijue[10];
char displayBuffer[20];
uint16_t atk_id = 0;
uint16_t atk_distance = 0;
uint8_t atk_ready = 0;
uint8_t atk_last_err = ATK_MS53L0M_ERROR;
volatile uint8_t g_roll_flag_vel = 0;
volatile uint8_t g_roll_flag_pos = 0;
static volatile MainState_t g_main_state = MAIN_STATE_TRACK;
static MainMode_t g_main_mode = MAIN_MODE_NONE;
static volatile uint8_t g_uart4_mode_request = 0U;
static volatile uint8_t g_vision_report_pending = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Main_UpdateOledStatus(void)
{
    OLED_ShowNum(1, 10, g_shijue_x  , 3);
    OLED_ShowNum(2, 10, g_shijue_y    , 3);
    OLED_ShowNum(3, 10, count1  , 3);
    OLED_ShowHexNum(1, 1, rxBuffer1[0], 3);
    OLED_ShowHexNum(2, 1, rxBuffer1[1], 3);
    OLED_ShowHexNum(3, 1, rxBuffer1[2], 3);
    OLED_ShowHexNum(4, 1, rxBuffer1[3], 3);
    OLED_ShowHexNum(1, 5, rxBuffer1[4], 3);
    OLED_ShowHexNum(2, 5, rxBuffer1[5], 3);
    OLED_ShowHexNum(3, 5, rxBuffer1[6], 3);
    OLED_ShowHexNum(4, 5, rxBuffer1[7], 3);
    OLED_ShowHexNum(4, 10, g_shijue_error_flag, 2);
}

static uint8_t Main_IsKeyPressed(void)
{
    return BSP_InputAnyKeyPressed();
}

static uint8_t Main_ReadBoValue(void)
{
    return BSP_InputReadDipSwitch();
}

static MainMode_t Main_TakeUart4ModeRequest(void)
{
    uint8_t request = g_uart4_mode_request;

    if (request != 1U) {
        return MAIN_MODE_NONE;
    }

    g_uart4_mode_request = 0U;
    return (MainMode_t)request;
}

static MainMode_t Main_WaitModeSelect(void)
{
    uint8_t bo_value;
    MainMode_t uart_mode;

    Moter_A(0);
    Moter_B(0);

    while (Main_IsKeyPressed() == 0U) {
        uart_mode = Main_TakeUart4ModeRequest();
        if (uart_mode != MAIN_MODE_NONE) {
            return uart_mode;
        }
        HAL_Delay(10);
    }
    HAL_Delay(20);
    bo_value = Main_ReadBoValue();
    while (Main_IsKeyPressed() != 0U) {
        HAL_Delay(10);
    }

    if (bo_value == 1U) return MAIN_MODE_TRACK;
    return MAIN_MODE_NONE;
}

static uint8_t Main_CountBits(uint8_t value)
{
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < 8U; i++) {
        if ((value & (uint8_t)(1U << i)) != 0U) {
            count++;
        }
    }

    return count;
}

static void Main_SetState(MainState_t state)
{
    Moter_A(0);
    Moter_B(0);

    g_main_state = state;
}

static void Main_StartMode(MainMode_t mode)
{
    g_main_mode = mode;

    if (mode == MAIN_MODE_TRACK) {
        Main_SetState(MAIN_STATE_TRACK);
    } else {
        Main_SetState(MAIN_STATE_STOP);
    }
}

static void Main_TrackRun(void)
{
    static uint8_t stop_count = 0U;
    static uint8_t last_three_black = 0U;
    uint8_t gray = Gray_Read();
    uint8_t three_black = (Main_CountBits(gray) >= TRACK_STOP_BLACK_COUNT);
    int16_t error = Gray_GetError();
    int16_t turn = error * TRACK_KP;

    if ((three_black != 0U) && (last_three_black == 0U)) {
        stop_count++;
        Moter_A(0);
        Moter_B(0);

        if (stop_count >= 2U) {
            Main_SetState(MAIN_STATE_STOP);
            g_main_mode = MAIN_MODE_NONE;
            last_three_black = three_black;
            return;
        }

        HAL_Delay(TRACK_FIRST_STOP_MS);
    }

    last_three_black = three_black;

    if (gray == 0U) {
        Moter_A(TRACK_BASE_PWM);
        Moter_B(TRACK_BASE_PWM);
    } else {
        Moter_A(TRACK_BASE_PWM - turn);
        Moter_B(TRACK_BASE_PWM + turn);
    }
}

static void Main_SendDebugReport(void)
{
    int length = 0;
    int written = 0;

#if APP_UART4_MODE != APP_UART4_MODE_DEBUG
    return;
#endif

    if ((g_vision_report_pending == 0U) ||
        !BSP_UartTxReady(BSP_UART_4))
    {
        return;
    }

#if APP_REPORT_VISION
    written = snprintf((char *)&vision_report_buffer[length],
                       sizeof(vision_report_buffer) - (size_t)length,
                       "count=%u,class_id=%u,x=%u,y=%u,w=%u,h=%u,score=%.3f,label=%s",
                       g_shijue_count,
                       g_shijue_class_id,
                       g_shijue_x,
                       g_shijue_y,
                       g_shijue_w,
                       g_shijue_h,
                       (double)g_shijue_score,
                       g_shijue_label);
    if ((written < 0) || ((size_t)written >= sizeof(vision_report_buffer) - (size_t)length))
    {
        return;
    }
    length += written;
#endif

#if !APP_REPORT_VISION
    (void)g_shijue_count;
#endif

#if APP_REPORT_ADC
    written = snprintf((char *)&vision_report_buffer[length],
                       sizeof(vision_report_buffer) - (size_t)length,
                       "%sadc4=%lumV,adc5=%lumV",
                       (length > 0) ? "," : "",
                       (unsigned long)Adc3UartReport_GetCh4Mv(),
                       (unsigned long)Adc3UartReport_GetCh5Mv());
    if ((written < 0) || ((size_t)written >= sizeof(vision_report_buffer) - (size_t)length))
    {
        return;
    }
    length += written;
#endif

#if APP_REPORT_ENCODER
    written = snprintf((char *)&vision_report_buffer[length],
                       sizeof(vision_report_buffer) - (size_t)length,
                       "%senc3=%ld,enc4=%ld",
                       (length > 0) ? "," : "",
                       (long)Encoder3_GetTotal(),
                       (long)Encoder4_GetTotal());
    if ((written < 0) || ((size_t)written >= sizeof(vision_report_buffer) - (size_t)length))
    {
        return;
    }
    length += written;
#endif

    (void)written;

    if ((length > 0) && ((size_t)(length + 2) < sizeof(vision_report_buffer)))
    {
        vision_report_buffer[length++] = '\r';
        vision_report_buffer[length++] = '\n';

        if (BSP_UartSendDma(BSP_UART_4,
                            vision_report_buffer,
                            (uint16_t)length) == BSP_STATUS_OK)
        {
            g_vision_report_pending = 0U;
        }
    }
}

static void Main_AppInit(void)
{
    /* 启动周期定时器和各串口的空闲中断 DMA 接收。 */
    BSP_TimeStartPeriodic();
    (void)BSP_UartStartReceiveToIdleDma(BSP_UART_1, rxBuffer1, sizeof(rxBuffer1));
    (void)BSP_UartStartReceiveToIdleDma(BSP_UART_2, rxBuffer2, sizeof(rxBuffer2));
    (void)BSP_UartStartReceiveToIdleDma(BSP_UART_3, rxBuffer3, sizeof(rxBuffer3));
    (void)BSP_UartStartReceiveToIdleDma(BSP_UART_4, rxBuffer4, sizeof(rxBuffer4));
    (void)BSP_UartStartReceiveToIdleDma(BSP_UART_5, rxBuffer5, sizeof(rxBuffer5));

    /* 初始化当前固件配置需要使用的设备。 */
    BSP_ServoInit();
    Encoder3_Init();
    Encoder4_Init();

    /* 默认不启用激光测距，避免占用当前调试输出。 */
    atk_ready = 0U;
    atk_last_err = ATK_MS53L0M_ERROR;

#if APP_UART4_MODE == APP_UART4_MODE_JY61P
    JY61P_InitConfig();
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

    OLED_Init();

    /* IMU660RC 使用 app_config.h 选定的总线；失败时不阻塞其他功能。 */
    (void)IMU660RC_Init();


    /* 设置电机和舵机的上电初始输出。 */
#if APP_TIM1_MODE == APP_TIM1_MODE_MOTER
    Moter_A(500);
    Moter_B(500);
    Moter_C(500);
    Moter_D(500);
#endif
    BSP_ServoSetPulse(BSP_SERVO_1, 500U);
    BSP_ServoSetPulse(BSP_SERVO_2, 500U);
    BSP_ServoSetPulse(BSP_SERVO_3, 500U);
    BSP_ServoSetPulse(BSP_SERVO_4, 500U);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (BSP_UartMatches(BSP_UART_4, huart))
    {
#if APP_UART4_MODE == APP_UART4_MODE_JY61P
      uint16_t i;
      for (i = 0U; i < Size; i++)
      {
        ProcessReceivedData(rxBuffer4[i]);
      }
#else
      (void)Size;
#endif
      (void)BSP_UartStartReceiveToIdleDma(BSP_UART_4,
                                          rxBuffer4,
                                          sizeof(rxBuffer4));
    }
    else if (BSP_UartMatches(BSP_UART_1, huart))
    {
      Shijue_ProcessRxBuffer(rxBuffer1, Size);
      (void)BSP_UartStartReceiveToIdleDma(BSP_UART_1,
                                          rxBuffer1,
                                          sizeof(rxBuffer1));
    }
    else if (BSP_UartMatches(BSP_UART_2, huart))
    {
        g_rx2_size = Size;
#if APP_USART2_MODE == APP_USART2_MODE_ATK_TOF
        atk_ms53l0m_uart_rx_event(rxBuffer2, Size);
#endif
        (void)BSP_UartStartReceiveToIdleDma(BSP_UART_2,
                                            rxBuffer2,
                                            sizeof(rxBuffer2));
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
  /* USER CODE BEGIN 2 */
    Main_AppInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  // Doji_MovePos(001,1500,0010);
  // HAL_Delay(30);
  // Doji_MovePos(001,1500,0010);
  // HAL_Delay(30);
  //2200鈥斺?200
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
   // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    //rxBuffer2[0] = 0x01;
    //Main_SetState(MAIN_STATE_STOP);
    while (1)
    {
      //Main_SendDebugReport();
      Moter_A(-500);
      HAL_Delay(1000);
      Moter_A(0);
      HAL_Delay(1000);
      Moter_A(500);
      HAL_Delay(1000);
      Moter_A(1000);
      HAL_Delay(1000);
      
      


      // HAL_Delay(800);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 000);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 000);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 000);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 000);
      // Moter_A(00);
      // Moter_B(00);
      // Moter_C(000);
      // Moter_D(000);
      // HAL_Delay(800);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 500);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 500);
      // Moter_A(500);
      // Moter_B(500);
      // Moter_C(500);
      // Moter_D(500);
      // HAL_Delay(800);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);
      // Moter_A(1000);
      // Moter_B(1000);
      // Moter_C(1000);
      // Moter_D(1000);
// MainMode_t uart_mode = Main_TakeUart4ModeRequest();  
//       if (uart_mode != MAIN_MODE_NONE)
//       {
//         Main_StartMode(uart_mode);
//       }
//       else if (g_main_mode == MAIN_MODE_NONE)
//       {
//         Main_StartMode(Main_WaitModeSelect());
//       }
//       else if (g_main_state == MAIN_STATE_TRACK)
//       {
//         Main_TrackRun();
//       }

      //State_RunCurrent();
      //Main_UpdateOledStatus();
      //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2500);HAL_Delay(1000);
      
        //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 5000);
       // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 5000);
        //HAL_Delay(5000);
       // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10000);
        ///__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 10000);
       // HAL_Delay(5000);
      //  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
      //  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
      //   HAL_Delay(5000);
     
    // //   Emm_V5_MMCL_Pos_Control(3, 1, 600, 200, 1600, true, true);
    // Emm_V5_MMCL_Pos_Control(4, 1, 600, 200, 1600, true, true);
    // Emm_V5_Multi_Motor_Cmd_UART5(0);
    // HAL_Delay(100);
    // Emm_V5_MMCL_Pos_Control(3, 1, 600, 200, 1700, true, true);
    // Emm_V5_MMCL_Pos_Control(4, 1, 600, 200, 1700, true, true);
    // Emm_V5_Multi_Motor_Cmd_UART5(0);HAL_Delay(100);
      //UpdateDisplay();
	    //atk_ms53l0m_show_distance(atk_ready, atk_id, &atk_distance);
      //  OLED_ShowHexNum(1, 1, rxBuffer1[1], 2);
      //  OLED_ShowHexNum(1, 4, rxBuffer1[2], 2);
      //  OLED_ShowHexNum(1, 7, rxBuffer1[3], 2);
      //  OLED_ShowHexNum(2, 1, rxBuffer1[4], 2);
      //  OLED_ShowHexNum(2, 4, rxBuffer1[5], 2);
      //  OLED_ShowHexNum(2, 7, rxBuffer1[6], 2);
      //  OLED_ShowHexNum(3, 1, rxBuffer1[7], 2);
      //  OLED_ShowHexNum(3, 4, rxBuffer1[8], 2);
      //  OLED_ShowHexNum(3, 7, rxBuffer1[9], 2);
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

    if (encoder_tick >= 5U)
    {
      encoder_tick = 0U;
      Encoder3_Update10ms();
      Encoder4_Update10ms();
    }

    if (count >= 1000U)
    {
      count = 0;
      count1++;
      g_vision_report_pending = 1U;
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
