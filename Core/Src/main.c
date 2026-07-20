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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc3_uart_report.h"
#include "oled.h"
#include "jy61p.h"
#include "atk_ms53l0m.h"
#include "moter.h"
#include "encoder.h"
#include "shijue.h"
#include "doji.h"
#include "Emm_V5.h"
#include "gray.h"
#include "TJC_SCREEN.h"
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
#define KEY_PRESSED GPIO_PIN_RESET
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
static uint8_t vision_report_buffer[160];
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
    return (HAL_GPIO_ReadPin(KAIGUAN1_GPIO_Port, KAIGUAN1_Pin) == KEY_PRESSED) ||
           (HAL_GPIO_ReadPin(KAIGUAN2_GPIO_Port, KAIGUAN2_Pin) == KEY_PRESSED) ||
           (HAL_GPIO_ReadPin(KAIGUAN3_GPIO_Port, KAIGUAN3_Pin) == KEY_PRESSED);
}

static uint8_t Main_ReadBoValue(void)
{
    uint8_t value = 0U;

    if (HAL_GPIO_ReadPin(bo1_GPIO_Port, bo1_Pin) == GPIO_PIN_SET) value |= 0x01U;
    if (HAL_GPIO_ReadPin(bo2_GPIO_Port, bo2_Pin) == GPIO_PIN_SET) value |= 0x02U;
    if (HAL_GPIO_ReadPin(bo3_GPIO_Port, bo3_Pin) == GPIO_PIN_SET) value |= 0x04U;
    if (HAL_GPIO_ReadPin(bo4_GPIO_Port, bo4_Pin) == GPIO_PIN_SET) value |= 0x08U;

    return value;
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart4)
    {
      HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rxBuffer4, sizeof(rxBuffer4));
      __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
    }
    else if (huart == &huart1)
    {
      Shijue_ProcessRxBuffer(rxBuffer1, Size);

      HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer1, sizeof(rxBuffer1));
      __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
    else if (huart == &huart2)
    {
        g_rx2_size = Size;
        atk_ms53l0m_uart_rx_event(rxBuffer2, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxBuffer2, sizeof(rxBuffer2));
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
    else if (huart == &huart5)
    {
    
        
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rxBuffer5, sizeof(rxBuffer5));
        __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
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
  /* USER CODE BEGIN 2 */
	
    // 鍚姩UART DMA鎺ユ�?
    HAL_TIM_Base_Start_IT(&htim9);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rxBuffer1,sizeof(rxBuffer1));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4,rxBuffer4,sizeof(rxBuffer4));
	__HAL_DMA_DISABLE_IT(&hdma_uart4_rx,DMA_IT_HT);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5,rxBuffer5,sizeof(rxBuffer5));
	__HAL_DMA_DISABLE_IT(&hdma_uart5_rx,DMA_IT_HT);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,rxBuffer2,sizeof(rxBuffer2));
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
  // atk_last_err = atk_ms53l0m_init(huart2.Init.BaudRate, &atk_id);
  // if (atk_last_err == ATK_MS53L0M_EOK)
  // {
  //   atk_ready = 1;
  // }
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    Encoder3_Init();
    Encoder4_Init();
    /* Keep laser disabled here first, so UART4 debug output is not blocked. */
    atk_ready = 0;
    atk_last_err = ATK_MS53L0M_ERROR;
    // 浼犳劅鍣ㄥ垵濮嬪寲閰嶇疆
    //JY61P_InitConfig();
	Moter_Init();
  //Adc3UartReport_Init();
  //Adc3UartReport_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  OLED_Init();
  // Doji_MovePos(001,1500,0010);
  // HAL_Delay(30);
  // Doji_MovePos(001,1500,0010);
  // HAL_Delay(30);
  //2200鈥斺?200
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
   // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    //rxBuffer2[0] = 0x01;
    Moter_A(00);
    Moter_B(500);
    Moter_C(500);
    Moter_D(500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 500);
    //Main_SetState(MAIN_STATE_STOP);
    while (1)
    {
      if ((g_vision_report_pending != 0U) &&
          (huart4.gState == HAL_UART_STATE_READY)) {
        int report_len = snprintf((char *)vision_report_buffer,
                                  sizeof(vision_report_buffer),
                                  "count=%u,class_id=%u,x=%u,y=%u,w=%u,h=%u,score=%.3f,label=%s\r\n",
                                  g_shijue_count, g_shijue_class_id,
                                  g_shijue_x, g_shijue_y, g_shijue_w,
                                  g_shijue_h, (double)g_shijue_score,
                                  g_shijue_label);
        if ((report_len > 0) && ((size_t)report_len < sizeof(vision_report_buffer))) {
          HAL_UART_Transmit_DMA(&huart4, vision_report_buffer, (uint16_t)report_len);
          g_vision_report_pending = 0U;
        }
      }

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
