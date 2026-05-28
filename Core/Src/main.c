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
#include "jy61p.h"
#include "atk_ms53l0m.h"
#include "moter.h"
#include "encoder.h"
#include "rolllll.h"
#include "shijue.h"
#include "state.h"
#include "doji.h"
#include "Emm_V5.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PACKET_TIMEOUT 100  
#define ROLL_VEL_DT_S 0.01f
#define ROLL_POS_DT_S 0.02f
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
volatile uint16_t g_rx2_size = 0U;
uint8_t shijue[10];
char displayBuffer[20];
uint16_t atk_id = 0;
uint16_t atk_distance = 0;
uint8_t atk_ready = 0;
uint8_t atk_last_err = ATK_MS53L0M_ERROR;
volatile uint8_t g_roll_flag_vel = 0;
volatile uint8_t g_roll_flag_pos = 0;
extern float g_roll_target_x;
extern float g_roll_target_y;
extern uint8_t g_state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Main_UpdateOledStatus(void)
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart4)
    {
      // for (int i = 0; i < Size; i++)
      // {
      //     ProcessReceivedData(rxBuffer4[i]);
      // }
      HAL_UART_Transmit_DMA(&huart4,rxBuffer4,Size);
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
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_TIM8_Init();
  MX_ADC3_Init();
  MX_TIM9_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
	
    // 鍚姩UART DMA鎺ユ敹
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
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    //Encoder3_Init();
    /* Keep laser disabled here first, so UART4 debug output is not blocked. */
    atk_ready = 0;
    atk_last_err = ATK_MS53L0M_ERROR;
    // 浼犳劅鍣ㄥ垵濮嬪寲閰嶇疆
    //JY61P_InitConfig();

    g_roll_target_x = 0.0f;
    g_roll_target_y = 0.0f;
    // RollCtrl_Init(
    //     0.2f, 0.00f, 0.0f, 80.0f, 40.0f,    /* 浣嶇疆鐜?*/
    //     2.5f, 0.8f, 4.0f, 2500.0f, 50.0f,  /* 閫熷害鐜?*/
    //     0.0f,  0.0f,   0.0f, 1000.0f, 200.0f  /* 瑙掑害鐜?鏈敤) */
    // );
    RollCtrl_Init(
        0.0f, 0.00f, 0.0f, 80.0f, 40.0f,    /* 浣嶇疆鐜?*/
        0.0f, 0.0f, 0.0f, 2500.0f, 50.0f,  /* 閫熷害鐜?*/
        0.0f,  0.0f,   0.0f, 1000.0f, 200.0f  /* 瑙掑害鐜?鏈敤) */
    );
    RollCtrl_SetCornerPidProfile(
        0.000f, 0.0f, 0.0f, 100.0f, 30.0f,   /* corner pos loop */
        0.0f,  0.0f,  0.0f, 2200.0f, 40.0f,  /* corner vel loop */
        0.0f,  0.0f,  0.0f, 1000.0f, 200.0f  /* corner angle loop (unused) */
    );
    RollCtrl_Reset();

	Moter_Init();
  Adc3UartReport_Init();
  Adc3UartReport_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  OLED_Init();
    //Moter_A(1000);
  // Doji_MovePos(001,1500,0010);
  // HAL_Delay(30);
  // Doji_MovePos(001,1500,0010);
  // HAL_Delay(30);
  int i=0;
  State_Init();
  //2200鈥斺€?200
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 5000);
  
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 10000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    //rxBuffer2[0] = 0x01;
    int j=0;
	  while (1)
	  {
      ///Adc3UartReport_Send();
      HAL_Delay(1000);
      HAL_Delay(1000);
      HAL_Delay(1000);
      HAL_Delay(1000);
      //State_RunCurrent();
      //Main_UpdateOledStatus();
      //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2500);HAL_Delay(1000);
      
      //   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
      //   HAL_Delay(1000);
      //   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);
      //   HAL_Delay(1000);
      //  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2500);
      //   HAL_Delay(1000);
     
    // //   Emm_V5_MMCL_Pos_Control(3, 1, 600, 200, 1600, true, true);
    // Emm_V5_MMCL_Pos_Control(4, 1, 600, 200, 1600, true, true);
    // Emm_V5_Multi_Motor_Cmd_UART5(0);
    // HAL_Delay(100);
    // Emm_V5_MMCL_Pos_Control(3, 1, 600, 200, 1700, true, true);
    // Emm_V5_MMCL_Pos_Control(4, 1, 600, 200, 1700, true, true);
    // Emm_V5_Multi_Motor_Cmd_UART5(0);HAL_Delay(100);
      // if (g_roll_flag_pos)
      // {
      //   g_roll_flag_pos = 0;
      //   RollCtrl_UpdatePos((float)g_shijue_x, (float)g_shijue_y, ROLL_POS_DT_S);
      // }

      // if (g_roll_flag_vel)
      // {
      //   g_roll_flag_vel = 0;
      //   RollCtrl_UpdateVel(g_shijue_vx, g_shijue_vy, ROLL_VEL_DT_S);
      //   RollCtrl_UpdateAngleOutput_duoji(0.0f, 0.0f, ROLL_VEL_DT_S);
      // }
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
    static uint8_t tick_33ms = 0;
    static uint8_t tick_66ms = 0;

    tick_33ms++;
    tick_66ms++;
    if (tick_33ms >= 33)
    {
      tick_33ms = 0;
      g_roll_flag_vel = 1;
    }
    if (tick_66ms >= 66)
    {
      tick_66ms = 0;
      g_roll_flag_pos = 1;
    }

    count++;
    if (count >= 1000)
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
