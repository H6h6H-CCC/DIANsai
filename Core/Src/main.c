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
#include "shijue.h"
#include "doji.h"
#include "Emm_V5.h"
#include "angle_sensor.h"

#include "balance_control.h"
#include "gray.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  MAIN_STATE_TRACK = 1,
  MAIN_STATE_BALANCE = 2,
  MAIN_STATE_WHEEL_LOCK = 3
} MainState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PACKET_TIMEOUT 100  
#define START_SWING_PWM 1000
#define START_SWING_B_COMP 50
#define START_SWING_MIN_MS 120
#define START_SWING_MAX_MS 150
#define START_SWING_STEP_MS 13
#define START_CATCH_ANGLE 5.5f
#define START_CATCH_RATE_DPS 0.5f
#define START_ANGLE_WAIT_MS 500U
#define TRACK_BASE_PWM 350
#define TRACK_KP 12
#define TRACK_STOP_BLACK_COUNT 3U
#define TRACK_FIRST_STOP_MS 1000U
#define BALANCE_RUN_MS 3000U
#define BALANCE_RESTART_DELAY_MS 500U
#define BALANCE_RESTART_MAX_COUNT 3U
#define WHEEL_LOCK_KP 40
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
static volatile MainState_t g_main_state = MAIN_STATE_TRACK;
static uint32_t g_state_start_tick = 0U;
static uint32_t g_balance_out_tick = 0U;
static uint8_t g_balance_restart_count = 0U;
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

static int16_t Main_CompensateRightSwingPwm(int16_t pwm)
{
    if (pwm > START_SWING_B_COMP) {
        return pwm - START_SWING_B_COMP;
    }
    if (pwm < -START_SWING_B_COMP) {
        return pwm + START_SWING_B_COMP;
    }
    return 0;
}

static float Main_AngleDiff(float target, float measure)
{
    float error = target - measure;

    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    return error;
}

static void Main_StartSwing(void)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_switch_tick;
    uint32_t last_angle_tick;
    int16_t switch_interval = START_SWING_MIN_MS;
    int16_t interval_step = START_SWING_STEP_MS;
    float angle;
    float last_angle;
    float target_angle;
    float error;
    float angle_delta;
    float angle_rate;
    uint32_t now_tick;
    uint32_t dt_ms;
    int16_t swing_pwm = START_SWING_PWM;

    Balance_Enable(0U);

    while ((AngleSensor_IsReady() == 0U) && ((HAL_GetTick() - start_tick) < START_ANGLE_WAIT_MS)) {
        HAL_Delay(1);
    }

    if (AngleSensor_IsReady() == 0U) {
        Balance_Enable(1U);
        return;
    }

    target_angle = Balance_GetTargetAngle();
    last_angle = AngleSensor_GetAngle();
    last_angle_tick = HAL_GetTick();
    // swing_pwm = (angle > target_angle) ? START_SWING_PWM : -START_SWING_PWM;
    last_switch_tick = HAL_GetTick();

    while (1) {
        now_tick = HAL_GetTick();
        angle = AngleSensor_GetAngle();
        error = Main_AngleDiff(target_angle, angle);
        dt_ms = now_tick - last_angle_tick;
        if (dt_ms == 0U) {
            angle_rate = 999.0f;
        } else {
            angle_delta = Main_AngleDiff(angle, last_angle);
            angle_rate = angle_delta * 1000.0f / (float)dt_ms;
            if (angle_rate < 0.0f) {
                angle_rate = -angle_rate;
            }
        }
        last_angle = angle;
        last_angle_tick = now_tick;

        if ((error < START_CATCH_ANGLE) &&
            (error > -START_CATCH_ANGLE) &&
            (angle_rate < START_CATCH_RATE_DPS)) {
            break;
        }

        if ((HAL_GetTick() - last_switch_tick) >= (uint32_t)switch_interval) {
            last_switch_tick = HAL_GetTick();
            swing_pwm = -swing_pwm;
            switch_interval += interval_step;
            if (switch_interval >= START_SWING_MAX_MS) {
                switch_interval = START_SWING_MAX_MS;
                interval_step = -START_SWING_STEP_MS;
            } else if (switch_interval <= START_SWING_MIN_MS) {
                switch_interval = START_SWING_MIN_MS;
                interval_step = START_SWING_STEP_MS;
            }
        }

        Moter_A(swing_pwm);
        Moter_B(Main_CompensateRightSwingPwm(swing_pwm));
        HAL_Delay(5);
    }

    Balance_Enable(1U);
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
    g_state_start_tick = HAL_GetTick();
    if (state != MAIN_STATE_BALANCE) {
        g_balance_out_tick = 0U;
    }
    if (state == MAIN_STATE_BALANCE) {
        Balance_Enable(1U);
    } else {
        Balance_Enable(0U);
    }
}

static void Main_BalanceRestartWatch(void)
{
    float angle;

    if (g_main_state != MAIN_STATE_BALANCE) {
        return;
    }

    if (AngleSensor_IsReady() == 0U) {
        g_balance_out_tick = 0U;
        return;
    }

    angle = AngleSensor_GetAngle();
    if ((angle >= Balance_GetMinAngle()) && (angle <= Balance_GetMaxAngle())) {
        g_balance_out_tick = 0U;
        return;
    }

    if (g_balance_out_tick == 0U) {
        g_balance_out_tick = HAL_GetTick();
        return;
    }

    if ((HAL_GetTick() - g_balance_out_tick) < BALANCE_RESTART_DELAY_MS) {
        return;
    }

    Moter_A(0);
    Moter_B(0);
    Balance_Enable(0U);
    g_balance_out_tick = 0U;

    if (g_balance_restart_count >= BALANCE_RESTART_MAX_COUNT) {
        return;
    }

    g_balance_restart_count++;
    Main_StartSwing();
    Main_SetState(MAIN_STATE_BALANCE);
}

static void Main_WheelLockUpdate5ms(void)
{
    int16_t left_speed = Encoder4_GetLastDelta();
    int16_t right_speed = Encoder3_GetLastDelta();

    Moter_A((int16_t)(-left_speed * WHEEL_LOCK_KP));
    Moter_B((int16_t)(-right_speed * WHEEL_LOCK_KP));
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
            Main_SetState(MAIN_STATE_BALANCE);
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
  AngleSensor_Init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    Encoder3_Init();
    Encoder4_Init();
    /* Keep laser disabled here first, so UART4 debug output is not blocked. */
    atk_ready = 0;
    atk_last_err = ATK_MS53L0M_ERROR;
    // 浼犳劅鍣ㄥ垵濮嬪寲閰嶇疆
    //JY61P_InitConfig();
	Moter_Init();
  Balance_Init();
  Balance_Enable(0U);
  Main_StartSwing();
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
    Moter_A(0);
    Moter_B(0);
    Main_SetState(MAIN_STATE_BALANCE);
    while (1)
	  {
      if (g_main_state == MAIN_STATE_TRACK)
      {
        Main_TrackRun();
      }
      else if (g_main_state == MAIN_STATE_BALANCE)
      {
        Main_BalanceRestartWatch();
      }
      // else if ((g_main_state == MAIN_STATE_BALANCE) &&
      //          ((HAL_GetTick() - g_state_start_tick) >= BALANCE_RUN_MS))
      // {
      //   Main_SetState(MAIN_STATE_WHEEL_LOCK);
      // }

      HAL_Delay(10);
      //AngleSensor_ReportUart4();
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
    static uint8_t balance_tick = 0U;

    count++;
    balance_tick++;

    if (balance_tick >= 5U)
    {
      balance_tick = 0U;
      Encoder3_Update10ms();
      Encoder4_Update10ms();
      if (g_main_state == MAIN_STATE_BALANCE)
      {
        Balance_Update10ms();
      }
      else if (g_main_state == MAIN_STATE_WHEEL_LOCK)
      {
        Main_WheelLockUpdate5ms();
      }
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
