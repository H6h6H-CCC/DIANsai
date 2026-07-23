/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOE
#define KAIGUAN2_Pin GPIO_PIN_4
#define KAIGUAN2_GPIO_Port GPIOE
#define KAIGUAN1_Pin GPIO_PIN_13
#define KAIGUAN1_GPIO_Port GPIOC
#define HUI5_Pin GPIO_PIN_2
#define HUI5_GPIO_Port GPIOF
#define KAIGUAN2F3_Pin GPIO_PIN_3
#define KAIGUAN2F3_GPIO_Port GPIOF
#define HUI6_Pin GPIO_PIN_4
#define HUI6_GPIO_Port GPIOF
#define KAIGUAN3_Pin GPIO_PIN_5
#define KAIGUAN3_GPIO_Port GPIOF
#define E1_Pin GPIO_PIN_0
#define E1_GPIO_Port GPIOC
#define E2_Pin GPIO_PIN_1
#define E2_GPIO_Port GPIOC
#define F1_Pin GPIO_PIN_2
#define F1_GPIO_Port GPIOC
#define F2_Pin GPIO_PIN_3
#define F2_GPIO_Port GPIOC
#define bo4_Pin GPIO_PIN_11
#define bo4_GPIO_Port GPIOF
#define HUI8_Pin GPIO_PIN_12
#define HUI8_GPIO_Port GPIOF
#define bo3_Pin GPIO_PIN_13
#define bo3_GPIO_Port GPIOF
#define HUI1_Pin GPIO_PIN_14
#define HUI1_GPIO_Port GPIOF
#define bo2_Pin GPIO_PIN_15
#define bo2_GPIO_Port GPIOF
#define HUI2_Pin GPIO_PIN_0
#define HUI2_GPIO_Port GPIOG
#define bo1_Pin GPIO_PIN_1
#define bo1_GPIO_Port GPIOG
#define HUI7_Pin GPIO_PIN_7
#define HUI7_GPIO_Port GPIOE
#define spics_Pin GPIO_PIN_8
#define spics_GPIO_Port GPIOE
#define D2_Pin GPIO_PIN_14
#define D2_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_15
#define D1_GPIO_Port GPIOB
#define BEE_Pin GPIO_PIN_11
#define BEE_GPIO_Port GPIOD
#define HUI3_Pin GPIO_PIN_3
#define HUI3_GPIO_Port GPIOG
#define HUI4_Pin GPIO_PIN_4
#define HUI4_GPIO_Port GPIOG
#define B2_Pin GPIO_PIN_5
#define B2_GPIO_Port GPIOG
#define B1_Pin GPIO_PIN_6
#define B1_GPIO_Port GPIOG
#define A2_Pin GPIO_PIN_7
#define A2_GPIO_Port GPIOG
#define A1_Pin GPIO_PIN_8
#define A1_GPIO_Port GPIOG
#define C2_Pin GPIO_PIN_8
#define C2_GPIO_Port GPIOC
#define C1_Pin GPIO_PIN_9
#define C1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
