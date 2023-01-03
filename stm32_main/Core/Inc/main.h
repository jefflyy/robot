/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Trig4_Pin GPIO_PIN_13
#define Trig4_GPIO_Port GPIOC
#define Echo1_Pin GPIO_PIN_0
#define Echo1_GPIO_Port GPIOA
#define Echo2_Pin GPIO_PIN_1
#define Echo2_GPIO_Port GPIOA
#define OpenMVTX_Pin GPIO_PIN_2
#define OpenMVTX_GPIO_Port GPIOA
#define OpenMVRX_Pin GPIO_PIN_3
#define OpenMVRX_GPIO_Port GPIOA
#define Trig2_Pin GPIO_PIN_5
#define Trig2_GPIO_Port GPIOA
#define Servo1_Pin GPIO_PIN_6
#define Servo1_GPIO_Port GPIOA
#define Servo2_Pin GPIO_PIN_7
#define Servo2_GPIO_Port GPIOA
#define Encoder2a_Pin GPIO_PIN_9
#define Encoder2a_GPIO_Port GPIOE
#define Encoder2b_Pin GPIO_PIN_11
#define Encoder2b_GPIO_Port GPIOE
#define Echo3_Pin GPIO_PIN_10
#define Echo3_GPIO_Port GPIOB
#define Echo4_Pin GPIO_PIN_11
#define Echo4_GPIO_Port GPIOB
#define Trig1_Pin GPIO_PIN_12
#define Trig1_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define IMUTX_Pin GPIO_PIN_8
#define IMUTX_GPIO_Port GPIOD
#define IMURX_Pin GPIO_PIN_9
#define IMURX_GPIO_Port GPIOD
#define Motor1_Pin GPIO_PIN_12
#define Motor1_GPIO_Port GPIOD
#define Motor2_Pin GPIO_PIN_13
#define Motor2_GPIO_Port GPIOD
#define Encoder1a_Pin GPIO_PIN_6
#define Encoder1a_GPIO_Port GPIOC
#define Encoder1b_Pin GPIO_PIN_7
#define Encoder1b_GPIO_Port GPIOC
#define BlueTX_Pin GPIO_PIN_9
#define BlueTX_GPIO_Port GPIOA
#define BlueRX_Pin GPIO_PIN_10
#define BlueRX_GPIO_Port GPIOA
#define Trig3_Pin GPIO_PIN_12
#define Trig3_GPIO_Port GPIOC
#define Motor1a_Pin GPIO_PIN_0
#define Motor1a_GPIO_Port GPIOD
#define Motor1b_Pin GPIO_PIN_1
#define Motor1b_GPIO_Port GPIOD
#define Motor2a_Pin GPIO_PIN_8
#define Motor2a_GPIO_Port GPIOB
#define Motor2b_Pin GPIO_PIN_9
#define Motor2b_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
