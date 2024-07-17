/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define ValveCAS_3_3V_Pin GPIO_PIN_4
#define ValveCAS_3_3V_GPIO_Port GPIOA
#define BRAKE_LIGHT_SIGNAL_Pin GPIO_PIN_6
#define BRAKE_LIGHT_SIGNAL_GPIO_Port GPIOA
#define ADC_BP1_Pin GPIO_PIN_4
#define ADC_BP1_GPIO_Port GPIOC
#define ADC_BP2_Pin GPIO_PIN_5
#define ADC_BP2_GPIO_Port GPIOC
#define BrakeSensorFront_BP3_3_3V_Pin GPIO_PIN_0
#define BrakeSensorFront_BP3_3_3V_GPIO_Port GPIOB
#define BrakeSensorRear_BP4_3_3V_Pin GPIO_PIN_1
#define BrakeSensorRear_BP4_3_3V_GPIO_Port GPIOB
#define WD_is_ready_Pin GPIO_PIN_2
#define WD_is_ready_GPIO_Port GPIOB
#define EBSActuator_Pin GPIO_PIN_6
#define EBSActuator_GPIO_Port GPIOC
#define RedundantActuator_Pin GPIO_PIN_7
#define RedundantActuator_GPIO_Port GPIOC
#define AS_Close_SDC_Pin GPIO_PIN_8
#define AS_Close_SDC_GPIO_Port GPIOC
#define Watchdog_Pin GPIO_PIN_6
#define Watchdog_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
