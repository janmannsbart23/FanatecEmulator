/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define IN_G10_Pin GPIO_PIN_9
#define IN_G10_GPIO_Port GPIOB
#define IN_SHIFTER_LEFT_Pin GPIO_PIN_5
#define IN_SHIFTER_LEFT_GPIO_Port GPIOA
#define IN_G6_Pin GPIO_PIN_0
#define IN_G6_GPIO_Port GPIOB
#define IN_G7_Pin GPIO_PIN_2
#define IN_G7_GPIO_Port GPIOB
#define IN_G9_Pin GPIO_PIN_8
#define IN_G9_GPIO_Port GPIOA
#define IN_G5_Pin GPIO_PIN_9
#define IN_G5_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_6
#define LD3_GPIO_Port GPIOC
#define IN_G4_Pin GPIO_PIN_10
#define IN_G4_GPIO_Port GPIOA
#define IN_G3_Pin GPIO_PIN_11
#define IN_G3_GPIO_Port GPIOA
#define IN_SHIFTER_RIGHT_Pin GPIO_PIN_12
#define IN_SHIFTER_RIGHT_GPIO_Port GPIOA
#define T_JTMS_Pin GPIO_PIN_13
#define T_JTMS_GPIO_Port GPIOA
#define T_JTCK_Pin GPIO_PIN_14
#define T_JTCK_GPIO_Port GPIOA
#define IN_G12_Pin GPIO_PIN_4
#define IN_G12_GPIO_Port GPIOB
#define IN_G11_Pin GPIO_PIN_5
#define IN_G11_GPIO_Port GPIOB
#define IN_G1_Pin GPIO_PIN_6
#define IN_G1_GPIO_Port GPIOB
#define IN_G2_Pin GPIO_PIN_7
#define IN_G2_GPIO_Port GPIOB
#define IN_G8_Pin GPIO_PIN_8
#define IN_G8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
