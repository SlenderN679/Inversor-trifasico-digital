/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define VIn__Pin GPIO_PIN_7
#define VIn__GPIO_Port GPIOF
#define IIn__Pin GPIO_PIN_9
#define IIn__GPIO_Port GPIOF
#define VDC_Pin GPIO_PIN_0
#define VDC_GPIO_Port GPIOC
#define Temp_Pin GPIO_PIN_3
#define Temp_GPIO_Port GPIOC
#define IA_Pin GPIO_PIN_0
#define IA_GPIO_Port GPIOA
#define IB_Pin GPIO_PIN_3
#define IB_GPIO_Port GPIOA
#define SINE_Pin GPIO_PIN_4
#define SINE_GPIO_Port GPIOA
#define CONST_Pin GPIO_PIN_5
#define CONST_GPIO_Port GPIOA
#define IN1Brk_Pin GPIO_PIN_6
#define IN1Brk_GPIO_Port GPIOA
#define IN2N_Pin GPIO_PIN_0
#define IN2N_GPIO_Port GPIOB
#define IN3N_Pin GPIO_PIN_1
#define IN3N_GPIO_Port GPIOB
#define IC_Pin GPIO_PIN_11
#define IC_GPIO_Port GPIOF
#define OUT1N_Pin GPIO_PIN_8
#define OUT1N_GPIO_Port GPIOE
#define OUT1_Pin GPIO_PIN_9
#define OUT1_GPIO_Port GPIOE
#define OUT2N_Pin GPIO_PIN_10
#define OUT2N_GPIO_Port GPIOE
#define OUT2_Pin GPIO_PIN_11
#define OUT2_GPIO_Port GPIOE
#define OUT3N_Pin GPIO_PIN_12
#define OUT3N_GPIO_Port GPIOE
#define OUT3_Pin GPIO_PIN_13
#define OUT3_GPIO_Port GPIOE
#define OUTBrk_Pin GPIO_PIN_15
#define OUTBrk_GPIO_Port GPIOE
#define SoftStart_Pin GPIO_PIN_11
#define SoftStart_GPIO_Port GPIOD
#define CHOPPER_Pin GPIO_PIN_12
#define CHOPPER_GPIO_Port GPIOD
#define ST7789_RST_Pin GPIO_PIN_13
#define ST7789_RST_GPIO_Port GPIOD
#define ST7789_DC_Pin GPIO_PIN_14
#define ST7789_DC_GPIO_Port GPIOD
#define ST7789_CS_Pin GPIO_PIN_15
#define ST7789_CS_GPIO_Port GPIOD
#define FAN_Pin GPIO_PIN_6
#define FAN_GPIO_Port GPIOC
#define IN2_Pin GPIO_PIN_7
#define IN2_GPIO_Port GPIOC
#define IN3_Pin GPIO_PIN_8
#define IN3_GPIO_Port GPIOC
#define Back_Pin GPIO_PIN_9
#define Back_GPIO_Port GPIOC
#define SW_Pin GPIO_PIN_0
#define SW_GPIO_Port GPIOD
#define SW_EXTI_IRQn EXTI0_IRQn
#define ST7789_CLK_Pin GPIO_PIN_3
#define ST7789_CLK_GPIO_Port GPIOB
#define ST7789_SPI_Pin GPIO_PIN_5
#define ST7789_SPI_GPIO_Port GPIOB
#define ENClk_Pin GPIO_PIN_6
#define ENClk_GPIO_Port GPIOB
#define ENDt_Pin GPIO_PIN_7
#define ENDt_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
