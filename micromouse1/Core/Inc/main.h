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

/* USER CODE BEGIN Private defines */

/* --- LED Trang thai & Nut bam chuyen che do --- */
#define LED_PORT                 GPIOC
#define LED_PIN                  GPIO_PIN_13  // LED Black Pill (Active LOW)

#define BTN_MODE_PORT            GPIOA
#define BTN_MODE_PIN             GPIO_PIN_2   // Nut nhan ngat EXTI2

/* --- Mach cau H TB6612 & Dieu khien huong Dong co --- */
#define STBY_PORT                GPIOA
#define STBY_PIN                 GPIO_PIN_5   // Chan Standby TB6612 (HIGH = On)

#define MOTOR_LEFT_IN1_PORT      GPIOB
#define MOTOR_LEFT_IN1_PIN       GPIO_PIN_12  // Huong quay motor Trai
#define MOTOR_LEFT_IN2_PORT      GPIOB
#define MOTOR_LEFT_IN2_PIN       GPIO_PIN_13  // Huong quay motor Trai

#define MOTOR_RIGHT_IN1_PORT     GPIOB
#define MOTOR_RIGHT_IN1_PIN      GPIO_PIN_14  // Huong quay motor Phai
#define MOTOR_RIGHT_IN2_PORT     GPIOB
#define MOTOR_RIGHT_IN2_PIN      GPIO_PIN_15  // Huong quay motor Phai

/* --- Chan XSHUT Bat/Tat & Doi dia chi 4 cam bien Laser VL53L0X --- */
#define XSHUT_LEFT_PORT          GPIOA
#define XSHUT_LEFT_PIN           GPIO_PIN_3   // Laser Ngang Trai

#define XSHUT_DIAG_L_PORT        GPIOA
#define XSHUT_DIAG_L_PIN         GPIO_PIN_4   // Laser Cheo Trai

#define XSHUT_DIAG_R_PORT        GPIOB
#define XSHUT_DIAG_R_PIN         GPIO_PIN_0   // Laser Cheo Phai

#define XSHUT_RIGHT_PORT         GPIOB
#define XSHUT_RIGHT_PIN          GPIO_PIN_1   // Laser Ngang Phai

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */