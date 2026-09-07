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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_STAT_Pin GPIO_PIN_13
#define LED_STAT_GPIO_Port GPIOC
#define ENCODER_l_A_Pin GPIO_PIN_0
#define ENCODER_l_A_GPIO_Port GPIOA
#define ENCODER_L_B_Pin GPIO_PIN_1
#define ENCODER_L_B_GPIO_Port GPIOA
#define SWITCH_PIN_Pin GPIO_PIN_2
#define SWITCH_PIN_GPIO_Port GPIOA
#define XSHUT_L_Pin GPIO_PIN_3
#define XSHUT_L_GPIO_Port GPIOA
#define XSHUT_FL_Pin GPIO_PIN_4
#define XSHUT_FL_GPIO_Port GPIOA
#define STBY_Pin GPIO_PIN_5
#define STBY_GPIO_Port GPIOA
#define ENCODER_R_A_Pin GPIO_PIN_6
#define ENCODER_R_A_GPIO_Port GPIOA
#define ENCODER_R_B_Pin GPIO_PIN_7
#define ENCODER_R_B_GPIO_Port GPIOA
#define XSHUT_FR_Pin GPIO_PIN_0
#define XSHUT_FR_GPIO_Port GPIOB
#define XSHUT_R_Pin GPIO_PIN_1
#define XSHUT_R_GPIO_Port GPIOB
#define I2C_IMU_SCL_Pin GPIO_PIN_10
#define I2C_IMU_SCL_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_12
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_13
#define AIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_14
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_15
#define BIN2_GPIO_Port GPIOB
#define PWM_A_Pin GPIO_PIN_8
#define PWM_A_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_9
#define PWM_B_GPIO_Port GPIOA
#define I2C_IMU_SDA_Pin GPIO_PIN_3
#define I2C_IMU_SDA_GPIO_Port GPIOB
#define I2C_LASER_SCL_Pin GPIO_PIN_6
#define I2C_LASER_SCL_GPIO_Port GPIOB
#define I2C_LASER_SDA_Pin GPIO_PIN_7
#define I2C_LASER_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
