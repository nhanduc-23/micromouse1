/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Private defines -----------------------------------------------------------*/
// LED trang thai va Nut nhan
#define LED_STAT_Pin            GPIO_PIN_13
#define LED_STAT_GPIO_Port      GPIOC
#define SWITCH_PIN_Pin          GPIO_PIN_2
#define SWITCH_PIN_GPIO_Port    GPIOA

// Encoder Banh Trai (TIM2 - PA0, PA1)
#define ENCODER_L_A_Pin         GPIO_PIN_0
#define ENCODER_L_A_GPIO_Port   GPIOA
#define ENCODER_l_A_Pin         ENCODER_L_A_Pin       // Hỗ trợ mã sinh tự động từ CubeMX
#define ENCODER_l_A_GPIO_Port   ENCODER_L_A_GPIO_Port // Hỗ trợ mã sinh tự động từ CubeMX
#define ENCODER_L_B_Pin         GPIO_PIN_1
#define ENCODER_L_B_GPIO_Port   GPIOA

// Encoder Banh Phai (TIM3 - PA6, PA7) -> Giữ nguyên như ảnh
#define ENCODER_R_A_Pin         GPIO_PIN_6
#define ENCODER_R_A_GPIO_Port   GPIOA
#define ENCODER_r_A_Pin         ENCODER_R_A_Pin       // Hỗ trợ mã sinh tự động từ CubeMX
#define ENCODER_r_A_GPIO_Port   ENCODER_R_A_GPIO_Port // Hỗ trợ mã sinh tự động từ CubeMX
#define ENCODER_R_B_Pin         GPIO_PIN_7
#define ENCODER_R_B_GPIO_Port   GPIOA

// Chan dieu khien Laser XSHUT (Goc CubeMX)
#define XSHUT_L_Pin             GPIO_PIN_3
#define XSHUT_L_GPIO_Port       GPIOA
#define XSHUT_FL_Pin            GPIO_PIN_4
#define XSHUT_FL_GPIO_Port      GPIOA
#define XSHUT_FR_Pin            GPIO_PIN_0
#define XSHUT_FR_GPIO_Port      GPIOB
#define XSHUT_R_Pin             GPIO_PIN_1
#define XSHUT_R_GPIO_Port       GPIOB

// Bi danh chuan hoa theo cau hinh hinh hoc thuc te (Phuong an A)
#define XSHUT_FRONT_L_Pin       XSHUT_L_Pin       // Trai Ngoai: Nhin thang 0 do
#define XSHUT_FRONT_L_GPIO_Port XSHUT_L_GPIO_Port
#define XSHUT_DIAG_L_Pin        XSHUT_FL_Pin      // Trai Trong: Chech 45 do te trai
#define XSHUT_DIAG_L_GPIO_Port  XSHUT_FL_GPIO_Port
#define XSHUT_DIAG_R_Pin        XSHUT_FR_Pin      // Phai Trong: Chech 45 do te phai
#define XSHUT_DIAG_R_GPIO_Port  XSHUT_FR_GPIO_Port
#define XSHUT_FRONT_R_Pin       XSHUT_R_Pin       // Phai Ngoai: Nhin thang 0 do
#define XSHUT_FRONT_R_GPIO_Port XSHUT_R_GPIO_Port

// Driver Dong co TB6612
#define STBY_Pin                GPIO_PIN_5
#define STBY_GPIO_Port          GPIOA
#define PWM_A_Pin               GPIO_PIN_8
#define PWM_A_GPIO_Port         GPIOA
#define PWM_B_Pin               GPIO_PIN_9
#define PWM_B_GPIO_Port         GPIOA
#define AIN1_Pin                GPIO_PIN_12
#define AIN1_GPIO_Port          GPIOB
#define AIN2_Pin                GPIO_PIN_13
#define AIN2_GPIO_Port          GPIOB
#define BIN1_Pin                GPIO_PIN_14
#define BIN1_GPIO_Port          GPIOB
#define BIN2_Pin                GPIO_PIN_15
#define BIN2_GPIO_Port          GPIOB

// Bus I2C1 (Laser ToF - PB6, PB7)
#define I2C_LASER_SCL_Pin       GPIO_PIN_6
#define I2C_LASER_SCL_GPIO_Port GPIOB
#define I2C_LASER_SDA_Pin       GPIO_PIN_7
#define I2C_LASER_SDA_GPIO_Port GPIOB

// Bus I2C2 (IMU MPU-6500 - PB10, PB3)
#define I2C_IMU_SCL_Pin         GPIO_PIN_10
#define I2C_IMU_SCL_GPIO_Port   GPIOB
#define I2C_IMU_SDA_Pin         GPIO_PIN_3
#define I2C_IMU_SDA_GPIO_Port   GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */