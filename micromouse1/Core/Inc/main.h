#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

void Error_Handler(void);

/* ==================== KHAI BAO PINOUT MICROMOUSE ==================== */

// 1. Den bao trang thai & Nut nhan dieu khien
#define LED_PIN             GPIO_PIN_13            // LED SMD tren mach Black Pill (PC13 - Active LOW)
#define LED_PORT            GPIOC

#define BTN_MODE_PIN        GPIO_PIN_2             // Nut bam chuyen che do / Start (PA2 - EXTI2)
#define BTN_MODE_PORT       GPIOA

// 2. Mach dieu khien dong co TB6612FNG
#define MOTOR_STBY_PIN      GPIO_PIN_5             // Chan Standby kich hoat TB6612 (PA5)
#define MOTOR_STBY_PORT     GPIOA

#define MOTOR_L_AIN1_PIN    GPIO_PIN_12            // Dinh huong dong co Trai AIN1 (PB12)
#define MOTOR_L_AIN1_PORT   GPIOB
#define MOTOR_L_AIN2_PIN    GPIO_PIN_13            // Dinh huong dong co Trai AIN2 (PB13)
#define MOTOR_L_AIN2_PORT   GPIOB

#define MOTOR_R_BIN1_PIN    GPIO_PIN_14            // Dinh huong dong co Phai BIN1 (PB14)
#define MOTOR_R_BIN1_PORT   GPIOB
#define MOTOR_R_BIN2_PIN    GPIO_PIN_15            // Dinh huong dong co Phai BIN2 (PB15)
#define MOTOR_R_BIN2_PORT   GPIOB

// 3. Chan dieu khien doi dia chi 4 cam bien Laser VL53L0X (XSHUT)
#define XSHUT_1_PIN         GPIO_PIN_3             // Chan XSHUT Laser Trai (PA3)
#define XSHUT_1_PORT        GPIOA

#define XSHUT_2_PIN         GPIO_PIN_4             // Chan XSHUT Laser Truoc-Trai (PA4)
#define XSHUT_2_PORT        GPIOA

#define XSHUT_3_PIN         GPIO_PIN_0             // Chan XSHUT Laser Truoc-Phai (PB0)
#define XSHUT_3_PORT        GPIOB

#define XSHUT_4_PIN         GPIO_PIN_1             // Chan XSHUT Laser Phai (PB1)
#define XSHUT_4_PORT        GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */