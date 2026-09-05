#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f4xx_hal.h"

// ==========================================
// 1. HANG SO TOAN HOC & DONG HOC
// ==========================================
#define PI                      3.1415926535f
#define RAD_TO_DEG              (180.0f / PI)
#define DEG_TO_RAD              (PI / 180.0f)

#define WHEEL_DIAMETER_MM       32.0f   // Duong kinh banh xe (mm)
#define WHEEL_TRACK_MM          75.0f   // Khoang cach giua 2 banh (mm)
#define ENCODER_PPR             12.0f   // So xung dia ma dem xung (PPR)
#define GEAR_RATIO              30.0f   // Ti so truyen dong co N20

// Tong so xung dem duoc khi banh xe quay 1 vong (Mode X4 Hardware Encoder)
#define COUNTS_PER_REV          (ENCODER_PPR * 4.0f * GEAR_RATIO)
#define WHEEL_CIRCUMFERENCE_MM  (PI * WHEEL_DIAMETER_MM)
#define MM_PER_COUNT            (WHEEL_CIRCUMFERENCE_MM / COUNTS_PER_REV)

#define FULL_CELL               180.0f  // Kich thuoc 1 o me cung (mm)
#define PWM_MAX_LIMIT           1000    // Gioi han PWM toi da (ARR)

// ==========================================
// 2. CAU TRUC DU LIEU (STRUCTS)
// ==========================================
typedef struct {
    int speed;          // mm/s    - Toc do tien co dinh khi re
    int entry_offset;   // mm      - Khoang cach tu tam den diem bat dau re
    int exit_offset;    // mm      - Khoang cach tu diem ket thuc re den tam
    float angle;        // deg     - Goc xoay
    float omega;        // deg/s   - Van toc goc
    float alpha;        // deg/s^2 - Gia toc goc
    int trigger;        //         - Gia tri cam bien truoc tai thoi diem re
} TurnParameters;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} Pin;

// ==========================================
// 3. DINH NGHIA PHAN CUNG (PIN MAPPING)
// ==========================================
// Driver Dong co (TB6612)
static const Pin MOTOR_STBY      = { GPIOC, GPIO_PIN_14 }; // PC14
static const Pin MOTOR_AIN1      = { GPIOB, GPIO_PIN_12 }; // PB12
static const Pin MOTOR_AIN2      = { GPIOB, GPIO_PIN_13 }; // PB13
static const Pin MOTOR_BIN1      = { GPIOB, GPIO_PIN_14 }; // PB14
static const Pin MOTOR_BIN2      = { GPIOB, GPIO_PIN_15 }; // PB15

// Cam bien VL53L0X (Chan XSHUT)
static const Pin XSHUT_FL        = { GPIOB, GPIO_PIN_0 };  // PB0
static const Pin XSHUT_FR        = { GPIOB, GPIO_PIN_1 };  // PB1
static const Pin XSHUT_L         = { GPIOB, GPIO_PIN_2 };  // PB2
static const Pin XSHUT_R         = { GPIOB, GPIO_PIN_10 }; // PB10

// Dem xung Encoder (Hardware TIM2 & TIM3)
static const Pin ENCODER_L_A     = { GPIOA, GPIO_PIN_0 }; // TIM2_CH1 - PA0
static const Pin ENCODER_L_B     = { GPIOA, GPIO_PIN_1 }; // TIM2_CH2 - PA1
static const Pin ENCODER_R_A     = { GPIOA, GPIO_PIN_6 }; // TIM3_CH1 - PA6
static const Pin ENCODER_R_B     = { GPIOA, GPIO_PIN_7 }; // TIM3_CH2 - PA7

// Dieu khien PWM (TIM1)
static const Pin PWM_LEFT        = { GPIOA, GPIO_PIN_8 }; // TIM1_CH1 - PA8
static const Pin PWM_RIGHT       = { GPIOA, GPIO_PIN_9 }; // TIM1_CH2 - PA9

// Nut nhan / Nut gat cau hinh
static const Pin SWITCH_1        = { GPIOA, GPIO_PIN_11 }; // PA11
static const Pin SWITCH_2        = { GPIOA, GPIO_PIN_12 }; // PA12

#endif // CONFIG_H