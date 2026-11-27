#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f4xx_hal.h"

// ==========================================
// 1. HANG SO TOAN HOC & DONG HOC ROBOT
// ==========================================
#define PI                      3.1415926535f
#define RAD_TO_DEG              (180.0f / PI)
#define DEG_TO_RAD              (PI / 180.0f)

#define WHEEL_DIAMETER_MM       34.0f   // Duong kinh banh xe (mm)
#define WHEEL_TRACK_MM          91.0f   // Khoang cach giua 2 banh xe (mm)
#define ENCODER_PPR             7.0f    // So xung tren 1 vong dia tu N20
#define GEAR_RATIO              30.0f   // Ti so truyen hop so dong co N20

#define COUNTS_PER_REV          825.0f
#define WHEEL_CIRCUMFERENCE_MM  (PI * WHEEL_DIAMETER_MM)                  // ~106.81 mm
#define MM_PER_COUNT            (WHEEL_CIRCUMFERENCE_MM / COUNTS_PER_REV) // ~0.1295 mm/xung

#define FULL_CELL               180.0f  // Kich thuoc 1 o me cung (mm)
#define PWM_MAX_LIMIT           1000    // Gioi han PWM toi da (ARR cua TIM1)

// Xung lui can chinh co hoc ty vach
#define DOCK_REVERSE_PWM        220.0f

// ==========================================
// 2. NGUONG CAM BIEN TOF CHUAN HOA TU SO LIEU DO THUC TE
// ==========================================
#define WALL_FRONT_THRESHOLD        135.0f  // <= 135mm la co vach truoc o hien tai
#define WALL_DIAG_THRESHOLD         168.0f  // < 152mm la vach that (do thuc te: 101-135mm)
#define WALL_OPENING_HIGH_THRESHOLD 188.0f  // >= 165mm la loi mo ben hong (do thuc te: 165-315mm)

#define TARGET_DIAG_DIST            110.0f  // Cu ly chuan giua tim hanh lang (do thuc te: 101-116mm)
#define MAX_WALL_ERROR_CLAMP        45.0f   // Kep sai so bu lai toi da (mm)

// Moc dung ToF mat thang: Truc banh xe o dung tam o do duoc 42-50mm
#define FRONT_TURN_TRIGGER_MAX_MM   48.0f   // Dung chinh xac tai tam o (48mm) roi moi xoay Pivot Turn
#define CRASH_DIST_LIMIT_MM         32.0f   // Phanh khan cap khi mui xe sat vach duoi 32mm

// THONG SO CO KHI PHUC VU CAN TIM 2 VACH (DUAL-WALL DOCKING)
#define D_AXLE_TO_REAR_MM           44.0f   // Khoang cach tu tam truc banh xe toi mep duoi xe (mm)
#define DOCK_RETURN_CENTER_MM       40.0f   // Bo cham 38mm + truot ~2mm = 40mm ve dung tam 84mm

// ==========================================
// 3. DINH NGHIA PHAN CUNG (PIN MAPPING)
// ==========================================
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} Pin;

static const Pin MOTOR_STBY      = { GPIOA, GPIO_PIN_5 };
static const Pin MOTOR_AIN1      = { GPIOB, GPIO_PIN_12 };
static const Pin MOTOR_AIN2      = { GPIOB, GPIO_PIN_13 };
static const Pin MOTOR_BIN1      = { GPIOB, GPIO_PIN_14 };
static const Pin MOTOR_BIN2      = { GPIOB, GPIO_PIN_15 };

static const Pin XSHUT_FRONT_L   = { GPIOA, GPIO_PIN_3 };
static const Pin XSHUT_DIAG_L    = { GPIOA, GPIO_PIN_4 };
static const Pin XSHUT_DIAG_R    = { GPIOB, GPIO_PIN_0 };
static const Pin XSHUT_FRONT_R   = { GPIOB, GPIO_PIN_1 };

static const Pin XSHUT_L         = { GPIOA, GPIO_PIN_3 };
static const Pin XSHUT_FL        = { GPIOA, GPIO_PIN_4 };
static const Pin XSHUT_FR        = { GPIOB, GPIO_PIN_0 };
static const Pin XSHUT_R         = { GPIOB, GPIO_PIN_1 };

static const Pin ENCODER_L_A     = { GPIOA, GPIO_PIN_0 };
static const Pin ENCODER_L_B     = { GPIOA, GPIO_PIN_1 };
static const Pin ENCODER_R_A     = { GPIOA, GPIO_PIN_6 };
static const Pin ENCODER_R_B     = { GPIOA, GPIO_PIN_7 };

static const Pin PWM_LEFT        = { GPIOA, GPIO_PIN_9 };
static const Pin PWM_RIGHT       = { GPIOA, GPIO_PIN_8 };

static const Pin LED_BOARD       = { GPIOC, GPIO_PIN_13 };
static const Pin BTN_MODE        = { GPIOA, GPIO_PIN_2 };
static const Pin BT_TXD          = { GPIOA, GPIO_PIN_11 };
static const Pin BT_RXD          = { GPIOA, GPIO_PIN_12 };

#endif // CONFIG_H