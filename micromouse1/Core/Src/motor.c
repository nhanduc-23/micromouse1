#include "motor.h"

extern TIM_HandleTypeDef htim1; // Timer 1 tao xung PWM cho 2 dong co

// Khoi tao va kich hoat mach cau H TB6612
void Motor_Init(void) {
    // Keo chan STBY (PA5) len HIGH de kich hoat IC TB6612
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
    
    // Bat xung PWM TIM1 Channel 1 (PA8) va Channel 2 (PA9)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

// Dieu khien dong co Trai (Left Motor)
void Motor_SetSpeed_Left(int speed) {
    if (speed > 0) {
        // Dong co quay tien
        HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_RESET);
    } else if (speed < 0) {
        // Dong co quay lui
        HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_SET);
        speed = -speed; // Chuyen sang so duong de gan cho PWM
    } else {
        // Dung dong co Trai
        HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_RESET);
    }
    
    // Gioi han gia tri PWM trong khoang 0 - 1000
    if (speed > 1000) speed = 1000;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
}

// Dieu khien dong co Phai (Right Motor)
void Motor_SetSpeed_Right(int speed) {
    if (speed > 0) {
        // Dong co quay tien
        HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_RESET);
    } else if (speed < 0) {
        // Dong co quay lui
        HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_SET);
        speed = -speed; // Chuyen sang so duong de gan cho PWM
    } else {
        // Dung dong co Phai
        HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_RESET);
    }
    
    // Gioi han gia tri PWM trong khoang 0 - 1000
    if (speed > 1000) speed = 1000;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
}

// Dieu khien dong thoi ca 2 dong co
void Motor_SetSpeed(int speed_L, int speed_R) {
    Motor_SetSpeed_Left(speed_L);
    Motor_SetSpeed_Right(speed_R);
}

// Dung xe ngay lap tuc
void Motor_Stop(void) {
    Motor_SetSpeed(0, 0);
}