#include "tb6612fng.h"
#include "config.h"

// Thong so bu luc keo can bang cho BANH TRAI
#define MOTOR_L_BOOST_OFFSET   2 
#define MOTOR_L_SCALE          1.017f

static TIM_HandleTypeDef *pwmtim;

void TB6612_Init(TIM_HandleTypeDef *htim) {
    pwmtim = htim;
    HAL_TIM_PWM_Start(pwmtim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(pwmtim, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(MOTOR_STBY.port, MOTOR_STBY.pin, GPIO_PIN_SET);
    TB6612_Brake();
}

void TB6612_SetSpeed(int16_t speed_left, int16_t speed_right) {
    // 1. BANH TRAI (TIM1 CH2)
    if (speed_left == 0) {
        HAL_GPIO_WritePin(MOTOR_BIN1.port, MOTOR_BIN1.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_BIN2.port, MOTOR_BIN2.pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_2, 0);
    } else if (speed_left > 0) {
        int32_t pwm_val = (int32_t)(speed_left * MOTOR_L_SCALE) + MOTOR_L_BOOST_OFFSET;
        if (pwm_val > PWM_MAX_LIMIT) pwm_val = PWM_MAX_LIMIT;
        HAL_GPIO_WritePin(MOTOR_BIN1.port, MOTOR_BIN1.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_BIN2.port, MOTOR_BIN2.pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_2, (uint32_t)pwm_val);
    } else {
        int32_t pwm_val = (int32_t)((-speed_left) * MOTOR_L_SCALE) + MOTOR_L_BOOST_OFFSET;
        if (pwm_val > PWM_MAX_LIMIT) pwm_val = PWM_MAX_LIMIT;
        HAL_GPIO_WritePin(MOTOR_BIN1.port, MOTOR_BIN1.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_BIN2.port, MOTOR_BIN2.pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_2, (uint32_t)pwm_val);
    }

    // 2. BANH PHAI (TIM1 CH1)
    if (speed_right == 0) {
        HAL_GPIO_WritePin(MOTOR_AIN1.port, MOTOR_AIN1.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_AIN2.port, MOTOR_AIN2.pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_1, 0);
    } else if (speed_right > 0) {
        HAL_GPIO_WritePin(MOTOR_AIN1.port, MOTOR_AIN1.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_AIN2.port, MOTOR_AIN2.pin, GPIO_PIN_RESET);
        if (speed_right > PWM_MAX_LIMIT) speed_right = PWM_MAX_LIMIT;
        __HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_1, (uint32_t)speed_right);
    } else {
        HAL_GPIO_WritePin(MOTOR_AIN1.port, MOTOR_AIN1.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_AIN2.port, MOTOR_AIN2.pin, GPIO_PIN_SET);
        int32_t pwm_val = -speed_right;
        if (pwm_val > PWM_MAX_LIMIT) pwm_val = PWM_MAX_LIMIT;
        __HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_1, (uint32_t)pwm_val);
    }
}

void TB6612_Stop(void) {
    HAL_GPIO_WritePin(MOTOR_AIN1.port, MOTOR_AIN1.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN2.port, MOTOR_AIN2.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_BIN1.port, MOTOR_BIN1.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_BIN2.port, MOTOR_BIN2.pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_2, 0);
}

void TB6612_Brake(void) {
    HAL_GPIO_WritePin(MOTOR_AIN1.port, MOTOR_AIN1.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_AIN2.port, MOTOR_AIN2.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_BIN1.port, MOTOR_BIN1.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_BIN2.port, MOTOR_BIN2.pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_2, 0);
}

void TB6612_BrakeLeft(void) {
    HAL_GPIO_WritePin(MOTOR_BIN1.port, MOTOR_BIN1.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_BIN2.port, MOTOR_BIN2.pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_2, 0);
}

void TB6612_BrakeRight(void) {
    HAL_GPIO_WritePin(MOTOR_AIN1.port, MOTOR_AIN1.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_AIN2.port, MOTOR_AIN2.pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_1, 0);
}