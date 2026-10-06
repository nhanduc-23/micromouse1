#include "tb6612fng.h"
#include "config.h"

static TIM_HandleTypeDef *pwmtim;

// Khoi tao driver dong co va bat cac kenh PWM
void TB6612_Init(TIM_HandleTypeDef *htim) {
    pwmtim = htim;
    
    // Bat PWM cho Kenh 1 (banh trai) va Kenh 2 (banh phai)
    HAL_TIM_PWM_Start(pwmtim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(pwmtim, TIM_CHANNEL_2);
    
    // Kich hoat chan STBY (neu co dung chan STBY trong config.h)
    HAL_GPIO_WritePin(MOTOR_STBY.port, MOTOR_STBY.pin, GPIO_PIN_SET);
    
    TB6612_Stop();
}

// Cai dat toc do va chieu quay 2 banh (-PWM_MAX_LIMIT den +PWM_MAX_LIMIT)
void TB6612_SetSpeed(int16_t speed_left, int16_t speed_right)
{
    // =========================
    // MOTOR LEFT
    // =========================
    if (speed_left == 0)
    {
        // COAST / th? motor
        HAL_GPIO_WritePin(MOTOR_AIN1.port,
                          MOTOR_AIN1.pin,
                          GPIO_PIN_RESET);

        HAL_GPIO_WritePin(MOTOR_AIN2.port,
                          MOTOR_AIN2.pin,
                          GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(pwmtim,
                              TIM_CHANNEL_1,
                              0);
    }
    else
    {
        if (speed_left > 0)
        {
            HAL_GPIO_WritePin(MOTOR_AIN1.port,
                              MOTOR_AIN1.pin,
                              GPIO_PIN_SET);

            HAL_GPIO_WritePin(MOTOR_AIN2.port,
                              MOTOR_AIN2.pin,
                              GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(MOTOR_AIN1.port,
                              MOTOR_AIN1.pin,
                              GPIO_PIN_RESET);

            HAL_GPIO_WritePin(MOTOR_AIN2.port,
                              MOTOR_AIN2.pin,
                              GPIO_PIN_SET);

            speed_left = -speed_left;
        }

        if (speed_left > PWM_MAX_LIMIT)
            speed_left = PWM_MAX_LIMIT;

        __HAL_TIM_SET_COMPARE(pwmtim,
                              TIM_CHANNEL_1,
                              (uint16_t)speed_left);
    }


    // =========================
    // MOTOR RIGHT
    // =========================
    if (speed_right == 0)
    {
        // COAST / th? motor
        HAL_GPIO_WritePin(MOTOR_BIN1.port,
                          MOTOR_BIN1.pin,
                          GPIO_PIN_RESET);

        HAL_GPIO_WritePin(MOTOR_BIN2.port,
                          MOTOR_BIN2.pin,
                          GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(pwmtim,
                              TIM_CHANNEL_2,
                              0);
    }
    else
    {
        if (speed_right > 0)
        {
            HAL_GPIO_WritePin(MOTOR_BIN1.port,
                              MOTOR_BIN1.pin,
                              GPIO_PIN_SET);

            HAL_GPIO_WritePin(MOTOR_BIN2.port,
                              MOTOR_BIN2.pin,
                              GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(MOTOR_BIN1.port,
                              MOTOR_BIN1.pin,
                              GPIO_PIN_RESET);

            HAL_GPIO_WritePin(MOTOR_BIN2.port,
                              MOTOR_BIN2.pin,
                              GPIO_PIN_SET);

            speed_right = -speed_right;
        }

        if (speed_right > PWM_MAX_LIMIT)
            speed_right = PWM_MAX_LIMIT;

        __HAL_TIM_SET_COMPARE(pwmtim,
                              TIM_CHANNEL_2,
                              (uint16_t)speed_right);
    }
}

// Dung dong co hoan toan
void TB6612_Stop(void) {
    TB6612_SetSpeed(0, 0);
}