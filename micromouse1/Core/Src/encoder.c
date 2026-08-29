#include "encoder.h"

extern TIM_HandleTypeDef htim2; // Timer 2 doc Encoder dong co Trai (PA0, PA1)
extern TIM_HandleTypeDef htim3; // Timer 3 doc Encoder dong co Phai (PA6, PA7)

// Khoi tao va kich hoat che do dem xung Encoder
void Encoder_Init(void) {
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    Encoder_Reset();
}

// Lay gia tri xung dem hien tai tu Encoder Trai
int16_t Encoder_Get_Left(void) {
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

// Lay gia tri xung dem hien tai tu Encoder Phai
int16_t Encoder_Get_Right(void) {
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

// Reset gia tri dem cua hai Encoder ve 0
void Encoder_Reset(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}