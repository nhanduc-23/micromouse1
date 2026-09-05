#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx_hal.h"

//Khoi tao Mode Encoder cho TIM2 (trai) va TIM3(Phai)
void Encoder_Init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right);

//Doc so xung tich luy hien tai cua hai banh 
void Encoder_GetCounters(int16_t *cnt_left, int16_t *cnt_right);

//Reset thanh ghi diem xing ve 0
void Encoder_Reset(void);
#endif