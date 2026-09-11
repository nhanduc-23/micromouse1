#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx_hal.h"

void Encoder_Init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right);
void Encoder_GetCounters(int32_t *cnt_left, int32_t *cnt_right);
void Encoder_Reset(void);

// Bo sung hàm doc quãng duong theo mm
void Encoder_GetDistanceMM(float *dist_left_mm, float *dist_right_mm);

#endif /* ENCODER_H */