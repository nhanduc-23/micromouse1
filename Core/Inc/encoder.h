#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

// Khoi tao bo dem xung Encoder cho 2 Timer
void Encoder_Init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right);

// Lay tong so xung tich luy tu luc bat dau chay
void Encoder_GetCounters(int32_t *cnt_left, int32_t *cnt_right);

// Lay so xung chenh lech (delta) trong chu ky lay mau vua qua (phuc vu PID toc do)
void Encoder_GetDelta(int32_t *delta_left, int32_t *delta_right);

// Lay quang duong chenh lech tinh theo mm trong chu ky vua qua
void Encoder_GetDeltaMM(float *d_left_mm, float *d_right_mm);

// Lay tong quang duong tich luy theo mm
void Encoder_GetDistanceMM(float *dist_left_mm, float *dist_right_mm);

// Reset lai toan bo bo dem xung va quang duong ve 0
void Encoder_Reset(void);

#endif /* ENCODER_H */