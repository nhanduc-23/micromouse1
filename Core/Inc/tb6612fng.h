#ifndef TB6612FNG_H
#define TB6612FNG_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

// Khoi tao PWM Timer1 va bat chan STBY
void TB6612_Init(TIM_HandleTypeDef *htim);

// Cai dat toc do cho 2 dong co
void TB6612_SetSpeed(int16_t speed_left, int16_t speed_right);

// Tha troi tu do ca 2 dong co (Coast)
void TB6612_Stop(void);

// Phanh cung dien tu ca 2 banh (Short Brake)
void TB6612_Brake(void);

// Phanh cung rieng tung banh lam truc xoay cho co che Swing Turn
void TB6612_BrakeLeft(void);
void TB6612_BrakeRight(void);

#endif /* TB6612FNG_H */