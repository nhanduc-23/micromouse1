#ifndef TB6612FNG_H
#define TB6612FNG_H

#include"stm32f4xx_hal.h"

// Nhan Duc //

//Khoi tao PWM Timer1 va kich hoat chan Standby PA5
void TB6612_Init(TIM_HandleTypeDef*htim);

//Cai dat toc do (-1000 den 1000) cho 2 banh xe
void TB6612_SetSpeed(int16_t speed_left, int16_t speed_right);

// Dung khan cap ca 2 dong co
void TB6612_Stop(void);



#endif /*TB6612FNG_H */