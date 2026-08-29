#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

//khai bao cac ham dieu khien dong co
void Motor_Init(void);
void Motor_SetSpeed_Left(int speed);
void Motor_SetSpeed_right(int speed);
void Motor_SetSpeed(int speed_L, int speed_R);
void Motor_Stop(void);

#endif /* __Motor_H */


