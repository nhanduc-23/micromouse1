#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

// Khai bao cac ham doc va quan ly Encoder
void Encoder_Init(void);
int16_t Encoder_Get_Left(void);
int16_t Encoder_Get_Right(void);
void Encoder_Reset(void);

#endif /* __ENCODER_H */