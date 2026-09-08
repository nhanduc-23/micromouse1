#ifndef VL53L0X_H
#define VL53L0X_H

#include "main.h"
#include "stm32f4xx_hal.h"

// Cau truc luu tru gia tri do khoang cach (mm) cua 4 cam bien
typedef struct {
    uint16_t l;  // Left (PA3)
    uint16_t fl; // Front-Left (PA4)
    uint16_t fr; // Front-Right (PB0)
    uint16_t r;  // Right (PB1)
} VL53L0X_Data;

// Khoi tao va dinh dia chi I2C moi cho ca 4 cam bien
uint8_t VL53L0X_Init_All(I2C_HandleTypeDef *hi2c);

// Doc gia tri khoang cach hien tai tu ca 4 cam bien (don vi mm)
void VL53L0X_Read_All(VL53L0X_Data *data);

#endif /* VL53L0X_H */





