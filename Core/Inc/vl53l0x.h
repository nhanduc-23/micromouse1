#ifndef VL53L0X_H
#define VL53L0X_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define VL53L0X_OFFSET_MM      38   // Sai so thuc nghiem da hieu chuan (mm)
#define VL53L0X_OUT_OF_RANGE   8190 // Gia tri bao vuot tam do / khong co tuong

// Cau truc du lieu chua khoang cach 4 mat laser
// Su dung union: Ho tro ten goi hinh hoc truc quan va giu nguyen tuong thich ma cu
typedef struct {
    union {
        uint16_t front_l; // Trai Ngoai (PA3 - Nhin thang 0 do xa, moc 0,0)
        uint16_t l;
    };
    union {
        uint16_t diag_l;  // Trai Trong (PA4 - Chech 45 do te trai, toa do +1.7, +1.2)
        uint16_t fl;
    };
    union {
        uint16_t diag_r;  // Phai Trong (PB0 - Chech 45 do te phai, toa do -1.7, +1.2)
        uint16_t fr;
    };
    union {
        uint16_t front_r; // Phai Ngoai (PB1 - Nhin thang 0 do xa, moc 0,0)
        uint16_t r;
    };
} VL53L0X_Data;

// Ham khoi tao va doc du lieu
HAL_StatusTypeDef VL53L0X_Init_All(I2C_HandleTypeDef *hi2c);
void VL53L0X_Read_All(VL53L0X_Data *data);

#endif /* VL53L0X_H */