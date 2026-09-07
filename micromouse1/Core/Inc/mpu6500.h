#ifndef MPU6500_H
#define MPU6500_H

#include "stm32f4xx_hal.h"

typedef struct {
    float gyro_z_raw;     // Toc do goc Gryo-Z chua hieu chinh (deg/s)
    float gyro_z;         // Toc do goc Gyro-Z da tru offset (deg/s)
    float gz_offset;      // Gia tri trôi (bias) ban dau khi robot dung yen
    float yaw_angle;      // Gia tri goc xoay tich luy (do)
} MPU6500_Data;

// Khoi tao MPU6500 qua I2C2
uint8_t MPU6500_Init(I2C_HandleTypeDef *hi2c);

// Hieu chinh offset cho Gyro-Z khi robot dung yen ban dau (lay 100 mau)
void MPU6500_Calibrate(void);

// Doc gia tri Gyro-Z va cap nhat goc xoay tich luy
void MPU6500_Read_GyroZ(float dt);

// Lay con tro du lieu MPU6500
MPU6500_Data* MPU6500_GetData(void);

// Dat lai goc xoay Yaw ve 0
void MPU6500_ResetYaw(void);

#endif /* MPU6500_H */