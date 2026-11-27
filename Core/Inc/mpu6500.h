#ifndef MPU6500_H
#define MPU6500_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float gyro_z_raw;     // Toc do goc Gyro-Z chua hieu chinh (deg/s)
    float gyro_z;         // Toc do goc Gyro-Z da tru offset va loc vung chet (deg/s)
    float gz_offset;      // Gia tri troi (bias) ban dau khi robot dung yen (deg/s)
    float yaw_angle;      // Goc xoay tich luy (do)
    bool  is_connected;   // Co bao trang thai ket noi phan cung
} MPU6500_Data;

// Khoi tao MPU6500 qua bus I2C (tu dong quet dia chi 0x68 va 0x69, kiem tra WHO_AM_I)
HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c);

// Hieu chinh offset cho Gyro-Z khi robot dung yen ban dau (lay mau va tinh trung binh)
void MPU6500_Calibrate(void);

// Doc toc do goc Gyro-Z va tich phan goc xoay Yaw
void MPU6500_Read_GyroZ(float dt);

// Lay con tro du lieu MPU6500
MPU6500_Data* MPU6500_GetData(void);

// Dat lai goc xoay Yaw ve 0
void MPU6500_ResetYaw(void);

#endif /* MPU6500_H */