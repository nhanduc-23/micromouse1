#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "stm32f4xx_hal.h"
#include "vl53l0x.h"
#include "mpu6500.h"
#include <stdbool.h>

typedef struct {
    bool wall_left;     // Co tuong ben trai hay khong
    bool wall_front;    // Co tuong phia truoc hay khong
    bool wall_right;    // Co tuong ben phai hay khong
    float wall_error;   // Do lech tam duong di (mm) phuc vu PID bam tuong
    float fused_angle;  // Goc xoay da duoc hop nhat tu Gyro va Encoder (deg)
} SensorFusion_Data;

// Khoi tao module Sensor Fusion
void SensorFusion_Init(void);

// Cap nhat va hop nhat du lieu tu tat ca cam bien theo chu ky dt (giay)
void SensorFusion_Update(VL53L0X_Data *vl_data, MPU6500_Data *mpu_data, float dist_l_mm, float dist_r_mm, float dt);

// Lay con tro du lieu hop nhat
SensorFusion_Data* SensorFusion_GetData(void);

// Ham dat lai bien luu vet ENcoder khi Reset Encoder
void SensorFusion_ResetEncoders(void);

// Dat lai goc hop nhat ve 0 khi bat dau luot chay moi
void SensorFusion_ResetAngle(void);

#endif /* SENSOR_FUSION_H */