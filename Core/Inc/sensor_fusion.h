#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "stm32f4xx_hal.h"
#include "vl53l0x.h"
#include "mpu6500.h"
#include <stdbool.h>

typedef struct {
    bool wall_left;     // Co vach ben trai (mat trong chech 45 do trai)
    bool wall_front;    // Co vach phia truoc (2 mat ngoai nhin thang 0 do xa)
    bool wall_right;    // Co vach ben phai (mat trong chech 45 do phai)
    float wall_error;   // Do lech tam duong di chieu vuong goc (mm) cho PID
    float front_dist;   // Khoang cach trung binh toi tuong truoc (mm)
    float front_diff;   // Do chenh lech 2 mat truoc (front_l - front_r) de can vuong goc
    float front_l;      // Khoang cach mat thang trai doc lap (mm)
    float front_r;      // Khoang cach mat thang phai doc lap (mm)
    float diag_l;       // [MOI]: Khoang cach mat cheo trai tho (mm) de bat cu nhay vot
    float diag_r;       // [MOI]: Khoang cach mat cheo phai tho (mm) de bat cu nhay vot
    float fused_angle;  // Goc xoay hop nhat tu Gyro va Encoder (deg)
} SensorFusion_Data;

void SensorFusion_Init(void);
void SensorFusion_Update(VL53L0X_Data *vl_data, MPU6500_Data *mpu_data, float dist_l_mm, float dist_r_mm, float dt);
SensorFusion_Data* SensorFusion_GetData(void);
void SensorFusion_ResetEncoders(void);
void SensorFusion_ResetAngle(void);

#endif /* SENSOR_FUSION_H */