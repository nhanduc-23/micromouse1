#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"

// Cau truc quan ly thong so va trang thai Bo dieu khien PID
typedef struct {
    float kp;             // He so Khuech dai Tyle (Proportional)
    float ki;             // He so Tich phan (Integral)
    float kd;             // He so Dao ham (Derivative)
    
    float prev_error;     // Sai so tai chu ky truoc
    float integral;       // Gia tri tich luy tich phan
    
    float out_min;        // Gioi han dau ra toi thieu (Vidu: -1000)
    float out_max;        // Gioi han dau ra toi da (Vidu: 1000)
    float integral_max;   // Gioi han chong bao hoa tich phan (Anti-windup)
} PID_Controller;

// Khoi tao cau hinh bo dieu khien PID
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_min, float out_max);

// Cap nhat va tinh toan dau ra PID theo chu ky dt (giay)
float PID_Compute(PID_Controller *pid, float setpoint, float measured, float dt);

// Xoa trang thai tich phan va sai so truoc do (Reset PID)
void PID_Reset(PID_Controller *pid);

#endif /* PID_H */