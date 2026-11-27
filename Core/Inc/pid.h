#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef struct {
    float kp;             // He so Khuech dai Ty le (Proportional)
    float ki;             // He so Tich phan (Integral)
    float kd;             // He so Dao ham (Derivative)
    
    float prev_error;     // Sai so tai chu ky truoc
    float integral;       // Gia tri tich luy tich phan
    float d_filtered;     // Thanh phan dao ham sau khi qua bo loc thong thap
    bool  is_first_run;   // Co bao chu ky dau tien de chong xung dao ham
    
    float out_min;        // Gioi han dau ra toi thieu
    float out_max;        // Gioi han dau ra toi da
    float i_term_max;     // Gioi han dong gop toi da cua thanh phan I (Anti-windup)
} PID_Controller;

// Khoi tao cau hinh bo dieu khien PID
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_min, float out_max);

// Cap nhat va tinh toan dau ra PID theo chu ky dt (giay)
float PID_Compute(PID_Controller *pid, float setpoint, float measured, float dt);

// Xoa trang thai tich phan va sai so (Reset PID)
void PID_Reset(PID_Controller *pid);

#endif /* PID_H */