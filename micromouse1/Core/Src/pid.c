#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    
    // Dat gioi han Tich phan toi da bang 50% gioi han dau ra toi da
    pid->integral_max = out_max * 0.5f; 
    
    PID_Reset(pid);
}

void PID_Reset(PID_Controller *pid) {
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}

float PID_Compute(PID_Controller *pid, float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // 1. Tinh sai so hien tai (Error)
    float error = setpoint - measured;

    // 2. Thanh phan Ty le (Proportional term)
    float p_out = pid->kp * error;

    // 3. Thanh phan Tich phan (Integral term) voi chong bao hoa Anti-Windup
    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }
    float i_out = pid->ki * pid->integral;

    // 4. Thanh phan Dao ham (Derivative term)
    float derivative = (error - pid->prev_error) / dt;
    float d_out = pid->kd * derivative;

    // 5. Tong hop dau ra va Khong khong che gioi han (Output Clamping)
    float output = p_out + i_out + d_out;

    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    // Luu sai so cho chu ky sau
    pid->prev_error = error;

    return output;
}