#include "pid.h"

// He so loc thong thap cho khau vi phan (0.25 giup lam min nhieu lay mau ToF 33-50ms)
#define PID_D_FILTER_ALPHA  0.25f

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    
    // Gioi han thanh phan I dong gop toi da 40% cong suat dau ra de tranh vot lo
    pid->i_term_max = out_max * 0.4f; 
    
    PID_Reset(pid);
}

void PID_Reset(PID_Controller *pid) {
    pid->prev_error   = 0.0f;
    pid->integral     = 0.0f;
    pid->d_filtered   = 0.0f;
    pid->is_first_run = true; // Bat co de chu ky dau tien khong bi giat dao ham
}

float PID_Compute(PID_Controller *pid, float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // 1. Tinh sai so hien tai
    float error = setpoint - measured;

    // 2. Thanh phan Ty le (P)
    float p_out = pid->kp * error;

    // 3. Thanh phan Tich phan (I) voi Anti-Windup
    float i_out = 0.0f;
    if (pid->ki > 0.0f) {
        pid->integral += error * dt;
        i_out = pid->ki * pid->integral;

        if (i_out > pid->i_term_max) {
            i_out = pid->i_term_max;
            pid->integral = i_out / pid->ki; // Kep chat tich phan
        } else if (i_out < -pid->i_term_max) {
            i_out = -pid->i_term_max;
            pid->integral = -pid->i_term_max / pid->ki;
        }
    } else {
        pid->integral = 0.0f;
    }

    // 4. Thanh phan Dao ham (D) co bo loc thong thap (Low-pass Filter)
    float d_out = 0.0f;
    if (pid->kd > 0.0f) {
        if (pid->is_first_run) {
            pid->prev_error   = error;
            pid->d_filtered   = 0.0f;
            pid->is_first_run = false;
        } else {
            float d_raw = (error - pid->prev_error) / dt;
            // Loc thong thap bac 1 triet tieu xung nhon do ToF tre nhip
            pid->d_filtered = (PID_D_FILTER_ALPHA * d_raw) + ((1.0f - PID_D_FILTER_ALPHA) * pid->d_filtered);
        }
        d_out = pid->kd * pid->d_filtered;
    }

    // 5. Tong hop va gioi han dau ra (Output Clamping)
    float output = p_out + i_out + d_out;

    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    pid->prev_error = error;
    return output;
}