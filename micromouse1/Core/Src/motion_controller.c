#include "motion_controller.h"
#include "conflig.h"
#include "tb6612fng.h"   
#include "encoder.h"
#include <math.h>
#include "stm32f4xx_hal_def.h"

static PID_Controller pid_wall;
static PID_Controller pid_gyro;
static MotionState current_state = MOTION_STATE_IDLE;

static float target_distance = 0.0f;
static float target_angle = 0.0f;
static bool is_finished = true;

void Motion_Init(void) {
    // Kp, Ki, Kd, OutMin, OutMax cho PID bám tu?ng
    PID_Init(&pid_wall, 2.0f, 0.0f, 0.5f, -300.0f, 300.0f);
    
    // Kp, Ki, Kd, OutMin, OutMax cho PID quay góc
    PID_Init(&pid_gyro, 4.0f, 0.1f, 0.8f, -500.0f, 500.0f);
    
    Motion_Stop();
}

void Motion_MoveForward(float distance_mm) {
    target_distance = distance_mm;
    Encoder_Reset();
    PID_Reset(&pid_wall);
    current_state = MOTION_STATE_FORWARD;
    is_finished = false;
}

void Motion_Rotate(float angle_deg) {
    target_angle = angle_deg;
    PID_Reset(&pid_gyro);
    current_state = MOTION_STATE_ROTATE;
    is_finished = false;
}

void Motion_Stop(void) {
    TB6612_SetSpeed(0, 0);
    current_state = MOTION_STATE_IDLE;
    is_finished = true;
}

bool Motion_IsFinished(void) {
    return is_finished;
}

void Motion_Update(float dt) {
    if (current_state == MOTION_STATE_IDLE) return;

    float dist_l = 0.0f;
    float dist_r = 0.0f;
	  Encoder_GetDistanceMM(&dist_l, &dist_r);
    float current_dist = (dist_l + dist_r) / 2.0f;

    SensorFusion_Data *sf = SensorFusion_GetData();

    if (current_state == MOTION_STATE_FORWARD) {
        // Dung khi da di du khoang cach dat
        if (current_dist >= target_distance) {
            Motion_Stop();
            return;
        }

        // Tinh toan chinh huong: uu tien PID bam tuong
        float correction = 0.0f;
        if (sf->wall_left || sf->wall_right) {
            correction = PID_Compute(&pid_wall, 0.0f, sf->wall_error, dt);
        }

        int base_speed = 400; // Toc do co ban
        TB6612_SetSpeed(base_speed - (int)correction, base_speed + (int)correction);

    } else if (current_state == MOTION_STATE_ROTATE) {
        float angle_error = target_angle - sf->fused_angle;

        // Dung khi sai so goc nho hon 1 do
        if (fabsf(angle_error) < 1.0f) {
            Motion_Stop();
            return;
        }

        float turn_speed = PID_Compute(&pid_gyro, target_angle, sf->fused_angle, dt);
       TB6612_SetSpeed(-(int)turn_speed, (int)turn_speed);
    }
}