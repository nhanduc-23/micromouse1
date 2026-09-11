#include "motion_controller.h"
#include "config.h"
#include "tb6612fng.h"   
#include "encoder.h"
#include <math.h>
#include "stm32f4xx_hal_def.h"

#define MIN_ROTATE_PWM      120.0f //Nguong PWM toi thieu de vuot ma sat tinh dong co N20
#define ROTATE_TIMEOUT_MS   1500 //Thoi gian cho toi da cho 1 lan xoay foc (1.5 giay)

static PID_Controller pid_wall;
static PID_Controller pid_gyro;
static MotionState current_state = MOTION_STATE_IDLE;

static float start_angle = 0.0f;
static float target_distance = 0.0f;
static float target_angle = 0.0f;
static bool is_finished = true;
static uint32_t rotate_start_tick = 0; // Bien ghi nhan thoi diem bat dau xoay

void Motion_Init(void) {
    // Kp, Ki, Kd, OutMin, OutMax cho PID bam tuong
    PID_Init(&pid_wall, 2.0f, 0.0f, 0.5f, -300.0f, 300.0f);
    
    // Kp, Ki, Kd, OutMin, OutMax cho PID quay goc
    PID_Init(&pid_gyro, 4.0f, 0.05f, 0.8f, -400.0f, 400.0f);
    
    Motion_Stop();
}

void Motion_MoveForward(float distance_mm) {
    target_distance = distance_mm;
    
		Encoder_Reset();
	  SensorFusion_ResetEncoders(); // Trang thai Sensor Funssion khi Reset Encoder
    
		PID_Reset(&pid_wall);
		PID_Reset(&pid_gyro);
		
	 // Khoa goc huong hien tai lam moc giu thang khi di qua o khong co tuong
    SensorFusion_Data *sf = SensorFusion_GetData();
    target_angle = sf->fused_angle; 

    current_state = MOTION_STATE_FORWARD;
    is_finished = false;
}

void Motion_Rotate(float angle_deg) {
    SensorFusion_Data *sf = SensorFusion_GetData();
    
    // Luu lai goc tuyet doi can dat toi dua tren goc hien tai
    start_angle = sf->fused_angle;
    target_angle = start_angle + angle_deg;
    
    PID_Reset(&pid_gyro);
    current_state = MOTION_STATE_ROTATE;
    is_finished = false;
	  rotate_start_tick = HAL_GetTick(); // Ghi nhan tick thoi gian bat dau
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
        }else{
				//Dung PID Gyro de giu vung huong goc ban dau khi di qua o trong
				 correction = PID_Compute(&pid_gyro, target_angle, sf->fused_angle, dt);
				}

        // Tu dong giam toc khi gan toi dich (Ramp Down)
        float remaining_dist = target_distance - current_dist;
        int base_speed = 400;
        if (remaining_dist < 40.0f) {
            base_speed = 150 + (int)(250.0f * (remaining_dist / 40.0f));
        }

        int speed_left = base_speed + (int)correction;
        int speed_right = base_speed - (int)correction;

        TB6612_SetSpeed(speed_left, speed_right);

    } else if (current_state == MOTION_STATE_ROTATE) {
        float angle_error = target_angle - sf->fused_angle;

        // 1. Dung khi sai so goc nho hon 1.0 do
        if (fabsf(angle_error) < 1.0f) {
            Motion_Stop();
            return;
        }
				
				// 2. Dieu kien Timeout an toan : Thoat ket neu qua 1.5 s chua xoay xong
				if (HAL_GetTick() - rotate_start_tick > ROTATE_TIMEOUT_MS) {
            Motion_Stop();
            return;
        }

        
				// 3. PID tinh toan toc do quay dua tren Target Angle va Fused Angle
        float turn_speed = PID_Compute(&pid_gyro, target_angle, sf->fused_angle, dt);
        
				//4. Bu vung chet (Deadzone Compensation): Dam bao PWM luon >= MIN_ROTATE_PWM
				if (turn_speed > 0.0f && turn_speed < MIN_ROTATE_PWM) {
            turn_speed = MIN_ROTATE_PWM;
        } else if (turn_speed < 0.0f && turn_speed > -MIN_ROTATE_PWM) {
            turn_speed = -MIN_ROTATE_PWM;
        }
        // Dieu khien 2 banh quay nguoc chieu nhau de xoay tai cho
        TB6612_SetSpeed(-(int)turn_speed, (int)turn_speed);
    }
}