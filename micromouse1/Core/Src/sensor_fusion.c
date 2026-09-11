/**
  ******************************************************************************
  * @file           : sensor_fusion.c
  * @brief          : Hop nhat du lieu cam bien (VL53L0X, MPU6500, Encoder)
  ******************************************************************************
  */

#include "sensor_fusion.h"
#include "config.h"

#define WALL_DETECTION_THRESHOLD 130.0f
#define TARGET_SIDE_DIST         55.0f

static SensorFusion_Data sf_data = {0};

// Bien luu quang duong chu ky truoc de tinh bien thien Delta
static float prev_dist_l = 0.0f;
static float prev_dist_r = 0.0f;

void SensorFusion_Init(void) {
    sf_data.wall_left = false;
    sf_data.wall_front = false;
    sf_data.wall_right = false;
    sf_data.wall_error = 0.0f;
    sf_data.fused_angle = 0.0f;
    
    SensorFusion_ResetEncoders();
}

//Xoa lich su quang duong cu ve 0 khi Encoder duoc Reset
void SensorFusion_ResetEncoders(void) {
    prev_dist_l = 0.0f;
    prev_dist_r = 0.0f;
}
void SensorFusion_Update(VL53L0X_Data *vl_data, MPU6500_Data *mpu_data, float dist_l_mm, float dist_r_mm, float dt) {
    // 1. Nhan dien co mat cua tuong (Wall Detection)
    sf_data.wall_left  = (vl_data->l < WALL_DETECTION_THRESHOLD);
    sf_data.wall_right = (vl_data->r < WALL_DETECTION_THRESHOLD);

    uint16_t front_dist = (vl_data->fl + vl_data->fr) / 2;
    sf_data.wall_front = (front_dist < WALL_DETECTION_THRESHOLD);

    // 2. Tinh toan sai so lech tam (Wall Error) cho PID bam tuong
    if (sf_data.wall_left && sf_data.wall_right) {
        sf_data.wall_error = (float)vl_data->l - (float)vl_data->r;
    } else if (sf_data.wall_left) {
        sf_data.wall_error = 2.0f * ((float)vl_data->l - TARGET_SIDE_DIST);
    } else if (sf_data.wall_right) {
        sf_data.wall_error = 2.0f * (TARGET_SIDE_DIST - (float)vl_data->r);
    } else {
        sf_data.wall_error = 0.0f;
    }

    //3. Hop nhat goc xoay theo Delta (Bo doan > 50.0f cu)
     float delta_l = dist_l_mm - prev_dist_l;
     float delta_r = dist_r_mm - prev_dist_r;

		 prev_dist_l = dist_l_mm;
     prev_dist_r = dist_r_mm;
    
		// Tinh goc xoay delta tu Encoder va Gyro trong chu ky hien tai
    float delta_angle_encoder = ((delta_r - delta_l) / WHEEL_TRACK_MM) * RAD_TO_DEG;
    float delta_angle_gyro = mpu_data->gyro_z * dt;

    // Hop nhat delta goc (95% Gyro + 5% Encoder) va tich luy vao fused_angle
    float delta_fused_angle = 0.95f * delta_angle_gyro + 0.05f * delta_angle_encoder;
    sf_data.fused_angle += delta_fused_angle;
}

SensorFusion_Data* SensorFusion_GetData(void) {
    return &sf_data;
}

void SensorFusion_ResetAngle(void) {
    sf_data.fused_angle = 0.0f;
    MPU6500_ResetYaw(); // Dat mpu_data.yaw_angle ve 0.0f
}