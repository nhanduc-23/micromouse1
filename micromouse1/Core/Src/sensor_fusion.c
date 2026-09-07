#include "sensor_fusion.h"
#include "conflig.h"

// Nguong khoang cach nhan dien co tuong (mm)
#define WALL_DETECTION_THRESHOLD 130.0f 

// Khoang cach ly tuong tu tam Robot toi tuong ben (mm)
#define TARGET_SIDE_DIST         55.0f  

static SensorFusion_Data sf_data = {0};

void SensorFusion_Init(void) {
    sf_data.wall_left = false;
    sf_data.wall_front = false;
    sf_data.wall_right = false;
    sf_data.wall_error = 0.0f;
    sf_data.fused_angle = 0.0f;
}

void SensorFusion_Update(VL53L0X_Data *vl_data, MPU6500_Data *mpu_data, float dist_l_mm, float dist_r_mm, float dt) {
    // 1. Nhan dien co mat cua tuong (Wall Detection)
    sf_data.wall_left  = (vl_data->l < WALL_DETECTION_THRESHOLD);
    sf_data.wall_right = (vl_data->r < WALL_DETECTION_THRESHOLD);
    
    // Tuong truoc tinh trung binh 2 cam bien truoc-trai va truoc-phai
    uint16_t front_dist = (vl_data->fl + vl_data->fr) / 2;
    sf_data.wall_front = (front_dist < WALL_DETECTION_THRESHOLD);

    // 2. Tinh toan sai so lech tam (Wall Error) cho thuat toan PID bam tuong
    if (sf_data.wall_left && sf_data.wall_right) {
        // Co ca 2 tuong: Error = Khoang cach trai - Khoang cach phai
        sf_data.wall_error = (float)vl_data->l - (float)vl_data->r;
    } else if (sf_data.wall_left) {
        // Chi co tuong trai: Error = 2 * (Khoang cach trai - Khoang cach chuan)
        sf_data.wall_error = 2.0f * ((float)vl_data->l - TARGET_SIDE_DIST);
    } else if (sf_data.wall_right) {
        // Chi co tuong phai: Error = 2 * (Khoang cach chuan - Khoang cach phai)
        sf_data.wall_error = 2.0f * (TARGET_SIDE_DIST - (float)vl_data->r);
    } else {
        // Khong co tuong 2 ben: Khong can thiap lech tuong
        sf_data.wall_error = 0.0f;
    }

    // 3. Hop nhat goc xoay (Complementary Filter giua Gyro Z va Encoder)
    // Goc uoc tinh tu Encoder: (Dist_R - Dist_L) / Wheel_Track * (180 / PI)
    float encoder_angle = ((dist_r_mm - dist_l_mm) / WHEEL_TRACK_MM) * RAD_TO_DEG;
    
    // Bo loc bu: 95% Gyro-Z + 5% Encoder de khu trôi goc
    sf_data.fused_angle = 0.95f * (sf_data.fused_angle + mpu_data->gyro_z * dt) + 0.05f * encoder_angle;
}

SensorFusion_Data* SensorFusion_GetData(void) {
    return &sf_data;
}