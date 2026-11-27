/**
  ******************************************************************************
  * @file           : sensor_fusion.c
  * @brief          : Hop nhat cam bien ToF, IMU Gyro-Z va tinh sai so tim o
  *                   Toi uu chong nhieu FL/FR & chong troi lech hanh lang 16x16
  ******************************************************************************
  */

#include "sensor_fusion.h"
#include "config.h"
#include <math.h>

#define COS_45_DEG               0.7071f // Chieu khoang cach tia laser 45 do ve phuong vuong goc
#define ERROR_DEADBAND_MM        1.5f    // Vung chet loc nhieu ToF chong giat rung motor (mm)

// Khoa cap nhat tuong ben va ngat PID vach ngay khi tien sat vach truoc (<= 135mm)
#define FRONT_WALL_LOCK_PID_MM   135.0f  
#define SIDE_WALL_SCAN_MAX_MM    95.0f

// Gioi han keo xe VE PHIA tuong don o nga ba (chong hut goc)
#define MAX_CORNER_PULL_MM       10.0f

// Bo loc xac nhan tuong truoc (chong xung nhieu 1 khung hinh doc hanh lang dai)
#define FRONT_STEREO_MAX_DIFF_MM 55.0f   // Chenh lech toi da giua FL va FR khi gap vach that (mm)
#define FRONT_CONFIRM_FRAMES     2       // Can 2 chu ky lien tiep (20ms) de xac nhan vach truoc

static SensorFusion_Data sf_data = {0};

static float prev_dist_l = 0.0f;
static float prev_dist_r = 0.0f;

// Bo dem loc trung vi 3 mau (Median-3) cho 2 mat truoc FL & FR
static float fl_hist[3] = {999.0f, 999.0f, 999.0f};
static float fr_hist[3] = {999.0f, 999.0f, 999.0f};
static uint8_t front_hist_idx = 0;
static uint8_t front_wall_confirm_cnt = 0;

static float Median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

void SensorFusion_Init(void) {
    sf_data.wall_left   = false;
    sf_data.wall_front  = false;
    sf_data.wall_right  = false;
    sf_data.wall_error  = 0.0f;
    sf_data.front_dist  = 999.0f;
    sf_data.front_diff  = 0.0f;
    sf_data.front_l     = 999.0f;
    sf_data.front_r     = 999.0f;
    sf_data.diag_l      = 999.0f;
    sf_data.diag_r      = 999.0f;
    sf_data.fused_angle = 0.0f;

    for (int i = 0; i < 3; i++) {
        fl_hist[i] = 999.0f;
        fr_hist[i] = 999.0f;
    }
    front_hist_idx = 0;
    front_wall_confirm_cnt = 0;

    SensorFusion_ResetEncoders();
}

void SensorFusion_ResetEncoders(void) {
    prev_dist_l = 0.0f;
    prev_dist_r = 0.0f;
}

void SensorFusion_Update(VL53L0X_Data *vl_data, MPU6500_Data *mpu_data, float dist_l_mm, float dist_r_mm, float dt) {
    if (!vl_data) return;

    float raw_fl = (float)vl_data->front_l;
    float raw_fr = (float)vl_data->front_r;
    float diag_l = (float)vl_data->diag_l;
    float diag_r = (float)vl_data->diag_r;

    // Loc trung vi 3 mau loai bo xung nhieu 1 khung hinh (1mm, 26mm, 59mm, 8190...)
    fl_hist[front_hist_idx] = (raw_fl > 4.0f && raw_fl < 2000.0f) ? raw_fl : 999.0f;
    fr_hist[front_hist_idx] = (raw_fr > 4.0f && raw_fr < 2000.0f) ? raw_fr : 999.0f;
    front_hist_idx = (front_hist_idx + 1) % 3;

    float front_l = Median3(fl_hist[0], fl_hist[1], fl_hist[2]);
    float front_r = Median3(fr_hist[0], fr_hist[1], fr_hist[2]);

    sf_data.front_l = front_l;
    sf_data.front_r = front_r;
    sf_data.diag_l  = diag_l;
    sf_data.diag_r  = diag_r;

    // ==============================================================
    // 1. NHAN DIEN TUONG TRUOC & KHOA DONG THUAN 2 MAT (STEREO CHECK)
    //    Vach that phia truoc BAT BUOC ca 2 mat FL & FR cung nhin thay
    //    va chenh lech <= 55mm (loai bo tia le phan xa doc hanh lang)
    // ==============================================================
    bool fl_valid = (front_l > 4.0f && front_l < 500.0f);
    bool fr_valid = (front_r > 4.0f && front_r < 500.0f);

    if (fl_valid && fr_valid && fabsf(front_l - front_r) <= FRONT_STEREO_MAX_DIFF_MM) {
        sf_data.front_dist = (front_l + front_r) * 0.5f;
        sf_data.front_diff = front_l - front_r;
    } else {
        // Neu 1 mat bao 8190 (999) hoac 2 mat lech > 55mm (vd FL=191, FR=1) -> Khong co vach chan
        sf_data.front_dist = 999.0f;
        sf_data.front_diff = 0.0f;
    }

    if (sf_data.front_dist < WALL_FRONT_THRESHOLD) {
        if (front_wall_confirm_cnt < FRONT_CONFIRM_FRAMES) {
            front_wall_confirm_cnt++;
        }
    } else {
        front_wall_confirm_cnt = 0;
    }

    sf_data.wall_front = (front_wall_confirm_cnt >= FRONT_CONFIRM_FRAMES);

    // ==============================================================
    // 2. NHAN DIEN TUONG BEN TRONG CUA SO QUET
    // ==============================================================
    float avg_dist = (dist_l_mm + dist_r_mm) * 0.5f;
    if ((avg_dist <= SIDE_WALL_SCAN_MAX_MM && sf_data.front_dist > FRONT_WALL_LOCK_PID_MM) || avg_dist < 15.0f) {
        sf_data.wall_left  = (diag_l > 0.5f && diag_l < WALL_DIAG_THRESHOLD);
        sf_data.wall_right = (diag_r > 0.5f && diag_r < WALL_DIAG_THRESHOLD);
    }

    // ==============================================================
    // 3. TINH SAI SO LECH TAM (WALL_ERROR) CHO BO DIEU KHIEN PID
    // ==============================================================
    float raw_error = 0.0f;

    bool left_wall_now  = (diag_l > 0.5f && diag_l < WALL_DIAG_THRESHOLD);
    bool right_wall_now = (diag_r > 0.5f && diag_r < WALL_DIAG_THRESHOLD);

    // Chi ngat PID vach khi tien sat tuong truoc (<= 135mm) hoac mat CA 2 vach ben
    if (sf_data.front_dist <= FRONT_WALL_LOCK_PID_MM || (!left_wall_now && !right_wall_now)) {
        raw_error = 0.0f;
    } 
    else if (left_wall_now && right_wall_now) {
        // Uu tien 1: Co ca 2 tuong ben -> Can giua hanh lang
        raw_error = (diag_l - diag_r) * COS_45_DEG;
    } 
    else if (left_wall_now) {
        // Uu tien 2: Chi co tuong trai (ben phai mo):
        // - Neu xe xa vach trai (raw_error > 0): Kep hut vao <= +10mm (chong hut goc nga re)
        // - Neu xe sat vach trai (raw_error < 0): KHONG KEP! Giu nguyen luc day ra giua tim o
        raw_error = 2.0f * (diag_l - TARGET_DIAG_DIST) * COS_45_DEG;
        if (raw_error > MAX_CORNER_PULL_MM) {
            raw_error = MAX_CORNER_PULL_MM;
        }
    } 
    else {
        // Uu tien 2: Chi co tuong phai (ben trai mo):
        // - Neu xe xa vach phai (raw_error < 0): Kep hut vao >= -10mm (chong hut goc nga re)
        // - Neu xe sat vach phai (raw_error > 0): KHONG KEP! Giu nguyen luc day ra giua tim o
        raw_error = 2.0f * (TARGET_DIAG_DIST - diag_r) * COS_45_DEG;
        if (raw_error < -MAX_CORNER_PULL_MM) {
            raw_error = -MAX_CORNER_PULL_MM;
        }
    }

    // Loc vung chet (Deadband)
    if (fabsf(raw_error) < ERROR_DEADBAND_MM) {
        raw_error = 0.0f;
    }

    // Gioi han sai so toi da (Clamping +-45mm)
    if (raw_error > MAX_WALL_ERROR_CLAMP) {
        raw_error = MAX_WALL_ERROR_CLAMP;
    } else if (raw_error < -MAX_WALL_ERROR_CLAMP) {
        raw_error = -MAX_WALL_ERROR_CLAMP;
    }
    sf_data.wall_error = raw_error;

    // ==============================================================
    // 4. HOP NHAT GOC XOAY (IMU GYRO Z + ENCODER)
    // ==============================================================
    if (fabsf(dist_l_mm - prev_dist_l) > 80.0f || fabsf(dist_r_mm - prev_dist_r) > 80.0f) {
        prev_dist_l = dist_l_mm;
        prev_dist_r = dist_r_mm;
    }

    float delta_l = dist_l_mm - prev_dist_l;
    float delta_r = dist_r_mm - prev_dist_r;
    prev_dist_l = dist_l_mm;
    prev_dist_r = dist_r_mm;

    float delta_angle_encoder = ((delta_r - delta_l) / WHEEL_TRACK_MM) * RAD_TO_DEG;
    float delta_fused_angle = 0.0f;

    if (mpu_data != NULL && mpu_data->is_connected) {
        float delta_angle_gyro = mpu_data->gyro_z * dt;
        if (fabsf(mpu_data->gyro_z) > 10.0f) {
            delta_fused_angle = delta_angle_gyro;
        } else {
            delta_fused_angle = 0.996f * delta_angle_gyro + 0.004f * delta_angle_encoder;
        }
    } else {
        delta_fused_angle = delta_angle_encoder;
    }

    sf_data.fused_angle += delta_fused_angle;
}

SensorFusion_Data* SensorFusion_GetData(void) {
    return &sf_data;
}

void SensorFusion_ResetAngle(void) {
    sf_data.fused_angle = 0.0f;
    MPU6500_ResetYaw();
}