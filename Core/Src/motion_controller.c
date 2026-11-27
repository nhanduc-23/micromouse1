/**
  ******************************************************************************
  * @file           : motion_controller.c
  * @brief          : Re gan tuong + Quay ngoat nhanh (Snap Turn) + Can chinh tam o ngay lap tuc
  *                   Toi uu hoa toc do cao cho sa ban thi dau 16x16 (High-Speed Tier):
  *                   - Toc do do duong (Explore / Return): 305 PWM (cua so quet 15-82mm)
  *                   - Speedrun 2 nac toc do: 420 PWM (ngan 1-3 o) / 505 PWM (dai >= 4 o)
  *                   - Quay ngoat Snap Turn: 345 PWM (Explore) / 390 PWM (Speedrun) + phanh nguoc 165 PWM
  *                   - Mo rong doc phanh (150mm / ToF 180mm) giu dung chuan sat tuong 45mm
  ******************************************************************************
  */

#include "motion_controller.h"
#include "config.h"
#include "tb6612fng.h"
#include "encoder.h"
#include <math.h>

#define BASE_DRIVE_PWM              305.0f  // Tang toc do duong (265 -> 305 PWM)
#define FAST_DRIVE_SHORT_PWM        420.0f  // Speedrun doan thang ngan 1 - 3 o (365 -> 420 PWM)
#define FAST_DRIVE_LONG_PWM         505.0f  // Speedrun hanh lang thang dai >= 4 o (435 -> 505 PWM)
#define MIN_DRIVE_PWM               95.0f

// Moc dung sat tuong truoc khi re (bu 1-2mm quan tinh phanh o toc do cao)
#define EXPLORE_FRONT_STOP_MM       45.0f   // Dung cach tuong truoc 45mm o che do do duong
#define FAST_FRONT_STOP_MM          45.0f   // Dung cach tuong truoc 45mm o che do Speedrun

static bool is_fast_mode = false;

// Dai PWM cho Quay ngoat nhanh (Snap Pivot Turn 2 banh)
#define MIN_ROTATE_PWM              205.0f
#define EXPLORE_ROTATE_MAX_PWM      345.0f  // Quay ngoat toc do cao khi do duong (315 -> 345 PWM)
#define FAST_ROTATE_MAX_PWM         390.0f  // Quay ngoat cuc nhanh khi Speedrun (355 -> 390 PWM)

// Phanh xung nguoc chu dong & Vi chinh goc nhanh sau khi quay ngoat
#define REVERSE_BRAKE_PWM           165.0f  // Tang xung ham nguoc dap quan tinh xoay cao (145 -> 165 PWM)
#define REVERSE_BRAKE_TIME_MS       19      // Thoi gian phanh xung nguoc (ms)
#define PASSIVE_BRAKE_END_MS        30      // Thoi gian khoa phanh ngan mach (ms)
#define SETTLING_TIME_MS            52      // Tong thoi gian on dinh gon (ms)
#define MICRO_ALIGN_MIN_PWM         95.0f
#define MICRO_ALIGN_MAX_PWM         115.0f
#define ALIGN_TOLERANCE_DEG         0.8f

// Vung can chinh tam o ngay lap tuc sau khi re xong (90mm dau tien)
#define POST_TURN_RECENTER_MM       90.0f

// Cua so quet vach o moi (15-82mm de du mau ToF o 305 PWM) va Diem chot (135mm)
#define CELL_SCAN_START_MM          15.0f
#define CELL_SCAN_END_MM            82.0f
#define CELL_DECISION_MM            135.0f

// Moc hinh hoc canh ket thuc vach ben: 264mm (mep vach o ke) - 144mm (tam nhin cheo) = 120mm
#define WALL_EDGE_TARGET_MM         120.0f

#define SIDE_OPEN_EARLY_MM          188.0f
#define FORWARD_TIMEOUT_MS          10000
#define ROTATE_TIMEOUT_MS           2000

static PID_Controller pid_wall;
static PID_Controller pid_gyro;
static MotionState current_state = MOTION_STATE_IDLE;

static float target_distance = 0.0f;
static float target_angle = 0.0f;
static float cardinal_angle = 0.0f;
static bool is_finished = true;

// Bien loc muot sai so vach & tu dong can bang 2 banh khi di thang
static float filtered_wall_error = 0.0f;
static float straight_trim_pwm   = 0.0f;

// Co kich hoat can chinh tam o ngay lap tuc sau cu re
static bool just_finished_turn = false;
static bool post_turn_recenter_active = false;

static uint32_t forward_start_tick = 0;
static uint32_t rotate_start_tick = 0;
static uint32_t settle_start_tick = 0;
static bool is_settling = false;
static bool is_turn_left = false;

// Bien quan ly chay lien tuc qua nhieu o (Explore / Return)
static float prev_enc_avg_mm = 0.0f;
static float cell_dist_mm = 0.0f;
static bool cell_scan_started = false;
static bool cell_checkpoint_ready = false;
static bool cell_checkpoint_dispatched = false;

// Bo dem phieu quet vach trong cua so 15mm - 82mm cua o moi
static uint8_t vote_wall_l = 0, vote_open_l = 0;
static uint8_t vote_wall_r = 0, vote_open_r = 0;

// Theo doi trang thai vach truoc do de dong bo canh vach (Wall-Edge Sync)
static bool prev_edge_wall_l = false;
static bool prev_edge_wall_r = false;
static bool edge_synced_this_cell = false;

// Bu sai so doc cho che do Fast Run nhieu o lien tiep
static float fwd_dist_offset = 0.0f;
static int8_t last_fwd_sync_cell = -1;

// Co tuong thich cho che do MoveForward co dinh
static bool early_open_left    = false;
static bool early_open_right   = false;
static bool latched_turn_left  = false;
static bool latched_turn_right = false;

static void ResetCellScanVotes(void) {
    vote_wall_l = 0;
    vote_open_l = 0;
    vote_wall_r = 0;
    vote_open_r = 0;
}

void Motion_Init(void) {
    PID_Init(&pid_wall, 0.30f, 0.0f, 0.0f, -40.0f, 40.0f);
    PID_Init(&pid_gyro, 3.45f, 0.0f, 0.58f, -FAST_ROTATE_MAX_PWM, FAST_ROTATE_MAX_PWM);
    cardinal_angle = 0.0f;
    target_angle = 0.0f;
    filtered_wall_error = 0.0f;
    straight_trim_pwm = 0.0f;
    just_finished_turn = false;
    post_turn_recenter_active = false;
    Motion_Stop();
}

void Motion_UpdatePIDWall(float kp, float ki, float kd) {
    pid_wall.kp = kp;
    pid_wall.ki = ki;
    pid_wall.kd = kd;
    pid_wall.integral = 0.0f;
}

void Motion_UpdatePIDGyro(float kp, float ki, float kd) {
    pid_gyro.kp = kp;
    pid_gyro.ki = ki;
    pid_gyro.kd = kd;
    pid_gyro.integral = 0.0f;
}

void Motion_MoveForward(float distance_mm) {
    if (distance_mm >= FULL_CELL) {
        // Bu tru quan tinh phanh cho dai toc do 420-505 PWM (9mm) va 305 PWM (11mm)
        target_distance = is_fast_mode ? (distance_mm - 9.0f) : (distance_mm - 11.0f);
    } else {
        target_distance = distance_mm;
    }

    // Kich hoat che do can chinh tam o ngay lap tuc neu vua thoat khoi cu re
    if (just_finished_turn && distance_mm >= 60.0f) {
        post_turn_recenter_active = true;
        just_finished_turn = false;
    } else {
        post_turn_recenter_active = false;
        just_finished_turn = false;
    }

    Encoder_Reset();
    SensorFusion_ResetEncoders();
    PID_Reset(&pid_wall);
    PID_Reset(&pid_gyro);
    filtered_wall_error = 0.0f;

    target_angle = cardinal_angle;
    fwd_dist_offset = 0.0f;
    last_fwd_sync_cell = -1;

    SensorFusion_Data *sf = SensorFusion_GetData();
    prev_edge_wall_l = (sf->diag_l > 1.0f && sf->diag_l < WALL_DIAG_THRESHOLD);
    prev_edge_wall_r = (sf->diag_r > 1.0f && sf->diag_r < WALL_DIAG_THRESHOLD);

    early_open_left    = false;
    early_open_right   = false;
    latched_turn_left  = false;
    latched_turn_right = false;

    current_state = MOTION_STATE_FORWARD;
    is_finished = false;
    is_settling = false;
    forward_start_tick = HAL_GetTick();
}

void Motion_StartContinuousForward(void) {
    // Kich hoat che do can chinh tam o ngay lap tuc neu vua thoat khoi cu re
    post_turn_recenter_active = just_finished_turn;
    just_finished_turn = false;

    Encoder_Reset();
    SensorFusion_ResetEncoders();
    PID_Reset(&pid_wall);
    PID_Reset(&pid_gyro);
    filtered_wall_error = 0.0f;

    target_angle = cardinal_angle;
    prev_enc_avg_mm = 0.0f;
    cell_dist_mm = 0.0f;

    cell_scan_started = false;
    cell_checkpoint_ready = false;
    cell_checkpoint_dispatched = false;
    edge_synced_this_cell = false;

    SensorFusion_Data *sf = SensorFusion_GetData();
    prev_edge_wall_l = (sf->diag_l > 1.0f && sf->diag_l < WALL_DIAG_THRESHOLD);
    prev_edge_wall_r = (sf->diag_r > 1.0f && sf->diag_r < WALL_DIAG_THRESHOLD);

    ResetCellScanVotes();

    current_state = MOTION_STATE_CONTINUOUS;
    is_finished = false;
    is_settling = false;
    forward_start_tick = HAL_GetTick();
}

bool Motion_IsCellCheckpointReady(void) {
    return cell_checkpoint_ready;
}

void Motion_GetCellWallVotes(bool *has_left, bool *has_right) {
    SensorFusion_Data *sf = SensorFusion_GetData();

    if (has_left) {
        if ((vote_wall_l + vote_open_l) > 0) {
            *has_left = !(vote_open_l > vote_wall_l);
        } else {
            if (sf->diag_l >= SIDE_OPEN_EARLY_MM) {
                *has_left = false;
            } else {
                *has_left = (sf->front_dist > 120.0f && sf->diag_l > 1.0f && sf->diag_l < WALL_DIAG_THRESHOLD);
            }
        }
    }

    if (has_right) {
        if ((vote_wall_r + vote_open_r) > 0) {
            *has_right = !(vote_open_r > vote_wall_r);
        } else {
            if (sf->diag_r >= SIDE_OPEN_EARLY_MM) {
                *has_right = false;
            } else {
                *has_right = (sf->front_dist > 120.0f && sf->diag_r > 1.0f && sf->diag_r < WALL_DIAG_THRESHOLD);
            }
        }
    }
}

void Motion_ContinueNextCell(void) {
    cell_dist_mm -= FULL_CELL;
    cell_scan_started = false;
    cell_checkpoint_ready = false;
    cell_checkpoint_dispatched = false;
    edge_synced_this_cell = false;
    ResetCellScanVotes();
    forward_start_tick = HAL_GetTick();
}

void Motion_CommitStopAtCenter(void) {
    cell_checkpoint_ready = false;
    if (is_finished || current_state == MOTION_STATE_IDLE) {
        return;
    }
    // Tru 12mm quan tinh phanh tu 305 PWM ve dung tam o / sat tuong
    target_distance = FULL_CELL - 12.0f;
    current_state = MOTION_STATE_STOP_AT_CENTER;
    is_finished = false;
}

void Motion_Rotate(float angle_deg) {
    cardinal_angle += angle_deg;
    target_angle = cardinal_angle;

    is_turn_left = (angle_deg > 0.0f);
    post_turn_recenter_active = false;
    just_finished_turn = false;

    PID_Reset(&pid_gyro);
    filtered_wall_error = 0.0f;
    straight_trim_pwm   = 0.0f;

    early_open_left    = false;
    early_open_right   = false;
    latched_turn_left  = false;
    latched_turn_right = false;

    current_state = MOTION_STATE_ROTATE;
    is_finished = false;
    is_settling = false;
    rotate_start_tick = HAL_GetTick();
}

void Motion_Stop(void) {
    TB6612_Brake();
    current_state = MOTION_STATE_IDLE;
    is_finished = true;
    is_settling = false;
    cell_checkpoint_ready = false;
}

void Motion_SetFastMode(bool enable_fast) {
    is_fast_mode = enable_fast;
}

bool Motion_IsFinished(void) {
    return is_finished;
}

bool Motion_GetLatchedTurnRight(void) {
    bool state = latched_turn_right;
    latched_turn_right = false;
    return state;
}

bool Motion_GetLatchedTurnLeft(void) {
    bool state = latched_turn_left;
    latched_turn_left = false;
    return state;
}

bool Motion_AlignHeadingStatic(void) {
    SensorFusion_Data *sf = SensorFusion_GetData();
    float heading_error = target_angle - sf->fused_angle;

    if (fabsf(heading_error) <= ALIGN_TOLERANCE_DEG) {
        TB6612_Brake();
        return true;
    }

    float align_pwm = MICRO_ALIGN_MIN_PWM + fabsf(heading_error) * 12.0f;
    if (align_pwm > MICRO_ALIGN_MAX_PWM) align_pwm = MICRO_ALIGN_MAX_PWM;

    if (heading_error > 0.0f) {
        TB6612_SetSpeed(-(int16_t)align_pwm, (int16_t)align_pwm);
    } else {
        TB6612_SetSpeed((int16_t)align_pwm, -(int16_t)align_pwm);
    }

    return false;
}

void Motion_SoftReverse(uint16_t time_ms) {
    TB6612_SetSpeed(-(int16_t)DOCK_REVERSE_PWM, -(int16_t)DOCK_REVERSE_PWM);
    HAL_Delay(time_ms);
    TB6612_Brake();
    HAL_Delay(30);
}

void Motion_ResetOdometryAtCenter(void) {
    SensorFusion_Data *sf = SensorFusion_GetData();
    sf->fused_angle = cardinal_angle;
    target_angle    = cardinal_angle;

    Encoder_Reset();
    SensorFusion_ResetEncoders();
    PID_Reset(&pid_wall);
    PID_Reset(&pid_gyro);
    filtered_wall_error = 0.0f;
    straight_trim_pwm = 0.0f;
    just_finished_turn = false;
    post_turn_recenter_active = false;
    Motion_Stop();
}

void Motion_SnapAngleToCardinal(void) {
    SensorFusion_Data *sf = SensorFusion_GetData();
    sf->fused_angle = cardinal_angle;
    target_angle    = cardinal_angle;
    filtered_wall_error = 0.0f;
    straight_trim_pwm   = 0.0f;
    PID_Reset(&pid_wall);
    PID_Reset(&pid_gyro);
}

void Motion_ResetEncodersAtCenter(void) {
    Encoder_Reset();
    SensorFusion_ResetEncoders();
    PID_Reset(&pid_wall);
    PID_Reset(&pid_gyro);
    filtered_wall_error = 0.0f;
    straight_trim_pwm   = 0.0f;
    just_finished_turn = false;
    post_turn_recenter_active = false;
    Motion_Stop();
}

// ==============================================================
// BO DIEU KHIEN BAM TIM & CAN CHINH TAM O NGAY LAP TUC SAU KHI RE
// ==============================================================
static float ComputeStraightSteering(SensorFusion_Data *sf, float dist_from_start, float dt) {
    bool recenter_burst = (post_turn_recenter_active &&
                           dist_from_start >= 0.0f &&
                           dist_from_start <= POST_TURN_RECENTER_MM);

    if (recenter_burst) {
        // Pha can chinh tam o ngay lap tuc (0 - 90mm sau cua):
        filtered_wall_error = 0.45f * filtered_wall_error + 0.55f * sf->wall_error;

        float angle_bias = filtered_wall_error * 0.35f;
        if (angle_bias > 10.0f)  angle_bias = 10.0f;
        if (angle_bias < -10.0f) angle_bias = -10.0f;

        float dynamic_target_angle = target_angle + angle_bias;
        float heading_corr = PID_Compute(&pid_gyro, sf->fused_angle, dynamic_target_angle, dt);
        float wall_corr = (pid_wall.kp * 1.65f) * filtered_wall_error;

        float correction = heading_corr + wall_corr + straight_trim_pwm;
        if (correction > 60.0f)  correction = 60.0f;
        if (correction < -60.0f) correction = -60.0f;
        return correction;
    }

    // Pha hanh trinh on dinh sau khi da vao dung tim o
    filtered_wall_error = 0.82f * filtered_wall_error + 0.18f * sf->wall_error;

    float angle_bias = filtered_wall_error * 0.22f;
    if (angle_bias > 6.5f)  angle_bias = 6.5f;
    if (angle_bias < -6.5f) angle_bias = -6.5f;

    float dynamic_target_angle = target_angle + angle_bias;

    float heading_corr = PID_Compute(&pid_gyro, sf->fused_angle, dynamic_target_angle, dt);

    float wall_corr = pid_wall.kp * filtered_wall_error;
    if (!post_turn_recenter_active && dist_from_start >= 0.0f && dist_from_start < 25.0f) {
        wall_corr *= (dist_from_start / 25.0f);
    }

    // Chi tich phan sai so goc so voi truc chuan cardinal (tranh windup do angle_bias)
    float heading_err = target_angle - sf->fused_angle;
    straight_trim_pwm += heading_err * 0.12f * dt;
    if (straight_trim_pwm > 8.0f)  straight_trim_pwm = 8.0f;
    if (straight_trim_pwm < -8.0f) straight_trim_pwm = -8.0f;

    float correction = heading_corr + wall_corr + straight_trim_pwm;
    if (correction > 52.0f)  correction = 52.0f;
    if (correction < -52.0f) correction = -52.0f;

    return correction;
}

void Motion_Update(float dt) {
    if (current_state == MOTION_STATE_IDLE) return;

    SensorFusion_Data *sf = SensorFusion_GetData();

    // ==============================================================
    // 1. CHE DO CHAY THANG LIEN TUC & TIEN SAT TUONG DE RE (EXPLORE / RETURN)
    // ==============================================================
    if (current_state == MOTION_STATE_CONTINUOUS || current_state == MOTION_STATE_STOP_AT_CENTER) {
        float dist_l = 0.0f, dist_r = 0.0f;
        Encoder_GetDistanceMM(&dist_l, &dist_r);
        float total_avg_mm = (dist_l + dist_r) * 0.5f;
        float delta_mm = total_avg_mm - prev_enc_avg_mm;
        prev_enc_avg_mm = total_avg_mm;

        cell_dist_mm += delta_mm;

        if (HAL_GetTick() - forward_start_tick > FORWARD_TIMEOUT_MS) {
            Motion_Stop();
            return;
        }

        // Phanh khan cap Crash Guard (<= 32mm)
        if (sf->front_dist > 10.0f && sf->front_dist <= CRASH_DIST_LIMIT_MM) {
            bool need_cp = (current_state == MOTION_STATE_CONTINUOUS &&
                            cell_dist_mm >= 85.0f && !cell_checkpoint_dispatched);
            Motion_Stop();
            if (need_cp) {
                cell_checkpoint_ready = true;
                cell_checkpoint_dispatched = true;
            }
            return;
        }

        // A. DONG BO CANH VACH (WALL-EDGE SYNC) - HIEU CHUAN MOC 120.0MM CHO 16x16
        bool cur_edge_wall_l = (sf->diag_l > 1.0f && sf->diag_l < WALL_DIAG_THRESHOLD);
        bool cur_edge_wall_r = (sf->diag_r > 1.0f && sf->diag_r < WALL_DIAG_THRESHOLD);

        if (!edge_synced_this_cell && sf->front_dist > 135.0f &&
            cell_dist_mm >= 100.0f && cell_dist_mm <= 135.0f) {
            bool left_falling  = (prev_edge_wall_l && !cur_edge_wall_l && sf->diag_l >= SIDE_OPEN_EARLY_MM);
            bool right_falling = (prev_edge_wall_r && !cur_edge_wall_r && sf->diag_r >= SIDE_OPEN_EARLY_MM);

            if (left_falling || right_falling) {
                cell_dist_mm = 0.65f * cell_dist_mm + 0.35f * WALL_EDGE_TARGET_MM;
                edge_synced_this_cell = true;
            }
        }
        prev_edge_wall_l = cur_edge_wall_l;
        prev_edge_wall_r = cur_edge_wall_r;

        // B. QUET VACH TRONG CUA SO 15mm -> 82mm & CHOT QUYET DINH TAI 135mm
        if (current_state == MOTION_STATE_CONTINUOUS) {
            if (!cell_scan_started && cell_dist_mm >= CELL_SCAN_START_MM) {
                cell_scan_started = true;
                ResetCellScanVotes();
            }

            if (cell_dist_mm >= CELL_SCAN_START_MM && cell_dist_mm <= CELL_SCAN_END_MM) {
                if (sf->front_dist > 95.0f) {
                    if (sf->diag_l >= SIDE_OPEN_EARLY_MM && sf->diag_l < 2000.0f) {
                        vote_open_l++;
                    }
                    if (sf->diag_r >= SIDE_OPEN_EARLY_MM && sf->diag_r < 2000.0f) {
                        vote_open_r++;
                    }
                }

                if (sf->front_dist > 120.0f) {
                    if (sf->diag_l > 1.0f && sf->diag_l < WALL_DIAG_THRESHOLD) {
                        vote_wall_l++;
                    }
                    if (sf->diag_r > 1.0f && sf->diag_r < WALL_DIAG_THRESHOLD) {
                        vote_wall_r++;
                    }
                }
            }

            if (sf->wall_front && cell_dist_mm >= 85.0f &&
                sf->front_dist > 10.0f && sf->front_dist <= EXPLORE_FRONT_STOP_MM) {
                bool need_cp = !cell_checkpoint_dispatched;
                Motion_Stop();
                if (need_cp) {
                    cell_checkpoint_ready = true;
                    cell_checkpoint_dispatched = true;
                }
                return;
            }

            if (!cell_checkpoint_dispatched && cell_dist_mm >= CELL_DECISION_MM) {
                cell_checkpoint_ready = true;
                cell_checkpoint_dispatched = true;
            }
        }

        // C. DIEU KIEN DUNG SAT TUONG TRUOC (45mm) HOAC TAM O (168mm) DE QUAY NGOAT
        if (current_state == MOTION_STATE_STOP_AT_CENTER) {
            if (sf->wall_front) {
                if (sf->front_dist > 10.0f && sf->front_dist <= EXPLORE_FRONT_STOP_MM) {
                    Motion_Stop();
                    return;
                }
                if (cell_dist_mm >= (target_distance + 20.0f)) {
                    Motion_Stop();
                    return;
                }
            } else {
                if (cell_dist_mm >= target_distance) {
                    Motion_Stop();
                    return;
                }
            }
        }

        // D. TINH TOAN TOC DO & CAN CHINH TIM O (305 PWM)
        float base_speed = BASE_DRIVE_PWM;

        if (total_avg_mm < 50.0f) {
            float accel_ratio = total_avg_mm / 50.0f;
            if (accel_ratio < 0.0f) accel_ratio = 0.0f;
            base_speed = MIN_DRIVE_PWM + ((BASE_DRIVE_PWM - MIN_DRIVE_PWM) * accel_ratio);
        }

        // Ham phanh theo ToF mat truoc tu 145mm -> dung chuan tai 45mm
        if (cell_dist_mm > 80.0f && sf->wall_front && sf->front_dist > 10.0f && sf->front_dist < 145.0f) {
            float margin = sf->front_dist - EXPLORE_FRONT_STOP_MM;
            if (margin < 0.0f) margin = 0.0f;
            float ratio = margin / (145.0f - EXPLORE_FRONT_STOP_MM);
            if (ratio > 1.0f) ratio = 1.0f;
            float f_speed = MIN_DRIVE_PWM + ((BASE_DRIVE_PWM - MIN_DRIVE_PWM) * ratio);
            if (f_speed < base_speed) base_speed = f_speed;
        }

        if (current_state == MOTION_STATE_STOP_AT_CENTER) {
            float rem = target_distance - cell_dist_mm;
            if (rem < 62.0f) {
                float decel_ratio = rem / 62.0f;
                if (decel_ratio < 0.0f) decel_ratio = 0.0f;
                float d_speed = MIN_DRIVE_PWM + ((BASE_DRIVE_PWM - MIN_DRIVE_PWM) * decel_ratio);
                if (d_speed < base_speed) base_speed = d_speed;
            }
        }

        float correction = ComputeStraightSteering(sf, total_avg_mm, dt);
        if (current_state == MOTION_STATE_STOP_AT_CENTER && fabsf(correction) > 26.0f) {
            correction = (correction > 0.0f) ? 26.0f : -26.0f;
        }

        int speed_left  = (int)(base_speed + correction);
        int speed_right = (int)(base_speed - correction);

        if (speed_left < (int)MIN_DRIVE_PWM)  speed_left  = (int)MIN_DRIVE_PWM;
        if (speed_right < (int)MIN_DRIVE_PWM) speed_right = (int)MIN_DRIVE_PWM;

        TB6612_SetSpeed((int16_t)speed_left, (int16_t)speed_right);
    }

    // ==============================================================
    // 2. CHE DO DI THANG CO DINH (FAST RUN 16x16 & TIEN NGAN SAU TY TUONG)
    // ==============================================================
    else if (current_state == MOTION_STATE_FORWARD) {
        float dist_l = 0.0f, dist_r = 0.0f;
        Encoder_GetDistanceMM(&dist_l, &dist_r);
        float raw_dist = (dist_l + dist_r) * 0.5f;

        // Dong bo canh vach o cac o trung gian (khoa truoc vung phanh 155mm)
        bool cur_edge_wall_l = (sf->diag_l > 1.0f && sf->diag_l < WALL_DIAG_THRESHOLD);
        bool cur_edge_wall_r = (sf->diag_r > 1.0f && sf->diag_r < WALL_DIAG_THRESHOLD);

        if (target_distance >= 60.0f && sf->front_dist > 135.0f) {
            float est_dist = raw_dist + fwd_dist_offset;
            if (est_dist > 0.0f && est_dist < (target_distance - 155.0f)) {
                int8_t cell_idx = (int8_t)(est_dist / FULL_CELL);
                float local_dist = est_dist - ((float)cell_idx * FULL_CELL);

                if (cell_idx != last_fwd_sync_cell &&
                    local_dist >= 100.0f && local_dist <= 135.0f) {
                    bool left_falling  = (prev_edge_wall_l && !cur_edge_wall_l && sf->diag_l >= SIDE_OPEN_EARLY_MM);
                    bool right_falling = (prev_edge_wall_r && !cur_edge_wall_r && sf->diag_r >= SIDE_OPEN_EARLY_MM);

                    if (left_falling || right_falling) {
                        float expected_total = ((float)cell_idx * FULL_CELL) + WALL_EDGE_TARGET_MM;
                        fwd_dist_offset += 0.35f * (expected_total - est_dist);
                        if (fwd_dist_offset > 15.0f)  fwd_dist_offset = 15.0f;
                        if (fwd_dist_offset < -35.0f) fwd_dist_offset = -35.0f;
                        last_fwd_sync_cell = cell_idx;
                    }
                }
            }
        }
        prev_edge_wall_l = cur_edge_wall_l;
        prev_edge_wall_r = cur_edge_wall_r;

        float current_dist = raw_dist + fwd_dist_offset;

        if (HAL_GetTick() - forward_start_tick > FORWARD_TIMEOUT_MS) {
            Motion_Stop();
            return;
        }

        if (sf->front_dist > 10.0f && sf->front_dist <= CRASH_DIST_LIMIT_MM) {
            Motion_Stop();
            return;
        }

        // Dung sat tuong truoc (45.0mm) hoac dung tam o (target_distance) roi moi quay ngoat
        float active_front_stop = is_fast_mode ? FAST_FRONT_STOP_MM : EXPLORE_FRONT_STOP_MM;

        if (sf->wall_front && target_distance >= 60.0f) {
            if (sf->front_dist > 10.0f && sf->front_dist <= active_front_stop) {
                Motion_Stop();
                return;
            }
            if (current_dist >= (target_distance + 20.0f)) {
                Motion_Stop();
                return;
            }
        } else {
            if (current_dist >= target_distance) {
                Motion_Stop();
                return;
            }
        }

        float remaining_dist = target_distance - current_dist;
        float fast_top_pwm = (target_distance >= (3.5f * FULL_CELL)) ? FAST_DRIVE_LONG_PWM : FAST_DRIVE_SHORT_PWM;
        float max_drive_pwm = is_fast_mode ? fast_top_pwm : BASE_DRIVE_PWM;
        float base_speed = max_drive_pwm;

        if (target_distance < 60.0f) {
            base_speed = 115.0f;
            if (remaining_dist < 12.0f) {
                float crawl = remaining_dist / 12.0f;
                if (crawl < 0.0f) crawl = 0.0f;
                base_speed = 90.0f + (25.0f * crawl);
            }
        } else {
            // Doc tang/giam toc can chinh cho dai 420 - 505 PWM
            float accel_zone = is_fast_mode ? 70.0f : 50.0f;
            float decel_zone = is_fast_mode ? 150.0f : 62.0f;

            if (current_dist < accel_zone) {
                float accel = current_dist / accel_zone;
                if (accel < 0.0f) accel = 0.0f;
                base_speed = MIN_DRIVE_PWM + ((max_drive_pwm - MIN_DRIVE_PWM) * accel);
            } else if (remaining_dist < decel_zone) {
                float decel = remaining_dist / decel_zone;
                if (decel < 0.0f) decel = 0.0f;
                base_speed = MIN_DRIVE_PWM + ((max_drive_pwm - MIN_DRIVE_PWM) * decel);
            }

            // Phanh som theo ToF mat truoc tu 180mm -> dung sat tuong tai 45mm
            float front_decel_start = is_fast_mode ? 180.0f : 145.0f;
            if (sf->front_dist > 10.0f && sf->front_dist < front_decel_start) {
                float margin = sf->front_dist - active_front_stop;
                if (margin < 0.0f) margin = 0.0f;
                float ratio = margin / (front_decel_start - active_front_stop);
                if (ratio > 1.0f) ratio = 1.0f;
                float f_speed = MIN_DRIVE_PWM + ((max_drive_pwm - MIN_DRIVE_PWM) * ratio);
                if (f_speed < base_speed) base_speed = f_speed;
            }
        }

        float correction = 0.0f;
        if (target_distance < 60.0f) {
            correction = PID_Compute(&pid_gyro, sf->fused_angle, target_angle, dt);
            if (correction > 20.0f)  correction = 20.0f;
            if (correction < -20.0f) correction = -20.0f;
        } else {
            correction = ComputeStraightSteering(sf, current_dist, dt);
            if (remaining_dist < 45.0f && fabsf(correction) > 26.0f) {
                correction = (correction > 0.0f) ? 26.0f : -26.0f;
            }
        }

        int speed_left  = (int)(base_speed + correction);
        int speed_right = (int)(base_speed - correction);

        if (speed_left < (int)MIN_DRIVE_PWM)  speed_left  = (int)MIN_DRIVE_PWM;
        if (speed_right < (int)MIN_DRIVE_PWM) speed_right = (int)MIN_DRIVE_PWM;

        TB6612_SetSpeed((int16_t)speed_left, (int16_t)speed_right);
    }

    // ==============================================================
    // 3. QUAY NGOAT NHANH SANG HUONG CAN RE (SNAP PIVOT TURN)
    // ==============================================================
    else if (current_state == MOTION_STATE_ROTATE) {
        float angle_error = target_angle - sf->fused_angle;
        // Bat pha phanh xung nguoc som 4.0 do truoc dich de triet tieu quan tinh quay cao
        bool target_reached = is_turn_left ? (angle_error <= 4.0f) : (angle_error >= -4.0f);

        if (is_settling || target_reached) {
            if (!is_settling) {
                is_settling = true;
                settle_start_tick = HAL_GetTick();
                if (is_turn_left) {
                    TB6612_SetSpeed((int16_t)REVERSE_BRAKE_PWM, -(int16_t)REVERSE_BRAKE_PWM);
                } else {
                    TB6612_SetSpeed(-(int16_t)REVERSE_BRAKE_PWM, (int16_t)REVERSE_BRAKE_PWM);
                }
                return;
            }

            uint32_t settle_elapsed = HAL_GetTick() - settle_start_tick;

            if (settle_elapsed < REVERSE_BRAKE_TIME_MS) {
                if (is_turn_left) {
                    TB6612_SetSpeed((int16_t)REVERSE_BRAKE_PWM, -(int16_t)REVERSE_BRAKE_PWM);
                } else {
                    TB6612_SetSpeed(-(int16_t)REVERSE_BRAKE_PWM, (int16_t)REVERSE_BRAKE_PWM);
                }
            } else if (settle_elapsed < PASSIVE_BRAKE_END_MS) {
                TB6612_Brake();
            } else if (settle_elapsed < SETTLING_TIME_MS) {
                float align_error = target_angle - sf->fused_angle;
                if (fabsf(align_error) > ALIGN_TOLERANCE_DEG) {
                    float align_pwm = MICRO_ALIGN_MIN_PWM + fabsf(align_error) * 12.0f;
                    if (align_pwm > MICRO_ALIGN_MAX_PWM) align_pwm = MICRO_ALIGN_MAX_PWM;

                    if (align_error > 0.0f) {
                        TB6612_SetSpeed(-(int16_t)align_pwm, (int16_t)align_pwm);
                    } else {
                        TB6612_SetSpeed((int16_t)align_pwm, -(int16_t)align_pwm);
                    }
                } else {
                    TB6612_Brake();
                }
            } else {
                Motion_Stop();
                just_finished_turn = true;
                return;
            }
            return;
        }

        if (HAL_GetTick() - rotate_start_tick > ROTATE_TIMEOUT_MS) {
            Motion_Stop();
            just_finished_turn = true;
            return;
        }

        float turn_effort = PID_Compute(&pid_gyro, target_angle, sf->fused_angle, dt);
        float turn_pwm = fabsf(turn_effort);
        float max_rot = is_fast_mode ? FAST_ROTATE_MAX_PWM : EXPLORE_ROTATE_MAX_PWM;

        if (turn_pwm < MIN_ROTATE_PWM) turn_pwm = MIN_ROTATE_PWM;
        if (turn_pwm > max_rot)        turn_pwm = max_rot;

        if (is_turn_left) {
            TB6612_SetSpeed(-(int16_t)turn_pwm, (int16_t)turn_pwm);
        } else {
            TB6612_SetSpeed((int16_t)turn_pwm, -(int16_t)turn_pwm);
        }
    }
}