#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "sensor_fusion.h"
#include "tb6612fng.h"
#include "encoder.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTION_STATE_IDLE,
    MOTION_STATE_FORWARD,          // Chay theo cu ly co dinh (Fast Run & lui/tien ngan)
    MOTION_STATE_CONTINUOUS,       // Chay thang lien tuc khong dung giua cac o (Explore / Return)
    MOTION_STATE_STOP_AT_CENTER,   // Ham toc muot de dung sat tuong / tam o chuan bi quay ngoat
    MOTION_STATE_ROTATE            // Quay ngoat toc do cao (Snap Pivot Turn + phanh xung nguoc)
} MotionState;

void Motion_Init(void);
void Motion_Update(float dt);

// Chay thang co dinh (Fast Run hoac tien ngan DOCK_RETURN_CENTER_MM)
void Motion_MoveForward(float distance_mm);

// Bat dau chay thang lien tuc qua nhieu o (Explore / Return)
void Motion_StartContinuousForward(void);

// Kiem tra xe da cham moc 135mm cua o moi de chot ban do & quyet dinh huong di
bool Motion_IsCellCheckpointReady(void);

// Lay ket qua bo phieu quet vach trai/phai trong cua so 15-78mm cua o moi
void Motion_GetCellWallVotes(bool *has_left, bool *has_right);

// Xac nhan di thang tiep sang o ke tiep (khong giam toc)
void Motion_ContinueNextCell(void);

// Ra lenh giam toc va dung sat tuong / tam o phia truoc de re hoac ty tuong
void Motion_CommitStopAtCenter(void);

// Quay ngoat nhanh sang huong can re (deg: duong = Re trai, am = Re phai/Quay dau)
void Motion_Rotate(float angle_deg);

void Motion_Stop(void);
bool Motion_IsFinished(void);

bool Motion_GetLatchedTurnRight(void);
bool Motion_GetLatchedTurnLeft(void);

void Motion_UpdatePIDWall(float kp, float ki, float kd);
void Motion_UpdatePIDGyro(float kp, float ki, float kd);

bool Motion_AlignHeadingStatic(void);
void Motion_SetFastMode(bool enable_fast);

void Motion_SoftReverse(uint16_t time_ms);
void Motion_ResetOdometryAtCenter(void);
void Motion_SnapAngleToCardinal(void);
void Motion_ResetEncodersAtCenter(void);

#endif /* MOTION_CONTROLLER_H */