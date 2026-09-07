#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "sensor_fusion.h"
#include "tb6612fng.h"
#include "encoder.h"
#include <stdbool.h>

typedef enum {
    MOTION_STATE_IDLE,
    MOTION_STATE_FORWARD,
    MOTION_STATE_ROTATE
} MotionState;

// Khoi tao Motion Controller va PID
void Motion_Init(void);

// Cap nhat vong dieu khien chuyen dong (Goi chu ky dinh ky trong ISR/Loop)
void Motion_Update(float dt);

// Yeu cau Robot tien mot khoang cach (mm)
void Motion_MoveForward(float distance_mm);

// Yeu cau Robot quay mot goc (degree: duong la quay trái, am la quay phai)
void Motion_Rotate(float angle_deg);

// Dung Robot ngay lap tuc
void Motion_Stop(void);

// Kiem tra Robot da hoan thanh hanh dong hay chua
bool Motion_IsFinished(void);

#endif /* MOTION_CONTROLLER_H */