#ifndef MAZE_SOLVER_H
#define MAZE_SOLVER_H

#include "main.h"
#include "sensor_fusion.h"
#include "flash.h"
#include "bt_debug.h"
#include <stdbool.h>

#define MAZE_SIZE 16

typedef enum { 
    DIR_NORTH = 0, 
    DIR_EAST  = 1, 
    DIR_SOUTH = 2, 
    DIR_WEST  = 3 
} Direction;

typedef enum { 
    ACTION_STOP = 0, 
    ACTION_FORWARD, 
    ACTION_TURN_LEFT, 
    ACTION_TURN_RIGHT, 
    ACTION_TURN_AROUND 
} Action;

/* Trang thai tuong 2-bit Peter Harrison */
typedef enum {
    WALL_EXIT    = 0, // 00b: Khong co tuong (duong thong)
    WALL_PRESENT = 1, // 01b: Co tuong
    WALL_UNKNOWN = 2  // 10b: Chua tung nhin thay
} WallState;

/* Mat na bit chuyen doi che do Do duong va Fast Run */
typedef enum {
    MASK_OPEN   = 0x01, // Do duong: UNKNOWN xem nhu duong thong de hut xe kham pha
    MASK_CLOSED = 0x03  // Fast Run: UNKNOWN xem nhu tuong kin an toan tuyet doi
} MazeMask;

typedef struct {
    uint8_t north : 2;
    uint8_t east  : 2;
    uint8_t south : 2;
    uint8_t west  : 2;
} Cell;

typedef struct {
    uint8_t x;
    uint8_t y;
    Direction dir;
} RobotPosition;

// Khoi tao me cung va dat vi tri robot ve (0,0) huong Bac
void Maze_Init(void);

// Quan ly mat na bit
void Maze_SetMask(MazeMask mask);
MazeMask Maze_GetMask(void);

// Cap nhat tuong tu cam bien va tu dong tinh lai Floodfill neu co tuong moi
void Maze_UpdateWalls(SensorFusion_Data *sf);

// Tinh toan ma tran khoang cach Floodfill dua theo muc tieu hien tai
void Maze_ComputeFloodfill(void);

// Lay hanh dong tiep theo dua tren ma tran Floodfill
Action Maze_GetNextAction(void);

// Cap nhat toa do va huong cua robot sau khi hoan tat buoc di
void Maze_UpdatePosition(Action act);

// Kiem tra da den dich hay chua (tuy thuoc vao che do tim tam hay quay ve)
bool Maze_IsGoalReached(void);
bool Maze_IsAtStart(void);

// Chon muc tieu cho Floodfill: true = Ve tam (7,7)-(8,8); false = Quay ve (0,0)
void Maze_SetTargetGoal(bool to_center);

// Lay thong tin vi tri va huong hien tai cua robot
RobotPosition Maze_GetPosition(void);

// Luu / Doc ma tran tuong vao Flash phuc vu luot chay toc do (Fast Run)
bool Maze_SaveToFlash(void);
bool Maze_LoadFromFlash(void);

// Nhin truoc lo trinh dem so o chay thang lien tiep
uint8_t Maze_GetConsecutiveForwardSteps(void);

// Nhin truoc hanh dong ke tiep sau khi di het forward_steps o thang (phuc vu Smooth Turn)
Action Maze_PeekNextActionAfterForward(uint8_t forward_steps);

// Thuat toan Flood Fill 3D phat goc cua de tim duong chay thang toi uu cho Fast Run
void Maze_ComputeTurnPenaltyFloodfill(void);

// [16x16]: Mo thong 4 vach noi bo khoang dich 2x2 trung tam (7,7)-(8,8)
void Maze_OpenCenterGoalBox(void);

// [16x16]: Kiem tra lo trinh da kham pha da dat chi phi ngan nhat ly tuong chua
bool Maze_IsOptimalPathFound(void);

#endif /* MAZE_SOLVER_H */