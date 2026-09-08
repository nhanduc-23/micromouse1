#ifndef MAZE_SOLVER_H
#define MAZE_SOLVER_H

#include "main.h"
#include "sensor_fusion.h"
#include "flash.h"
#include <stdbool.h>

#define MAZE_SIZE 16

typedef enum { DIR_NORTH = 0, DIR_EAST, DIR_SOUTH, DIR_WEST } Direction;
typedef enum { ACTION_STOP = 0, ACTION_FORWARD, ACTION_TURN_LEFT, ACTION_TURN_RIGHT, ACTION_TURN_AROUND } Action;

typedef struct {
    uint8_t north : 1;
    uint8_t east  : 1;
    uint8_t south : 1;
    uint8_t west  : 1;
    uint8_t visited: 1;
} Cell;

typedef struct {
    uint8_t x;
    uint8_t y;
    Direction dir;
} RobotPosition;

void Maze_Init(void);
void Maze_UpdateWalls(SensorFusion_Data *sf);
void Maze_ComputeFloodfill(void);
Action Maze_GetNextAction(void);
void Maze_UpdatePosition(Action act);
bool Maze_IsGoalReached(void);

// Tich hop Flash persistence
bool Maze_SaveToFlash(void);
bool Maze_LoadFromFlash(void);

#endif /* MAZE_SOLVER_H */