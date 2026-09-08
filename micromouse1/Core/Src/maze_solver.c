#include "maze_solver.h"
#include <string.h>

static Cell maze[MAZE_SIZE][MAZE_SIZE];
static uint8_t dist_map[MAZE_SIZE][MAZE_SIZE];
static RobotPosition robot = {0, 0, DIR_NORTH};

typedef struct { uint8_t x, y; } QueueNode;
static QueueNode queue[MAZE_SIZE * MAZE_SIZE];

void Maze_Init(void) {
    memset(maze, 0, sizeof(maze));
    robot.x = 0;
    robot.y = 0;
    robot.dir = DIR_NORTH;

    for (int i = 0; i < MAZE_SIZE; i++) {
        maze[i][0].south = 1;
        maze[i][MAZE_SIZE - 1].north = 1;
        maze[0][i].west = 1;
        maze[MAZE_SIZE - 1][i].east = 1;
    }

    Maze_ComputeFloodfill();
}

void Maze_UpdateWalls(SensorFusion_Data *sf) {
    uint8_t x = robot.x, y = robot.y;
    maze[x][y].visited = 1;

    Direction left_dir  = (robot.dir + 3) % 4;
    Direction right_dir = (robot.dir + 1) % 4;
    Direction front_dir = robot.dir;

    if (sf->wall_front) {
        if (front_dir == DIR_NORTH) maze[x][y].north = 1;
        if (front_dir == DIR_EAST)  maze[x][y].east  = 1;
        if (front_dir == DIR_SOUTH) maze[x][y].south = 1;
        if (front_dir == DIR_WEST)  maze[x][y].west  = 1;
    }
    if (sf->wall_left) {
        if (left_dir == DIR_NORTH) maze[x][y].north = 1;
        if (left_dir == DIR_EAST)  maze[x][y].east  = 1;
        if (left_dir == DIR_SOUTH) maze[x][y].south = 1;
        if (left_dir == DIR_WEST)  maze[x][y].west  = 1;
    }
    if (sf->wall_right) {
        if (right_dir == DIR_NORTH) maze[x][y].north = 1;
        if (right_dir == DIR_EAST)  maze[x][y].east  = 1;
        if (right_dir == DIR_SOUTH) maze[x][y].south = 1;
        if (right_dir == DIR_WEST)  maze[x][y].west  = 1;
    }
}

void Maze_ComputeFloodfill(void) {
    memset(dist_map, 255, sizeof(dist_map));
    int head = 0, tail = 0;

    uint8_t goals[4][2] = {{7,7}, {7,8}, {8,7}, {8,8}};
    for (int i = 0; i < 4; i++) {
        uint8_t gx = goals[i][0], gy = goals[i][1];
        dist_map[gx][gy] = 0;
        queue[tail++] = (QueueNode){gx, gy};
    }

    while (head < tail) {
        QueueNode curr = queue[head++];
        uint8_t x = curr.x, y = curr.y;
        uint8_t current_dist = dist_map[x][y];

        if (!maze[x][y].north && y < MAZE_SIZE - 1 && dist_map[x][y + 1] == 255) {
            dist_map[x][y + 1] = current_dist + 1;
            queue[tail++] = (QueueNode){x, y + 1};
        }
        if (!maze[x][y].east && x < MAZE_SIZE - 1 && dist_map[x + 1][y] == 255) {
            dist_map[x + 1][y] = current_dist + 1;
            queue[tail++] = (QueueNode){x + 1, y};
        }
        if (!maze[x][y].south && y > 0 && dist_map[x][y - 1] == 255) {
            dist_map[x][y - 1] = current_dist + 1;
            queue[tail++] = (QueueNode){x, y - 1};
        }
        if (!maze[x][y].west && x > 0 && dist_map[x - 1][y] == 255) {
            dist_map[x - 1][y] = current_dist + 1;
            queue[tail++] = (QueueNode){x - 1, y};
        }
    }
}

Action Maze_GetNextAction(void) {
    uint8_t x = robot.x, y = robot.y;
    uint8_t min_dist = dist_map[x][y];
    Direction best_dir = robot.dir;

    if (!maze[x][y].north && y < MAZE_SIZE - 1 && dist_map[x][y + 1] < min_dist) {
        min_dist = dist_map[x][y + 1];
        best_dir = DIR_NORTH;
    }
    if (!maze[x][y].east && x < MAZE_SIZE - 1 && dist_map[x + 1][y] < min_dist) {
        min_dist = dist_map[x + 1][y];
        best_dir = DIR_EAST;
    }
    if (!maze[x][y].south && y > 0 && dist_map[x][y - 1] < min_dist) {
        min_dist = dist_map[x][y - 1];
        best_dir = DIR_SOUTH;
    }
    if (!maze[x][y].west && x > 0 && dist_map[x - 1][y] < min_dist) {
        min_dist = dist_map[x - 1][y];
        best_dir = DIR_WEST;
    }

    int dir_diff = (best_dir - robot.dir + 4) % 4;
    if (dir_diff == 0) return ACTION_FORWARD;
    if (dir_diff == 1) return ACTION_TURN_RIGHT;
    if (dir_diff == 2) return ACTION_TURN_AROUND;
    if (dir_diff == 3) return ACTION_TURN_LEFT;

    return ACTION_STOP;
}

void Maze_UpdatePosition(Action act) {
    if (act == ACTION_TURN_LEFT)   robot.dir = (robot.dir + 3) % 4;
    if (act == ACTION_TURN_RIGHT)  robot.dir = (robot.dir + 1) % 4;
    if (act == ACTION_TURN_AROUND) robot.dir = (robot.dir + 2) % 4;

    if (act == ACTION_FORWARD) {
        if (robot.dir == DIR_NORTH) robot.y++;
        if (robot.dir == DIR_EAST)  robot.x++;
        if (robot.dir == DIR_SOUTH) robot.y--;
        if (robot.dir == DIR_WEST)  robot.x--;
    }
}

bool Maze_IsGoalReached(void) {
    return (dist_map[robot.x][robot.y] == 0);
}

bool Maze_SaveToFlash(void) {
    return (Flash_SaveData((uint8_t *)maze, sizeof(maze)) == HAL_OK);
}

bool Maze_LoadFromFlash(void) {
    Flash_ReadData((uint8_t *)maze, sizeof(maze));
    Maze_ComputeFloodfill();
    return true;
}