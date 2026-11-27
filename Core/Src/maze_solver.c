/**
  ******************************************************************************
  * @file           : maze_solver.c
  * @brief          : Giai thuat Flood Fill Peter Harrison (Bitmask 2-bit & MASK_OPEN/CLOSED)
  *                   Tich hop 3D Turn-Penalty toi uu cho Smooth Turn 90 do (16x16)
  *                   Vung dich trung tam 4 o: (7,7), (7,8), (8,7), (8,8)
  ******************************************************************************
  */

#include "maze_solver.h"
#include "config.h"
#include "sensor_fusion.h"
#include <string.h>

/* Toa do xuat phat tai goc duoi trai (0,0) */
#define START_X             0
#define START_Y             0

/* Gioi han sa ban thi dau tieu chuan 16x16 (toa do tu 0 den 15) */
#define ACTIVE_MAZE_LIMIT   16

// ==============================================================
// TRONG SO CHO THUAT TOAN PHAT GOC CUA (3D TURN-PENALTY - SMOOTH TURN)
// ==============================================================
#define COST_STRAIGHT       10      // Chi phi di thang 1 o (180mm)
#define PENALTY_TURN        22      // Phat re muot 90 do (Smooth Turn khong dung lai)
#define PENALTY_UTURN       60      // Phat quay dau 180 do

/* Bien toan cuc quan ly trang thai me cung */
static Cell maze[MAZE_SIZE][MAZE_SIZE];
static uint8_t dist_map[MAZE_SIZE][MAZE_SIZE];
static RobotPosition robot = {START_X, START_Y, DIR_NORTH};
static bool target_is_center = true;
static MazeMask current_mask = MASK_OPEN; // Mac dinh la MASK_OPEN de do duong

// Ma tran chi phi 3D: cost_map[x][y][huong_mui_xe]
static uint16_t cost_map[MAZE_SIZE][MAZE_SIZE][4];
static bool use_turn_penalty = false;

// Hang doi 2D cho Floodfill BFS
typedef struct {
    uint8_t x, y;
} QueueNode;
static QueueNode queue[MAZE_SIZE * MAZE_SIZE];

// Hang doi 3D cho SPFA phat goc cua
typedef struct {
    uint8_t x;
    uint8_t y;
    uint8_t dir;
} State3D;

#define Q3D_SIZE 2048
static State3D q3d[Q3D_SIZE];
static uint16_t q3d_head = 0;
static uint16_t q3d_tail = 0;
static bool in_q3d[MAZE_SIZE][MAZE_SIZE][4];

// Vector toa do 4 huong: 0:BAC, 1:DONG, 2:NAM, 3:TAY
static const int8_t DX[4] = {0, 1, 0, -1};
static const int8_t DY[4] = {1, 0, -1, 0};

/* Kiem tra toa do (x, y) co thuoc cum 4 o dich hay khong */
static inline bool IsCenterGoalCell(uint8_t x, uint8_t y) {
#if (ACTIVE_MAZE_LIMIT == 16)
    return (x >= 7 && x <= 8 && y >= 7 && y <= 8);
#else
    return (x >= 3 && x <= 4 && y <= 1);
#endif
}

// ==============================================================
// LOGIC BITMASK PETER HARRISON: KIEM TRA LOI THOAT / CO TUONG
// ==============================================================

static inline bool IsExit(uint8_t x, uint8_t y, Direction dir) {
    if (x >= ACTIVE_MAZE_LIMIT || y >= ACTIVE_MAZE_LIMIT) return false;

    // Tuong bao bien ngoai sa ban
    if (dir == DIR_NORTH && y >= ACTIVE_MAZE_LIMIT - 1) return false;
    if (dir == DIR_EAST  && x >= ACTIVE_MAZE_LIMIT - 1) return false;
    if (dir == DIR_SOUTH && y == 0) return false;
    if (dir == DIR_WEST  && x == 0) return false;

    WallState ws = WALL_UNKNOWN;
    if (dir == DIR_NORTH)      ws = (WallState)maze[x][y].north;
    else if (dir == DIR_EAST)  ws = (WallState)maze[x][y].east;
    else if (dir == DIR_SOUTH) ws = (WallState)maze[x][y].south;
    else if (dir == DIR_WEST)  ws = (WallState)maze[x][y].west;

    return ((ws & current_mask) == WALL_EXIT);
}

static inline bool HasWall(uint8_t x, uint8_t y, Direction dir) {
    return !IsExit(x, y, dir);
}

static inline bool Maze_CellIsVisited(uint8_t x, uint8_t y) {
    return (maze[x][y].north != WALL_UNKNOWN &&
            maze[x][y].east  != WALL_UNKNOWN &&
            maze[x][y].south != WALL_UNKNOWN &&
            maze[x][y].west  != WALL_UNKNOWN);
}

/* Quan ly hang doi 3D vong tron */
static inline void Q3D_Push(uint8_t x, uint8_t y, uint8_t dir) {
    if (!in_q3d[x][y][dir]) {
        uint16_t next_tail = (q3d_tail + 1) % Q3D_SIZE;
        if (next_tail != q3d_head) {
            q3d[q3d_tail].x = x;
            q3d[q3d_tail].y = y;
            q3d[q3d_tail].dir = dir;
            q3d_tail = next_tail;
            in_q3d[x][y][dir] = true;
        }
    }
}

static inline State3D Q3D_Pop(void) {
    State3D s = q3d[q3d_head];
    q3d_head = (q3d_head + 1) % Q3D_SIZE;
    in_q3d[s.x][s.y][s.dir] = false;
    return s;
}

static inline bool Q3D_IsEmpty(void) {
    return (q3d_head == q3d_tail);
}

/**
  * @brief  Dat trang thai tuong vo dieu kien cho ca 2 o lien ke
  */
static void SetWallState(uint8_t x, uint8_t y, Direction dir, WallState state) {
    if (dir == DIR_NORTH) {
        maze[x][y].north = state;
        if (y < ACTIVE_MAZE_LIMIT - 1) maze[x][y + 1].south = state;
    } else if (dir == DIR_EAST) {
        maze[x][y].east = state;
        if (x < ACTIVE_MAZE_LIMIT - 1) maze[x + 1][y].west = state;
    } else if (dir == DIR_SOUTH) {
        maze[x][y].south = state;
        if (y > 0) maze[x][y - 1].north = state;
    } else if (dir == DIR_WEST) {
        maze[x][y].west = state;
        if (x > 0) maze[x - 1][y].east = state;
    }
}

/**
  * @brief  Quy tac bat bien Peter Harrison: Chi cap nhat neu tuong do dang la UNKNOWN
  */
static bool UpdateWallState(uint8_t x, uint8_t y, Direction dir, WallState state) {
    if ((x == 0 && y == 0 && dir == DIR_EAST) || (x == 1 && y == 0 && dir == DIR_WEST)) {
        return false;
    }

    WallState current_state = WALL_UNKNOWN;
    if (dir == DIR_NORTH)      current_state = (WallState)maze[x][y].north;
    else if (dir == DIR_EAST)  current_state = (WallState)maze[x][y].east;
    else if (dir == DIR_SOUTH) current_state = (WallState)maze[x][y].south;
    else if (dir == DIR_WEST)  current_state = (WallState)maze[x][y].west;

    if (current_state == WALL_UNKNOWN) {
        SetWallState(x, y, dir, state);
        return true;
    }
    return false;
}

void Maze_OpenCenterGoalBox(void) {
#if (ACTIVE_MAZE_LIMIT == 16)
    SetWallState(7, 7, DIR_NORTH, WALL_EXIT);
    SetWallState(7, 7, DIR_EAST,  WALL_EXIT);
    SetWallState(8, 8, DIR_SOUTH, WALL_EXIT);
    SetWallState(8, 8, DIR_WEST,  WALL_EXIT);
#endif
}

void Maze_Init(void) {
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            maze[x][y].north = WALL_UNKNOWN;
            maze[x][y].east  = WALL_UNKNOWN;
            maze[x][y].south = WALL_UNKNOWN;
            maze[x][y].west  = WALL_UNKNOWN;
        }
    }

    robot.x = START_X;
    robot.y = START_Y;
    robot.dir = DIR_NORTH;
    target_is_center = true;
    use_turn_penalty = false;
    current_mask = MASK_OPEN;

    for (int i = 0; i < ACTIVE_MAZE_LIMIT; i++) {
        SetWallState(i, 0, DIR_SOUTH, WALL_PRESENT);
        SetWallState(i, ACTIVE_MAZE_LIMIT - 1, DIR_NORTH, WALL_PRESENT);
        SetWallState(0, i, DIR_WEST, WALL_PRESENT);
        SetWallState(ACTIVE_MAZE_LIMIT - 1, i, DIR_EAST, WALL_PRESENT);
    }

    SetWallState(0, 0, DIR_EAST, WALL_PRESENT);

    Maze_ComputeFloodfill();
}

void Maze_SetMask(MazeMask mask) {
    current_mask = mask;
}

MazeMask Maze_GetMask(void) {
    return current_mask;
}

static void ClearWall(uint8_t x, uint8_t y, Direction dir) {
    if (dir == DIR_NORTH && y >= ACTIVE_MAZE_LIMIT - 1) return;
    if (dir == DIR_EAST  && x >= ACTIVE_MAZE_LIMIT - 1) return;
    if (dir == DIR_SOUTH && y == 0) return;
    if (dir == DIR_WEST  && x == 0) return;

    if ((x == 0 && y == 0 && dir == DIR_EAST) || (x == 1 && y == 0 && dir == DIR_WEST)) return;

    SetWallState(x, y, dir, WALL_EXIT);
}

void Maze_UpdateWalls(SensorFusion_Data *sf) {
    uint8_t x = robot.x, y = robot.y;
    Direction front_dir = robot.dir;
    Direction left_dir  = (Direction)((robot.dir + 3) % 4);
    Direction right_dir = (Direction)((robot.dir + 1) % 4);

    bool wall_changed = false;

    WallState st_front = sf->wall_front ? WALL_PRESENT : WALL_EXIT;
    if (UpdateWallState(x, y, front_dir, st_front)) wall_changed = true;

    WallState st_left  = sf->wall_left ? WALL_PRESENT : WALL_EXIT;
    if (UpdateWallState(x, y, left_dir, st_left)) wall_changed = true;

    WallState st_right = sf->wall_right ? WALL_PRESENT : WALL_EXIT;
    if (UpdateWallState(x, y, right_dir, st_right)) wall_changed = true;

    if (IsCenterGoalCell(x, y)) {
        Maze_OpenCenterGoalBox();
        wall_changed = true;
    }

    if (wall_changed) {
        Maze_ComputeFloodfill();
    }
}

void Maze_SetTargetGoal(bool to_center) {
    target_is_center = to_center;
    use_turn_penalty = false;
    current_mask = MASK_OPEN;
    Maze_ComputeFloodfill();
}

void Maze_ComputeFloodfill(void) {
    use_turn_penalty = false;
    current_mask = MASK_OPEN;
    memset(dist_map, 255, sizeof(dist_map));
    int head = 0, tail = 0;

    if (target_is_center) {
#if (ACTIVE_MAZE_LIMIT == 16)
        uint8_t goals[4][2] = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
#else
        uint8_t goals[4][2] = {{3, 0}, {3, 1}, {4, 0}, {4, 1}};
#endif
        for (int i = 0; i < 4; i++) {
            uint8_t gx = goals[i][0], gy = goals[i][1];
            if (gx < ACTIVE_MAZE_LIMIT && gy < ACTIVE_MAZE_LIMIT) {
                dist_map[gx][gy] = 0;
                queue[tail++] = (QueueNode){gx, gy};
            }
        }
    } else {
        dist_map[START_X][START_Y] = 0;
        queue[tail++] = (QueueNode){START_X, START_Y};
    }

    while (head < tail) {
        QueueNode curr = queue[head++];
        uint8_t x = curr.x, y = curr.y;
        uint8_t current_dist = dist_map[x][y];

        for (int d = 0; d < 4; d++) {
            if (IsExit(x, y, (Direction)d)) {
                int nx = x + DX[d];
                int ny = y + DY[d];
                if (nx >= 0 && nx < ACTIVE_MAZE_LIMIT && ny >= 0 && ny < ACTIVE_MAZE_LIMIT) {
                    if (dist_map[nx][ny] > current_dist + 1) {
                        dist_map[nx][ny] = current_dist + 1;
                        queue[tail++] = (QueueNode){(uint8_t)nx, (uint8_t)ny};
                    }
                }
            }
        }
    }
}

static void RunTurnPenaltySPFA(void) {
    memset(cost_map, 0xFF, sizeof(cost_map));
    memset(in_q3d, 0, sizeof(in_q3d));
    q3d_head = 0;
    q3d_tail = 0;

    if (target_is_center) {
#if (ACTIVE_MAZE_LIMIT == 16)
        uint8_t goals[4][2] = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
#else
        uint8_t goals[4][2] = {{3, 0}, {3, 1}, {4, 0}, {4, 1}};
#endif
        for (int i = 0; i < 4; i++) {
            uint8_t gx = goals[i][0], gy = goals[i][1];
            if (gx < ACTIVE_MAZE_LIMIT && gy < ACTIVE_MAZE_LIMIT) {
                for (uint8_t d = 0; d < 4; d++) {
                    cost_map[gx][gy][d] = 0;
                    Q3D_Push(gx, gy, d);
                }
            }
        }
    } else {
        for (uint8_t d = 0; d < 4; d++) {
            cost_map[START_X][START_Y][d] = 0;
            Q3D_Push(START_X, START_Y, d);
        }
    }

    while (!Q3D_IsEmpty()) {
        State3D curr = Q3D_Pop();
        uint16_t c = cost_map[curr.x][curr.y][curr.dir];

        int px = curr.x - DX[curr.dir];
        int py = curr.y - DY[curr.dir];

        if (px >= 0 && px < ACTIVE_MAZE_LIMIT && py >= 0 && py < ACTIVE_MAZE_LIMIT) {
            if (IsExit((uint8_t)px, (uint8_t)py, (Direction)curr.dir)) {
                uint16_t new_cost = c + COST_STRAIGHT;
                if (new_cost < cost_map[px][py][curr.dir]) {
                    cost_map[px][py][curr.dir] = new_cost;
                    Q3D_Push((uint8_t)px, (uint8_t)py, curr.dir);
                }
            }
        }

        for (uint8_t pdir = 0; pdir < 4; pdir++) {
            if (pdir == curr.dir) continue;

            uint8_t diff = (curr.dir - pdir + 4) % 4;
            uint16_t penalty = (diff == 2) ? PENALTY_UTURN : PENALTY_TURN;
            uint16_t new_cost = c + penalty;

            if (new_cost < cost_map[curr.x][curr.y][pdir]) {
                cost_map[curr.x][curr.y][pdir] = new_cost;
                Q3D_Push(curr.x, curr.y, pdir);
            }
        }
    }
}

void Maze_ComputeTurnPenaltyFloodfill(void) {
    current_mask = MASK_CLOSED;
    RunTurnPenaltySPFA();
    use_turn_penalty = true;
}

bool Maze_IsOptimalPathFound(void) {
    bool saved_target  = target_is_center;
    MazeMask saved_mask = current_mask;
    bool saved_penalty = use_turn_penalty;

    target_is_center = true;
    current_mask = MASK_OPEN;
    RunTurnPenaltySPFA();
    uint16_t optimistic_cost = cost_map[START_X][START_Y][DIR_NORTH];

    current_mask = MASK_CLOSED;
    RunTurnPenaltySPFA();
    uint16_t confirmed_cost = cost_map[START_X][START_Y][DIR_NORTH];

    target_is_center = saved_target;
    current_mask = saved_mask;
    if (saved_penalty) {
        Maze_ComputeTurnPenaltyFloodfill();
    } else {
        Maze_ComputeFloodfill();
    }

    return (confirmed_cost != 0xFFFF && confirmed_cost <= optimistic_cost);
}

static Action GetBestActionAtState(uint8_t x, uint8_t y, Direction d) {
    if (target_is_center) {
        if (IsCenterGoalCell(x, y)) return ACTION_STOP;
    } else {
        if (x == START_X && y == START_Y) return ACTION_STOP;
    }

    if (use_turn_penalty) {
        uint32_t min_cost = 0xFFFFFFFF;
        Action best_action = ACTION_STOP;

        if (IsExit(x, y, d)) {
            int nx = x + DX[d];
            int ny = y + DY[d];
            uint32_t c_fwd = (uint32_t)cost_map[nx][ny][d] + COST_STRAIGHT;
            min_cost = c_fwd;
            best_action = ACTION_FORWARD;
        }

        Direction r_dir = (Direction)((d + 1) % 4);
        uint32_t c_right = (uint32_t)cost_map[x][y][r_dir] + PENALTY_TURN;
        if (c_right < min_cost) {
            min_cost = c_right;
            best_action = ACTION_TURN_RIGHT;
        }

        Direction l_dir = (Direction)((d + 3) % 4);
        uint32_t c_left = (uint32_t)cost_map[x][y][l_dir] + PENALTY_TURN;
        if (c_left < min_cost) {
            min_cost = c_left;
            best_action = ACTION_TURN_LEFT;
        }

        Direction b_dir = (Direction)((d + 2) % 4);
        uint32_t c_back = (uint32_t)cost_map[x][y][b_dir] + PENALTY_UTURN;
        if (c_back < min_cost) {
            min_cost = c_back;
            best_action = ACTION_TURN_AROUND;
        }

        return best_action;
    }

    uint8_t min_dist = 255;
    Direction best_dir = d;
    bool found_move = false;

    Direction test_dirs[4] = {
        d,
        (Direction)((d + 1) % 4),
        (Direction)((d + 3) % 4),
        (Direction)((d + 2) % 4)
    };

    for (int i = 0; i < 4; i++) {
        Direction td = test_dirs[i];
        if (!IsExit(x, y, td)) continue;
        int nx = x + DX[td];
        int ny = y + DY[td];
        uint8_t nd = dist_map[nx][ny];
        if (!found_move || nd < min_dist) {
            min_dist = nd;
            best_dir = td;
            found_move = true;
        }
    }

    if (!found_move) return ACTION_TURN_AROUND;
    int dir_diff = (best_dir - d + 4) % 4;
    if (dir_diff == 0) return ACTION_FORWARD;
    if (dir_diff == 1) return ACTION_TURN_RIGHT;
    if (dir_diff == 2) return ACTION_TURN_AROUND;
    if (dir_diff == 3) return ACTION_TURN_LEFT;
    return ACTION_STOP;
}

Action Maze_GetNextAction(void) {
    if (use_turn_penalty) {
        return GetBestActionAtState(robot.x, robot.y, robot.dir);
    }

    uint8_t x = robot.x, y = robot.y;

    if (dist_map[x][y] == 0) return ACTION_STOP;

    if (dist_map[x][y] == 255) {
        SensorFusion_Data *sf = SensorFusion_GetData();
        Direction left_d  = (Direction)((robot.dir + 3) % 4);
        Direction right_d = (Direction)((robot.dir + 1) % 4);

        if (sf->front_dist > 165.0f)                   ClearWall(x, y, robot.dir);
        if (sf->diag_l >= WALL_OPENING_HIGH_THRESHOLD) ClearWall(x, y, left_d);
        if (sf->diag_r >= WALL_OPENING_HIGH_THRESHOLD) ClearWall(x, y, right_d);
        Maze_ComputeFloodfill();

        if (dist_map[x][y] == 255) {
            Direction back_d = (Direction)((robot.dir + 2) % 4);
            ClearWall(x, y, back_d);
            Maze_ComputeFloodfill();
        }
    }

    uint8_t min_dist = 255;
    Direction best_dir = robot.dir;
    bool found_move = false;

    Direction test_dirs[4] = {
        robot.dir,
        (Direction)((robot.dir + 1) % 4),
        (Direction)((robot.dir + 3) % 4),
        (Direction)((robot.dir + 2) % 4)
    };

    for (int i = 0; i < 4; i++) {
        Direction d = test_dirs[i];
        if (!IsExit(x, y, d)) continue;

        int nx = x + DX[d];
        int ny = y + DY[d];
        uint8_t neighbor_dist = dist_map[nx][ny];

        if (!found_move || neighbor_dist < min_dist) {
            min_dist = neighbor_dist;
            best_dir = d;
            found_move = true;
        }
        else if (neighbor_dist == min_dist && found_move) {
            int bx = x + DX[best_dir];
            int by = y + DY[best_dir];

            bool best_is_visited = Maze_CellIsVisited(bx, by);
            bool neighbor_is_visited = Maze_CellIsVisited(nx, ny);

            if (best_is_visited && !neighbor_is_visited) {
                best_dir = d;
            }
        }
    }

    if (!found_move) {
        Direction back_dir = (Direction)((robot.dir + 2) % 4);
        ClearWall(x, y, back_dir);
        Maze_ComputeFloodfill();
        return ACTION_TURN_AROUND;
    }

    int dir_diff = (best_dir - robot.dir + 4) % 4;

    if (dir_diff == 0) return ACTION_FORWARD;
    if (dir_diff == 1) return ACTION_TURN_RIGHT;
    if (dir_diff == 2) return ACTION_TURN_AROUND;
    if (dir_diff == 3) return ACTION_TURN_LEFT;

    return ACTION_STOP;
}

uint8_t Maze_GetConsecutiveForwardSteps(void) {
    uint8_t cur_x = robot.x;
    uint8_t cur_y = robot.y;
    Direction cur_dir = robot.dir;
    uint8_t steps = 0;

    while (steps < (ACTIVE_MAZE_LIMIT - 1)) {
        if (target_is_center) {
            if (IsCenterGoalCell(cur_x, cur_y)) break;
        } else {
            if (cur_x == START_X && cur_y == START_Y) break;
        }

        if (HasWall(cur_x, cur_y, cur_dir)) break;

        int nx = cur_x + DX[cur_dir];
        int ny = cur_y + DY[cur_dir];
        if (nx < 0 || nx >= ACTIVE_MAZE_LIMIT || ny < 0 || ny >= ACTIVE_MAZE_LIMIT) break;

        if (use_turn_penalty) {
            uint32_t c_fwd = (uint32_t)cost_map[nx][ny][cur_dir] + COST_STRAIGHT;
            Direction l_dir = (Direction)((cur_dir + 3) % 4);
            Direction r_dir = (Direction)((cur_dir + 1) % 4);
            uint32_t c_left = (uint32_t)cost_map[cur_x][cur_y][l_dir] + PENALTY_TURN;
            uint32_t c_right = (uint32_t)cost_map[cur_x][cur_y][r_dir] + PENALTY_TURN;

            if (c_fwd <= c_left && c_fwd <= c_right) {
                steps++;
                cur_x = (uint8_t)nx;
                cur_y = (uint8_t)ny;
            } else {
                break;
            }
        } else {
            if (dist_map[nx][ny] == dist_map[cur_x][cur_y] - 1) {
                steps++;
                cur_x = (uint8_t)nx;
                cur_y = (uint8_t)ny;
            } else {
                break;
            }
        }
    }

    return (steps == 0) ? 1 : steps;
}

Action Maze_PeekNextActionAfterForward(uint8_t forward_steps) {
    uint8_t cur_x = robot.x;
    uint8_t cur_y = robot.y;
    Direction cur_dir = robot.dir;

    for (uint8_t i = 0; i < forward_steps; i++) {
        if (HasWall(cur_x, cur_y, cur_dir)) break;
        int nx = cur_x + DX[cur_dir];
        int ny = cur_y + DY[cur_dir];
        if (nx < 0 || nx >= ACTIVE_MAZE_LIMIT || ny < 0 || ny >= ACTIVE_MAZE_LIMIT) break;
        cur_x = (uint8_t)nx;
        cur_y = (uint8_t)ny;
    }

    return GetBestActionAtState(cur_x, cur_y, cur_dir);
}

void Maze_UpdatePosition(Action act) {
    if (act == ACTION_TURN_LEFT)   robot.dir = (Direction)((robot.dir + 3) % 4);
    if (act == ACTION_TURN_RIGHT)  robot.dir = (Direction)((robot.dir + 1) % 4);
    if (act == ACTION_TURN_AROUND) robot.dir = (Direction)((robot.dir + 2) % 4);

    if (act == ACTION_FORWARD) {
        if (robot.dir == DIR_NORTH && robot.y < ACTIVE_MAZE_LIMIT - 1) robot.y++;
        if (robot.dir == DIR_EAST  && robot.x < ACTIVE_MAZE_LIMIT - 1) robot.x++;
        if (robot.dir == DIR_SOUTH && robot.y > 0)                     robot.y--;
        if (robot.dir == DIR_WEST  && robot.x > 0)                     robot.x--;
    }
}

bool Maze_IsGoalReached(void) {
    if (target_is_center) {
        return IsCenterGoalCell(robot.x, robot.y);
    }
    return (robot.x == START_X && robot.y == START_Y);
}

bool Maze_IsAtStart(void) {
    return (robot.x == START_X && robot.y == START_Y);
}

RobotPosition Maze_GetPosition(void) {
    return robot;
}

typedef struct {
    uint32_t header;
    Cell maze_data[MAZE_SIZE][MAZE_SIZE];
} FlashStorage;

bool Maze_SaveToFlash(void) {
    FlashStorage storage;
    storage.header = FLASH_MAGIC_HEADER;

    Maze_OpenCenterGoalBox();
    memcpy(storage.maze_data, maze, sizeof(maze));

    for (int x = 0; x < ACTIVE_MAZE_LIMIT; x++) {
        for (int y = 0; y < ACTIVE_MAZE_LIMIT; y++) {
            if (!IsCenterGoalCell((uint8_t)x, (uint8_t)y)) {
                if (storage.maze_data[x][y].north == WALL_UNKNOWN) storage.maze_data[x][y].north = WALL_PRESENT;
                if (storage.maze_data[x][y].east  == WALL_UNKNOWN) storage.maze_data[x][y].east  = WALL_PRESENT;
                if (storage.maze_data[x][y].south == WALL_UNKNOWN) storage.maze_data[x][y].south = WALL_PRESENT;
                if (storage.maze_data[x][y].west  == WALL_UNKNOWN) storage.maze_data[x][y].west  = WALL_PRESENT;
            }
        }
    }

    return (Flash_SaveData((uint8_t *)&storage, (uint16_t)sizeof(storage)) == HAL_OK);
}

bool Maze_LoadFromFlash(void) {
    FlashStorage storage;
    Flash_ReadData((uint8_t *)&storage, (uint16_t)sizeof(storage));

    if (storage.header != FLASH_MAGIC_HEADER) {
        LOG_ERR("Flash empty or invalid header!\r\n");
        Maze_Init();
        return false;
    }

    memcpy(maze, storage.maze_data, sizeof(maze));
    robot.x = START_X;
    robot.y = START_Y;
    robot.dir = DIR_NORTH;
    target_is_center = true;

    current_mask = MASK_CLOSED;

    Maze_ComputeTurnPenaltyFloodfill();
    return true;
}