/**
  ******************************************************************************
  * @file           : maze_solver.c
  * @brief          : Implement thuat toan Flood Fill & Dieu huong Micromouse
  ******************************************************************************
  */

#include "maze_solver.h"
#include <string.h>

/* Bien toan cuc noi bo */
static Cell maze[MAZE_SIZE][MAZE_SIZE];
static uint8_t dist_map[MAZE_SIZE][MAZE_SIZE];
static RobotPosition robot = {0, 0, DIR_NORTH};

typedef struct { 
    uint8_t x, y; 
} QueueNode;

static QueueNode queue[MAZE_SIZE * MAZE_SIZE];

/**
  * @brief  Khoi tao me cung va dat vi tri robot ve (0,0) huong BAC
  */
void Maze_Init(void) {
    memset(maze, 0, sizeof(maze));
    robot.x = 0;
    robot.y = 0;
    robot.dir = DIR_NORTH;

    // Khoi tao tuong bao quanh me cung 16x16
    for (int i = 0; i < MAZE_SIZE; i++) {
        maze[i][0].south = 1;
        maze[i][MAZE_SIZE - 1].north = 1;
        maze[0][i].west = 1;
        maze[MAZE_SIZE - 1][i].east = 1;
    }

    Maze_ComputeFloodfill();
}

/**
  * @brief  Ham ho tro cap nhat tuong hai chieu (doi xung giua 2 o ke nhau)
  */
static void SetWall(uint8_t x, uint8_t y, Direction dir) {
    if (dir == DIR_NORTH) {
        maze[x][y].north = 1;
        if (y < MAZE_SIZE - 1) maze[x][y + 1].south = 1;
    } else if (dir == DIR_EAST) {
        maze[x][y].east = 1;
        if (x < MAZE_SIZE - 1) maze[x + 1][y].west = 1;
    } else if (dir == DIR_SOUTH) {
        maze[x][y].south = 1;
        if (y > 0) maze[x][y - 1].north = 1;
    } else if (dir == DIR_WEST) {
        maze[x][y].west = 1;
        if (x > 0) maze[x - 1][y].east = 1;
    }
}

/**
  * @brief  Cap nhat tuong tu du lieu cam bien tai vi tri hien tai cua robot
  */
void Maze_UpdateWalls(SensorFusion_Data *sf) {
    uint8_t x = robot.x, y = robot.y;
    maze[x][y].visited = 1;

    Direction left_dir  = (Direction)((robot.dir + 3) % 4);
    Direction right_dir = (Direction)((robot.dir + 1) % 4);
    Direction front_dir = robot.dir;

    if (sf->wall_front) SetWall(x, y, front_dir);
    if (sf->wall_left)  SetWall(x, y, left_dir);
    if (sf->wall_right) SetWall(x, y, right_dir);
}

/**
  * @brief  Tinh toan bang khoang cach Flood Fill tu 4 o trung tam (7,7),(7,8),(8,7),(8,8)
  */
void Maze_ComputeFloodfill(void) {
    memset(dist_map, 255, sizeof(dist_map));
    int head = 0, tail = 0;

    // 4 o trung tam me cung 16x16
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

        // Huong Bac (NORTH)
        if (!maze[x][y].north && y < MAZE_SIZE - 1 && dist_map[x][y + 1] == 255) {
            dist_map[x][y + 1] = current_dist + 1;
            queue[tail++] = (QueueNode){x, y + 1};
        }

        // Huong Dong (EAST)
        if (!maze[x][y].east && x < MAZE_SIZE - 1 && dist_map[x + 1][y] == 255) {
            dist_map[x + 1][y] = current_dist + 1;
            queue[tail++] = (QueueNode){x + 1, y};
        }

        // Huong Nam (SOUTH)
        if (!maze[x][y].south && y > 0 && dist_map[x][y - 1] == 255) {
            dist_map[x][y - 1] = current_dist + 1;
            queue[tail++] = (QueueNode){x, y - 1};
        }

        // Huong Tay (WEST)
        if (!maze[x][y].west && x > 0 && dist_map[x - 1][y] == 255) {
            dist_map[x - 1][y] = current_dist + 1;
            queue[tail++] = (QueueNode){x - 1, y};
        }
    }
}

/**
  * @brief  Xac dinh hanh dong tiep theo dua tren bang Flood Fill
  *         Uu tien goc quay (Di thang > Re phai > Re trai > Quay dau)
  *         va uu tien o chua kham pha (unvisited).
  */
Action Maze_GetNextAction(void) {
    uint8_t x = robot.x, y = robot.y;
    uint8_t current_dist = dist_map[x][y];

    // Neu da toi dich (khoang cach = 0) thi dung lai
    if (current_dist == 0) return ACTION_STOP;

    uint8_t min_dist = 255;
    Direction best_dir = robot.dir;
    bool found_move = false;

    // Thu tu kiem tra uu tien tuong doi so voi huong robot hien tai
    Direction test_dirs[4] = {
        robot.dir,
        (Direction)((robot.dir + 1) % 4),
        (Direction)((robot.dir + 3) % 4),
        (Direction)((robot.dir + 2) % 4)
    };

    for (int i = 0; i < 4; i++) {
        Direction d = test_dirs[i];
        int nx = x, ny = y;
        bool has_wall = false;

        if (d == DIR_NORTH)      { ny = y + 1; has_wall = maze[x][y].north || (y >= MAZE_SIZE - 1); }
        else if (d == DIR_EAST)  { nx = x + 1; has_wall = maze[x][y].east  || (x >= MAZE_SIZE - 1); }
        else if (d == DIR_SOUTH) { ny = y - 1; has_wall = maze[x][y].south || (y == 0); }
        else if (d == DIR_WEST)  { nx = x - 1; has_wall = maze[x][y].west  || (x == 0); }

        if (has_wall) continue;

        uint8_t neighbor_dist = dist_map[nx][ny];

        // Truong hop 1: Tim duoc o co khoang cach nho hon han
        if (neighbor_dist < min_dist) {
            min_dist = neighbor_dist;
            best_dir = d;
            found_move = true;
        } 
        // Truong hop 2: Khoang cach bang nhau, uu tien o CHUA EXPLORE (visited == 0)
        else if (neighbor_dist == min_dist && found_move) {
            int bx = x, by = y;
            if (best_dir == DIR_NORTH) by = y + 1;
            else if (best_dir == DIR_EAST) bx = x + 1;
            else if (best_dir == DIR_SOUTH) by = y - 1;
            else if (best_dir == DIR_WEST) bx = x - 1;

            bool best_is_visited = maze[bx][by].visited;
            bool neighbor_is_visited = maze[nx][ny].visited;

            if (best_is_visited && !neighbor_is_visited) {
                best_dir = d;
            }
        }
    }

    if (!found_move) return ACTION_STOP;

    int dir_diff = (best_dir - robot.dir + 4) % 4;

    if (dir_diff == 0) return ACTION_FORWARD;
    if (dir_diff == 1) return ACTION_TURN_RIGHT;
    if (dir_diff == 2) return ACTION_TURN_AROUND;
    if (dir_diff == 3) return ACTION_TURN_LEFT;

    return ACTION_STOP;
}

/**
  * @brief  Cap nhat vi tri va huong cua robot sau khi thuc hien hanh dong
  */
void Maze_UpdatePosition(Action act) {
    if (act == ACTION_TURN_LEFT)   robot.dir = (Direction)((robot.dir + 3) % 4);
    if (act == ACTION_TURN_RIGHT)  robot.dir = (Direction)((robot.dir + 1) % 4);
    if (act == ACTION_TURN_AROUND) robot.dir = (Direction)((robot.dir + 2) % 4);

    if (act == ACTION_FORWARD) {
        if (robot.dir == DIR_NORTH) robot.y++;
        if (robot.dir == DIR_EAST)  robot.x++;
        if (robot.dir == DIR_SOUTH) robot.y--;
        if (robot.dir == DIR_WEST)  robot.x--;
    }
}

/**
  * @brief  Kiem tra robot da cham vao o dich hay chua
  */
bool Maze_IsGoalReached(void) {
    return (dist_map[robot.x][robot.y] == 0);
}

/**
  * @brief  Luu ma tran tuong vao bo nho Flash
  */
// Struct dong goi kem Header kiem tra
typedef struct {
    uint32_t header;
    Cell maze_data[MAZE_SIZE][MAZE_SIZE];
} FlashStorage;
/**
  * @brief  Luu ma tran tuong kem Header vao Flash
  */
bool Maze_SaveToFlash(void) {
    FlashStorage storage;
    storage.header = FLASH_MAGIC_HEADER; //Gan ma Header hop le
    memcpy(storage.maze_data, maze, sizeof(maze));
    return (Flash_SaveData((uint8_t *)&storage, sizeof(storage)) == HAL_OK);
}

/**
  * @brief  Doc ma tran tuong tu Flash va tinh lai Flood Fill cho Fast Run
  */
bool Maze_LoadFromFlash(void) {
		FlashStorage storage;
    Flash_ReadData((uint8_t *)&storage, sizeof(storage));
    
	//Kiem tra Header truoc khi nap du lieu
    if (storage.header != FLASH_MAGIC_HEADER) {
        LOG_ERR("Flash empty or invalid header!\r\n");
        Maze_Init(); // Neu Flash rong, quay ve khoi tao me cung mac dinh
        return false;
    }
 //Neu header hop le --> nap ma tran tuong
    memcpy(maze, storage.maze_data, sizeof(maze));
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            maze[x][y].visited = 0;
        }
    }
  
    Maze_ComputeFloodfill();
    return true;
}

