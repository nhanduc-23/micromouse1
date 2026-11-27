#ifndef BT_DEBUG_H
#define BT_DEBUG_H

#include "main.h"
#include <stdio.h>
#include <stdbool.h>

// Bien luu thong so PID hien hanh
extern float kp_wall, ki_wall, kd_wall;
extern float kp_gyro, ki_gyro, kd_gyro;

// Co cho phep / tam dung in dong du lieu cam bien len man hinh
extern bool g_bt_stream_enabled;

void BT_Init(UART_HandleTypeDef *huart);
void BT_Init_Rx(UART_HandleTypeDef *huart);
void BT_Process(void); // Goi trong vong lap chinh de xu ly lenh ngoai ISR

#define LOG_SYS(...)          printf("[SYS] " __VA_ARGS__)
#define LOG_SRCH(...)         printf("[SRCH] " __VA_ARGS__)
#define LOG_WRN(...)          printf("[WRN] " __VA_ARGS__)
#define LOG_ERR(...)          printf("[ERR] " __VA_ARGS__)
#define LOG_MAZE_POS(x, y, dir)    printf("[MAZE] POS:%d,%d,%c\r\n", (int)(x), (int)(y), (char)(dir))
#define LOG_MAZE_STAT(f, v, s)     printf("[MAZE] STAT:%d,%d,%d\r\n", (int)(f), (int)(v), (int)(s))

#endif /* BT_DEBUG_H */