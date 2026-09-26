#ifndef BT_DEBUG_H
#define BT_DEBUG_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

// Bien luu thong so PID bam tuong
extern float kp_wall;
extern float ki_wall;
extern float kd_wall;

// Bien luu thong PID Gyro
extern float kp_gyro;
extern float ki_gyro;
extern float kd_gyro;

void BT_Init(UART_HandleTypeDef *huart);
void BT_Init_Rx(UART_HandleTypeDef *huart);
void BT_ParseCommand(char *cmd);

#define LOG_SYS(...)          printf("[SYS] " __VA_ARGS__)
#define LOG_SRCH(...)         printf("[SRCH] " __VA_ARGS__)
#define LOG_WRN(...)          printf("[WRN] " __VA_ARGS__)
#define LOG_ERR(...)          printf("[ERR] " __VA_ARGS__)
#define LOG_MAZE_POS(x, y, dir)    printf("[MAZE] POS:%d,%d,%c\r\n", (int)(x), (int)(y), (char)(dir))
#define LOG_MAZE_STAT(f, v, s)     printf("[MAZE] STAT:%d,%d,%d\r\n", (int)(f), (int)(v), (int)(s))

#endif /* BT_DEBUG_H */