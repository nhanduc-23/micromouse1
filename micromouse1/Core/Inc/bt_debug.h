#ifndef BT_DEBUG_H
#define BT_DEBUG_H

#include "stm32f4xx_hal.h"
#include <stdio.h>

// Bien toan cuc luu thong so PID de module Motion su dung
extern float kp_speed;
extern float ki_speed;
extern float kd_speed;

// Khoi tao Bluetooth Debug truyê`n va nhan du lieu (USART6)
void BT_Init(UART_HandleTypeDef *huart);
void BT_Init_Rx(UART_HandleTypeDef *huart);

// Ham phan tich va xu ly chuoi lenh gui tu Web Console
void BT_ParseCommand(char *cmd);

// =================================================================
// MACRO GHI LOG CHUAN HOÁ CHO GIAO DIEN NEZUMI_CHAN DEBUG CONSOLE
// =================================================================

// Log He thong & Khoi tao (Tab ALL / SYS)
#define LOG_SYS(fmt, ...)          printf("[SYS] " fmt "\r\n", ##__VA_ARGS__)

// Log Thuat toan tim duong (Tab SRCH)
#define LOG_SRCH(fmt, ...)         printf("[SRCH] " fmt "\r\n", ##__VA_ARGS__)

// Log Canh bao va cham / lech tuong (Tab WRN)
#define LOG_WRN(fmt, ...)          printf("[WRN] " fmt "\r\n", ##__VA_ARGS__)

// Log Bao loi phan cung / Timeout (Tab ERR)
#define LOG_ERR(fmt, ...)          printf("[ERR] " fmt "\r\n", ##__VA_ARGS__)

// Cap nhat vi tri va huong Robot len ban do 16x16 (Tab MAZE)
#define LOG_MAZE_POS(x, y, dir)    printf("[MAZE] POS:%d,%d,%c\r\n", (int)(x), (int)(y), (char)(dir))

// Cap nhat thong so Me cung (Floodfill value, Visited count, Steps)
#define LOG_MAZE_STAT(f, v, s)     printf("[MAZE] STAT:%d,%d,%d\r\n", (int)(f), (int)(v), (int)(s))

#endif /* BT_DEBUG_H */