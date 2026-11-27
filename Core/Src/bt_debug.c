/**
  ******************************************************************************
  * @file           : bt_debug.c
  * @brief          : Giao tiep UART Bluetooth & Tinh chinh PID tu xa
  ******************************************************************************
  */

#include "bt_debug.h"
#include "motion_controller.h"
#include <string.h>
#include <stdlib.h>

static UART_HandleTypeDef *bt_huart = NULL;
static uint8_t rx_byte = 0;
static char rx_buffer[64];
static uint8_t rx_index = 0;

static char pending_command[64];
static volatile bool has_new_command = false;

// Co bat/tat in luong du lieu ToF (Mac dinh: BAT)
bool g_bt_stream_enabled = true;

// Thong so khoi dau cua Wall PID (Dong bo voi Motion_Init)
float kp_wall = 0.25f;
float ki_wall = 0.0f;
float kd_wall = 0.0f;

// Thong so khoi dau cua Gyro PID (Dong bo voi Motion_Init)
float kp_gyro = 2.15f;
float ki_gyro = 0.00f;
float kd_gyro = 0.45f;

void BT_Init(UART_HandleTypeDef *huart) {
    bt_huart = huart;
}

void BT_Init_Rx(UART_HandleTypeDef *huart) {
    bt_huart = huart;
    HAL_UART_Receive_IT(bt_huart, &rx_byte, 1);
}

/* ========================================================================== */
/* CHUYEN HUONG PRINTF HO TRO CA GCC (PLATFORMIO/CUBEIDE) VA KEIL MDK          */
/* ========================================================================== */
#ifdef __GNUC__
int __io_putchar(int ch) {
    if (bt_huart != NULL) {
        HAL_UART_Transmit(bt_huart, (uint8_t *)&ch, 1, 10);
    }
    return ch;
}

int _write(int file, char *ptr, int len) {
    if (bt_huart != NULL && ptr != NULL) {
        HAL_UART_Transmit(bt_huart, (uint8_t *)ptr, (uint16_t)len, 100);
    }
    return len;
}
#else
int fputc(int ch, FILE *f) {
    if (bt_huart != NULL) {
        HAL_UART_Transmit(bt_huart, (uint8_t *)&ch, 1, 10);
    }
    return ch;
}
#endif

/* ========================================================================== */
/* XU LY NGAT UART VA TU DONG PHUC HOI LOI OVERRUN                            */
/* ========================================================================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (bt_huart != NULL && huart->Instance == bt_huart->Instance) {
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (rx_index > 0 && !has_new_command) {
                rx_buffer[rx_index] = '\0';
                strncpy(pending_command, rx_buffer, sizeof(pending_command) - 1);
                pending_command[sizeof(pending_command) - 1] = '\0';
                has_new_command = true;
            }
            rx_index = 0;
        } else if (rx_index < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index++] = (char)rx_byte;
        } else {
            rx_index = 0; // Chong tran bo dem
        }

        HAL_UART_Receive_IT(bt_huart, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (bt_huart != NULL && huart->Instance == bt_huart->Instance) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        HAL_UART_Receive_IT(bt_huart, &rx_byte, 1);
    }
}

/* ========================================================================== */
/* PHAN TICH VA THUC THI LENH TU BLUETOOTH                                    */
/* ========================================================================== */
static void BT_ParseCommand(char *cmd) {
    float p = 0.0f, i = 0.0f, d = 0.0f;

    // 1. Lenh Bat/Tat in luong ToF: Phim "T" hoac "t"
    if (cmd[0] == 'T' || cmd[0] == 't') {
        g_bt_stream_enabled = !g_bt_stream_enabled;
        LOG_SYS("\r\n>>> ToF Stream: %s <<<\r\n", g_bt_stream_enabled ? "BAT (RUN)" : "TAT (PAUSE)");
        return;
    }

    // 2. Lenh xem thong so PID hien tai: Phim "P" hoac "p"
    if (cmd[0] == 'P' || cmd[0] == 'p') {
        LOG_SYS("\r\n--- CURRENT PID VALUES ---\r\n");
        LOG_SYS("WALL: Kp=%.2f, Ki=%.3f, Kd=%.2f\r\n", kp_wall, ki_wall, kd_wall);
        LOG_SYS("GYRO: Kp=%.2f, Ki=%.3f, Kd=%.2f\r\n", kp_gyro, ki_gyro, kd_gyro);
        LOG_SYS("--------------------------\r\n");
        return;
    }

    // 3. Lenh cap nhat Wall PID: Ho tro ca "W 0.75 0.0 0.6" va "PID_WALL 0.75 0.0 0.6"
    if ((cmd[0] == 'W' || cmd[0] == 'w') && (cmd[1] == ' ' || cmd[1] == '\t')) {
        if (sscanf(cmd + 1, "%f %f %f", &p, &i, &d) >= 1) {
            kp_wall = p; ki_wall = i; kd_wall = d;
            Motion_UpdatePIDWall(p, i, d);
            LOG_SYS("\r\n>>> [OK] WALL PID: P=%.2f, I=%.3f, D=%.2f\r\n", p, i, d);
            return;
        }
    } else if (sscanf(cmd, "PID_WALL %f %f %f", &p, &i, &d) == 3) {
        kp_wall = p; ki_wall = i; kd_wall = d;
        Motion_UpdatePIDWall(p, i, d);
        LOG_SYS("\r\n>>> [OK] PID Wall updated: P=%.2f, I=%.2f, D=%.2f\r\n", p, i, d);
        return;
    }

    // 4. Lenh cap nhat Gyro PID: Ho tro ca "G 2.3 0.0 0.65", "PID_GYRO...", va "PID..."
    if ((cmd[0] == 'G' || cmd[0] == 'g') && (cmd[1] == ' ' || cmd[1] == '\t')) {
        if (sscanf(cmd + 1, "%f %f %f", &p, &i, &d) >= 1) {
            kp_gyro = p; ki_gyro = i; kd_gyro = d;
            Motion_UpdatePIDGyro(p, i, d);
            LOG_SYS("\r\n>>> [OK] GYRO PID: P=%.2f, I=%.3f, D=%.2f\r\n", p, i, d);
            return;
        }
    } else if (sscanf(cmd, "PID_GYRO %f %f %f", &p, &i, &d) == 3) {
        kp_gyro = p; ki_gyro = i; kd_gyro = d;
        Motion_UpdatePIDGyro(p, i, d);
        LOG_SYS("\r\n>>> [OK] PID Gyro updated: P=%.2f, I=%.2f, D=%.2f\r\n", p, i, d);
        return;
    } else if (sscanf(cmd, "PID %f %f %f", &p, &i, &d) == 3) {
        kp_gyro = p; ki_gyro = i; kd_gyro = d;
        Motion_UpdatePIDGyro(p, i, d);
        LOG_SYS("\r\n>>> [OK] PID Gyro (Default) updated: P=%.2f, I=%.2f, D=%.2f\r\n", p, i, d);
        return;
    }

    LOG_WRN("Unknown cmd: %s\r\n", cmd);
}

// Xu ly lenh Bluetooth ben ngoai ngat ISR
void BT_Process(void) {
    if (has_new_command) {
        BT_ParseCommand(pending_command);
        has_new_command = false;
    }
}