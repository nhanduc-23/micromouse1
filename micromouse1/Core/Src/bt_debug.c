#include "bt_debug.h"
#include "motion_controller.h"
#include <string.h>
#include <stdlib.h>

static UART_HandleTypeDef *bt_huart = NULL;
static uint8_t rx_data = 0;
static char rx_buffer[64];
static uint8_t rx_index = 0;

//Gia tri ban dau c?a Wall PID
float kp_wall = 2.0f;
float ki_wall = 0.0f;
float kd_wall = 0.5f;

// Gia tri ban dau cua Gyro PID
float kp_gyro = 4.0f;
float ki_gyro = 0.05f;
float kd_gyro = 0.8f;

void BT_Init(UART_HandleTypeDef *huart) {
    bt_huart = huart;
}

void BT_Init_Rx(UART_HandleTypeDef *huart) {
    bt_huart = huart;
    // Kich hoat ngat nhan 1 byte tu Bluetooth UART
    HAL_UART_Receive_IT(bt_huart, &rx_data, 1);
}

// Ghi de ham fputc de chuyen huong printf qua Bluetooth UART
int fputc(int ch, FILE *f) {
    if (bt_huart != NULL) {
        HAL_UART_Transmit(bt_huart, (uint8_t *)&ch, 1, 10);
    }
    return ch;
}

// Ham xu ly ngat nhan UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (bt_huart != NULL && huart->Instance == bt_huart->Instance) {
        if (rx_data == '\n' || rx_data == '\r') {
            rx_buffer[rx_index] = '\0';
            if (rx_index > 0) {
                BT_ParseCommand(rx_buffer);
            }
            rx_index = 0;
        } else if (rx_index < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index++] = rx_data;
        } else{
					//Reset rx_index khi tran bo dem de san sang nhap lenh tiep theo
        rx_index = 0;
				}
        // Tiep tuc cho ngat nhan byte tiep theo
        HAL_UART_Receive_IT(bt_huart, &rx_data, 1);
    }
}

void BT_ParseCommand(char *cmd) {
    float p, i, d;

	// 1. Lenh cap nhat PID bam tuong: PID_WALL
    if (sscanf(cmd, "PID_WALL %f %f %f", &p, &i, &d) == 3) {
        kp_wall = p;
        ki_wall = i;
        kd_wall = d;
        Motion_UpdatePIDWall(p, i, d);
        LOG_SYS("Cap nhat PID Wall thanh cong: P=%.2f, I=%.2f, D=%.2f\r\n", p, i, d);
    }
    // 2. Lenh cap nhat PID Gyro: "PID_GYRO 4.5 0.05 1.0"
    else if (sscanf(cmd, "PID_GYRO %f %f %f", &p, &i, &d) == 3) {
        kp_gyro = p;
        ki_gyro = i;
        kd_gyro = d;
        Motion_UpdatePIDGyro(p, i, d);
        LOG_SYS("Cap nhat PID Gyro thanh cong: P=%.2f, I=%.2f, D=%.2f\r\n", p, i, d);
    }
    // 3. Lenh mac dinh cho Gyro PID (tuong thích ngu?c): "PID 4.0 0.05 0.8"
    else if (sscanf(cmd, "PID %f %f %f", &p, &i, &d) == 3) {
        kp_gyro = p;
        ki_gyro = i;
        kd_gyro = d;
        Motion_UpdatePIDGyro(p, i, d);
        LOG_SYS("Cap nhat PID Gyro (Default) thanh cong: P=%.2f, I=%.2f, D=%.2f\r\n", p, i, d);
    } 
    else {
        LOG_WRN("Lenh khong hop le: %s\r\n", cmd);
    }
}