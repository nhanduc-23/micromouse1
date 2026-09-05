#include "bt_debug.h"
#include <string.h>
#include <stdlib.h>

static UART_HandleTypeDef *bt_huart = NULL;
static uint8_t rx_data = 0;
static char rx_buffer[64];
static uint8_t rx_index = 0;

// Gia tri khoi tao PID mac dinh
float kp_speed = 1.2f;
float ki_speed = 0.01f;
float kd_speed = 0.05f;

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
        HAL_UART_Transmit(bt_huart, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
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
        }
        
        // Tiep tuc cho ngat nhan byte tiep theo
        HAL_UART_Receive_IT(bt_huart, &rx_data, 1);
    }
}

// Bóc tách va xu ly lenh gui ve tu Web Console (Vi du: "PID 1.5 0.01 0.2")
void BT_ParseCommand(char *cmd) {
    float p, i, d;
    if (sscanf(cmd, "PID %f %f %f", &p, &i, &d) == 3) {
        kp_speed = p;
        ki_speed = i;
        kd_speed = d;
        LOG_SYS("Cap nhat PID thanh cong: P=%.2f, I=%.2f, D=%.2f", kp_speed, ki_speed, kd_speed);
    } else {
        LOG_WRN("Lenh khong hop le: %s", cmd);
    }
}