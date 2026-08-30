#include "vl53l0x.h"

extern I2C_HandleTypeDef hi2c1;
Distance_Data_t sensor_dist = {0};

// Ham ho tro doi dia chi I2C cho 1 cam bien VL53L0X
static void VL53L0X_SetAddress(uint8_t current_addr, uint8_t new_addr)
{
    uint8_t payload[2] = {0x8A, (uint8_t)(new_addr >> 1)};
    HAL_I2C_Master_Transmit(&hi2c1, current_addr, payload, 2, HAL_MAX_DELAY);
}

// Khoi tao va phan chia dia chi I2C cho 4 cam bien qua chan XSHUT
void VL53L0X_Init_All(void)
{
    // 1. Keo 4 chan XSHUT xuong LOW de reset va dua tat ca cam bien vao che do cho
    HAL_GPIO_WritePin(XSHUT_LEFT_PORT, XSHUT_LEFT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_DIAG_L_PORT, XSHUT_DIAG_L_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_DIAG_R_PORT, XSHUT_DIAG_R_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_RIGHT_PORT, XSHUT_RIGHT_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);

    // 2. Kich hoat va doi dia chi I2C lan luot cho tung cam bien
    // Laser Ngang Trai (PA3)
    HAL_GPIO_WritePin(XSHUT_LEFT_PORT, XSHUT_LEFT_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
    VL53L0X_SetAddress(VL53L0X_ADDR_DEFAULT, VL53L0X_ADDR_LEFT);

    // Laser Cheo Trai (PA4)
    HAL_GPIO_WritePin(XSHUT_DIAG_L_PORT, XSHUT_DIAG_L_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
    VL53L0X_SetAddress(VL53L0X_ADDR_DEFAULT, VL53L0X_ADDR_DIAG_LEFT);

    // Laser Cheo Phai (PB0)
    HAL_GPIO_WritePin(XSHUT_DIAG_R_PORT, XSHUT_DIAG_R_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
    VL53L0X_SetAddress(VL53L0X_ADDR_DEFAULT, VL53L0X_ADDR_DIAG_RIGHT);

    // Laser Ngang Phai (PB1)
    HAL_GPIO_WritePin(XSHUT_RIGHT_PORT, XSHUT_RIGHT_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
    VL53L0X_SetAddress(VL53L0X_ADDR_DEFAULT, VL53L0X_ADDR_RIGHT);
}

// Ham doc khoang cach tu tat ca cam bien (truyen vao con tro dist)
void VL53L0X_Read_All(Distance_Data_t *dist) {
    if (dist == NULL) return;

    // TODO: Doc thanh ghi khoang cach I2C thuc te va luu vao sensor_dist
    dist->left = sensor_dist.left;
    dist->diag_left = sensor_dist.diag_left;
    dist->diag_right = sensor_dist.diag_right;
    dist->right = sensor_dist.right;
}

// Kiem tra co tuong phia truoc hay khong (dung 2 mat cheo)
bool wallFront(void) {
    return (sensor_dist.diag_left < THRESHOLD_DIAG_FRONT_MM || 
            sensor_dist.diag_right < THRESHOLD_DIAG_FRONT_MM);
}

// Kiem tra tuong ben trai (dung mat ngang trai)
bool wallLeft(void) {
    return (sensor_dist.left < THRESHOLD_SIDE_MM);
}

// Kiem tra tuong ben phai (dung mat ngang phai)
bool wallRight(void) {
    return (sensor_dist.right < THRESHOLD_SIDE_MM);
}