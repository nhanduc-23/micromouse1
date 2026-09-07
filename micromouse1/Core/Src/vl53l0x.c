#include "vl53l0x.h"
#include "conflig.h"

// Dia chi I2C 8-bit moi cho 4 cam bien (Write Address)
#define ADDR_L   0x54
#define ADDR_FL  0x56
#define ADDR_FR  0x58
#define ADDR_R   0x5A

#define VL53L0X_DEFAULT_ADDR 0x52

static I2C_HandleTypeDef *sensor_i2c;

// Ghi 1 byte vao thanh ghi cam bien
static void VL53L0X_WriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    HAL_I2C_Master_Transmit(sensor_i2c, addr, buf, 2, 10);
}

// Doc 2 byte khoang cach tu thanh ghi 0x14
static uint16_t VL53L0X_ReadRange(uint8_t addr) {
    uint8_t reg = 0x14;
    uint8_t data[2] = {0};
    
    HAL_I2C_Master_Transmit(sensor_i2c, addr, &reg, 1, 10);
    if (HAL_I2C_Master_Receive(sensor_i2c, addr, data, 2, 10) == HAL_OK) {
        return (uint16_t)((data[0] << 8) | data[1]);
    }
    return 8190; // Tra ve gia tri max khi loi hoac vuot tam do
}

uint8_t VL53L0X_Init_All(I2C_HandleTypeDef *hi2c) {
    sensor_i2c = hi2c;

    // 1. Dua tat ca chan XSHUT ve LOW de Reset toan bo cam bien
    HAL_GPIO_WritePin(XSHUT_L.port,  XSHUT_L.pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_FL.port, XSHUT_FL.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_FR.port, XSHUT_FR.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_R.port,  XSHUT_R.pin,  GPIO_PIN_RESET);
    HAL_Delay(10);

    // 2. Bat va doi dia chi I2C tung cam bien mot theo thu tu
    // Cam bien Trai (Left - PA3)
    HAL_GPIO_WritePin(XSHUT_L.port, XSHUT_L.pin, GPIO_PIN_SET);
    HAL_Delay(5);
    VL53L0X_WriteReg(VL53L0X_DEFAULT_ADDR, 0x8A, ADDR_L >> 1);

    // Cam bien Truoc-Trai (Front-Left - PA4)
    HAL_GPIO_WritePin(XSHUT_FL.port, XSHUT_FL.pin, GPIO_PIN_SET);
    HAL_Delay(5);
    VL53L0X_WriteReg(VL53L0X_DEFAULT_ADDR, 0x8A, ADDR_FL >> 1);

    // Cam bien Truoc-Phai (Front-Right - PB0)
    HAL_GPIO_WritePin(XSHUT_FR.port, XSHUT_FR.pin, GPIO_PIN_SET);
    HAL_Delay(5);
    VL53L0X_WriteReg(VL53L0X_DEFAULT_ADDR, 0x8A, ADDR_FR >> 1);

    // Cam bien Phai (Right - PB1)
    HAL_GPIO_WritePin(XSHUT_R.port, XSHUT_R.pin, GPIO_PIN_SET);
    HAL_Delay(5);
    VL53L0X_WriteReg(VL53L0X_DEFAULT_ADDR, 0x8A, ADDR_R >> 1);

    return 1;
}

void VL53L0X_Read_All(VL53L0X_Data *data) {
    data->l  = VL53L0X_ReadRange(ADDR_L);
    data->fl = VL53L0X_ReadRange(ADDR_FL);
    data->fr = VL53L0X_ReadRange(ADDR_FR);
    data->r  = VL53L0X_ReadRange(ADDR_R);
}