#include "mpu6500.h"

#define MPU6500_ADDR        (0x68 << 1) // Dia chi I2C 8-bit (AD0 = LOW)
#define MPU6500_PWR_MGMT_1  0x6B
#define MPU6500_GYRO_CONFIG 0x1B
#define MPU6500_GYRO_ZOUT_H 0x47

// Thang do ±2000 deg/s --> Re-scale factor: 16.4 LSB/(deg/s)
#define GYRO_SCALE_FACTOR   16.4f

static I2C_HandleTypeDef *mpu_i2c;
static MPU6500_Data mpu_data = {0};

static uint8_t MPU6500_WriteReg(uint8_t reg, uint8_t data) {
    return (HAL_I2C_Mem_Write(mpu_i2c, MPU6500_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) == HAL_OK);
}

static uint8_t MPU6500_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len) {
    return (HAL_I2C_Mem_Read(mpu_i2c, MPU6500_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK);
}

HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c) {
    mpu_i2c = hi2c;

    // 1. Thuc hien Reset chip va danh thuc (Power Management 1)
    if (!MPU6500_WriteReg(MPU6500_PWR_MGMT_1, 0x00)) return 0;
    HAL_Delay(50);

    // 2. Cau hinh Gyro Full Scale Range ±2000 deg/s (Thanh ghi 0x1B = 0x18)
    if (!MPU6500_WriteReg(MPU6500_GYRO_CONFIG, 0x18)) return 0;
    HAL_Delay(10);

    // 3. Calibrate offset khi khoi tao
    MPU6500_Calibrate();

    return HAL_OK;
}

void MPU6500_Calibrate(void) {
    int32_t sum = 0;
    uint8_t buf[2];
    uint16_t samples = 200;

    for (uint16_t i = 0; i < samples; i++) {
        if (MPU6500_ReadRegs(MPU6500_GYRO_ZOUT_H, buf, 2)) {
            int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
            sum += raw;
        }
        HAL_Delay(2);
    }

    // Tinh trung binh trôi Gyro-Z (dinh dang deg/s)
    mpu_data.gz_offset = ((float)sum / samples) / GYRO_SCALE_FACTOR;
    mpu_data.yaw_angle = 0.0f;
}

void MPU6500_Read_GyroZ(float dt) {
    uint8_t buf[2];

    if (MPU6500_ReadRegs(MPU6500_GYRO_ZOUT_H, buf, 2)) {
        int16_t raw_z = (int16_t)((buf[0] << 8) | buf[1]);
        
        // Quy doi ve deg/s
        mpu_data.gyro_z_raw = (float)raw_z / GYRO_SCALE_FACTOR;
        
        // Bù offset
        mpu_data.gyro_z = mpu_data.gyro_z_raw - mpu_data.gz_offset;

        // Tich phan Euler tinh goc xoay Yaw (Do)
        mpu_data.yaw_angle += mpu_data.gyro_z * dt;
    }
}

MPU6500_Data* MPU6500_GetData(void) {
    return &mpu_data;
}

void MPU6500_ResetYaw(void) {
    mpu_data.yaw_angle = 0.0f;
}