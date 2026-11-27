#include "mpu6500.h"
#include "config.h"
#include <math.h>

// Dia chi thanh ghi noi cua MPU6500
#define MPU_REG_SMPLRT_DIV    0x19
#define MPU_REG_CONFIG        0x1A
#define MPU_REG_GYRO_CONFIG   0x1B
#define MPU_REG_ACCEL_CONFIG  0x1C
#define MPU_REG_GYRO_ZOUT_H   0x47
#define MPU_REG_PWR_MGMT_1    0x6B
#define MPU_REG_WHO_AM_I      0x75

// Thang do ±2000 deg/s -> He so ti le: 16.4 LSB/(deg/s)
#define GYRO_SCALE_FACTOR     16.4f

// Nguong loc vung chet (Deadband): Bo qua nhieu nho hon 0.15 deg/s khi dung yen
#define GYRO_DEADBAND_DPS     0.15f

static I2C_HandleTypeDef *mpu_i2c;
static uint8_t mpu_address = (0x68 << 1); // Mac dinh 0xD0
static MPU6500_Data mpu_data = {0};

// Ghi 1 byte vao thanh ghi (giam timeout xuong 10ms tranh treo CPU)
static bool MPU6500_WriteReg(uint8_t reg, uint8_t data) {
    return (HAL_I2C_Mem_Write(mpu_i2c, mpu_address, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10) == HAL_OK);
}

// Doc nhieu byte tu thanh ghi
static bool MPU6500_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len) {
    return (HAL_I2C_Mem_Read(mpu_i2c, mpu_address, reg, I2C_MEMADD_SIZE_8BIT, data, len, 10) == HAL_OK);
}

HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c) {
    mpu_i2c = hi2c;
    mpu_data.is_connected = false;

    // 1. Quet va xac thuc chip qua thanh ghi WHO_AM_I (0x75)
    uint8_t who_am_i = 0;
    mpu_address = (0x68 << 1); // Thu dia chi 0xD0 (AD0 = GND)
    MPU6500_ReadRegs(MPU_REG_WHO_AM_I, &who_am_i, 1);

    if (who_am_i != 0x70 && who_am_i != 0x71 && who_am_i != 0x73 && who_am_i != 0x68) {
        mpu_address = (0x69 << 1); // Thu dia chi 0xD2 (AD0 = 3.3V)
        who_am_i = 0;
        MPU6500_ReadRegs(MPU_REG_WHO_AM_I, &who_am_i, 1);
        
        // Neu ca hai dia chi deu khong phan hoi dung ma chip thi tra ve loi
        if (who_am_i != 0x70 && who_am_i != 0x71 && who_am_i != 0x73 && who_am_i != 0x68) {
            return HAL_ERROR;
        }
    }

    // 2. Reset chip de dua toan bo thanh ghi ve mac dinh
    if (!MPU6500_WriteReg(MPU_REG_PWR_MGMT_1, 0x80)) return HAL_ERROR;
    HAL_Delay(50);

    // 3. Danh thuc chip va dong bo xung nhip theo Gyroscope (chong troi nhiet)
    if (!MPU6500_WriteReg(MPU_REG_PWR_MGMT_1, 0x01)) return HAL_ERROR;
    HAL_Delay(10);

    // 4. Bat bo loc so DLPF bang thong 42Hz de triet tieu rung dong co N20
    if (!MPU6500_WriteReg(MPU_REG_CONFIG, 0x03)) return HAL_ERROR;

    // 5. Thiet lap toc do lay mau: 1kHz / (1 + 4) = 200Hz
    if (!MPU6500_WriteReg(MPU_REG_SMPLRT_DIV, 0x04)) return HAL_ERROR;

    // 6. Cau hinh dải do Gyro ±2000 deg/s (FS_SEL = 3 -> 0x18)
    if (!MPU6500_WriteReg(MPU_REG_GYRO_CONFIG, 0x18)) return HAL_ERROR;
    HAL_Delay(10);

    mpu_data.is_connected = true;

    // 7. Hieu chuan sai so tinh (bias offset) ban dau
    MPU6500_Calibrate();

    return HAL_OK;
}

void MPU6500_Calibrate(void) {
    if (!mpu_data.is_connected) return;

    int32_t sum = 0;
    uint8_t buf[2];
    uint16_t samples = 200;
    uint16_t valid_samples = 0;

    for (uint16_t i = 0; i < samples; i++) {
        if (MPU6500_ReadRegs(MPU_REG_GYRO_ZOUT_H, buf, 2)) {
            int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
            sum += raw;
            valid_samples++;
        }
        
        // Nhay LED canh bao dang trong qua trinh Calibrate
        if (i % 25 == 0) {
            HAL_GPIO_TogglePin(LED_BOARD.port, LED_BOARD.pin);
        }
        HAL_Delay(2);
    }

    // Tat LED sau khi hoan tat hieu chuan
    HAL_GPIO_WritePin(LED_BOARD.port, LED_BOARD.pin, GPIO_PIN_SET);

    if (valid_samples > 0) {
        // Tinh trung binh sai lech goc trôi tĩnh (deg/s)
        mpu_data.gz_offset = ((float)sum / valid_samples) / GYRO_SCALE_FACTOR;
    }
    mpu_data.yaw_angle = 0.0f;
}

void MPU6500_Read_GyroZ(float dt) {
    if (!mpu_data.is_connected) return;

    uint8_t buf[2];
    if (MPU6500_ReadRegs(MPU_REG_GYRO_ZOUT_H, buf, 2)) {
        int16_t raw_z = (int16_t)((buf[0] << 8) | buf[1]);

        // Quy doi thanh van toc goc tho (deg/s)
        mpu_data.gyro_z_raw = (float)raw_z / GYRO_SCALE_FACTOR;

        // Bù trừ sai so tĩnh (offset)
        float gz = mpu_data.gyro_z_raw - mpu_data.gz_offset;

        // Ap dung bo loc vung chet (Deadband Filter) chong tu troi goc khi dung yen
        if (fabsf(gz) < GYRO_DEADBAND_DPS) {
            gz = 0.0f;
        }
        mpu_data.gyro_z = gz;

        // Tich phan van toc goc theo thoi gian de tinh goc xoay Yaw (Do)
        mpu_data.yaw_angle += mpu_data.gyro_z * dt;
    }
}

MPU6500_Data* MPU6500_GetData(void) {
    return &mpu_data;
}

void MPU6500_ResetYaw(void) {
    mpu_data.yaw_angle = 0.0f;
}