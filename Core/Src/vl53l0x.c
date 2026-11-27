/**
  ******************************************************************************
  * @file           : vl53l0x.c
  * @brief          : Driver VL53L0X chuan Pololu / ST API + Loc nhieu RangeStatus
  *                   Chong nhieu phan xa goc hep (grazing angle) tren hanh lang dai
  ******************************************************************************
  */

#include "vl53l0x.h"
#include "config.h"

// Dia chi I2C 8-bit cho 4 cam bien theo hinh hoc thuc te:
#define ADDR_FRONT_L   0x54  // 7-bit: 0x2A | Trai Ngoai (PA3 - Nhin thang 0 do)
#define ADDR_DIAG_L    0x56  // 7-bit: 0x2B | Trai Trong (PA4 - Chech 45 do te trai)
#define ADDR_DIAG_R    0x58  // 7-bit: 0x2C | Phai Trong (PB0 - Chech 45 do te phai)
#define ADDR_FRONT_R   0x5A  // 7-bit: 0x2D | Phai Ngoai (PB1 - Nhin thang 0 do)

#define VL53L0X_DEFAULT_ADDR                 0x52 // Dia chi khoi tao mac dinh (7-bit: 0x29)
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID  0xC0 // Thanh ghi nhan dien chip (= 0xEE)

// Nguong tho toi thieu (VL53L0X_OFFSET_MM = 38mm la mep vo xe -> raw <= 43mm la nhieu crosstalk)
#define VL53L0X_MIN_VALID_RAW_MM             (VL53L0X_OFFSET_MM + 5)

static I2C_HandleTypeDef *sensor_i2c = NULL;

// Bo dem luu gia tri do gan nhat cua 4 cam bien tranh dut gay chu ky
static uint16_t last_valid_range[4] = {
    VL53L0X_OUT_OF_RANGE, VL53L0X_OUT_OF_RANGE,
    VL53L0X_OUT_OF_RANGE, VL53L0X_OUT_OF_RANGE
};

// Bang thanh ghi Tuning Settings tieu chuan cua STMicroelectronics (giong Pololu Arduino)
typedef struct {
    uint8_t reg;
    uint8_t val;
} VL53L0X_RegInit;

static const VL53L0X_RegInit tuning_settings[] = {
    {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00},
    {0x10, 0x00}, {0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF},
    {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
    {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00},
    {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25},
    {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
    {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00},
    {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
    {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
    {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00},
    {0x78, 0x21}, {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00},
    {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
    {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00},
    {0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01}, {0x31, 0x04},
    {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
    {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28},
    {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE},
    {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
    {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01},
    {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}
};

static HAL_StatusTypeDef VL53L0X_WriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(sensor_i2c, addr, buf, 2, 5);
}

static HAL_StatusTypeDef VL53L0X_WriteReg16Bit(uint8_t addr, uint8_t reg, uint16_t val) {
    uint8_t buf[3] = {reg, (uint8_t)((val >> 8) & 0xFF), (uint8_t)(val & 0xFF)};
    return HAL_I2C_Master_Transmit(sensor_i2c, addr, buf, 3, 5);
}

static uint8_t VL53L0X_ReadReg(uint8_t addr, uint8_t reg) {
    uint8_t val = 0;
    HAL_I2C_Mem_Read(sensor_i2c, addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 5);
    return val;
}

// Thuc hien chu trinh hieu chuan VHV hoac Phase Calib
static bool VL53L0X_PerformSingleRefCalibration(uint8_t addr, uint8_t vhv_init_byte) {
    VL53L0X_WriteReg(addr, 0x00, 0x01 | vhv_init_byte); // SYSRANGE_START

    uint32_t start = HAL_GetTick();
    while ((VL53L0X_ReadReg(addr, 0x13) & 0x07) == 0) {
        if (HAL_GetTick() - start > 100) {
            return false;
        }
    }

    VL53L0X_WriteReg(addr, 0x0B, 0x01); // Xoa co ngat
    VL53L0X_WriteReg(addr, 0x00, 0x00);
    return true;
}

// Lay thong tin SPAD va cau hinh ban do SPAD hop le
static bool VL53L0X_SetSpads(uint8_t addr) {
    VL53L0X_WriteReg(addr, 0x80, 0x01);
    VL53L0X_WriteReg(addr, 0xFF, 0x01);
    VL53L0X_WriteReg(addr, 0x00, 0x00);
    VL53L0X_WriteReg(addr, 0xFF, 0x06);
    VL53L0X_WriteReg(addr, 0x83, VL53L0X_ReadReg(addr, 0x83) | 0x04);
    VL53L0X_WriteReg(addr, 0xFF, 0x07);
    VL53L0X_WriteReg(addr, 0x81, 0x01);
    VL53L0X_WriteReg(addr, 0x80, 0x01);
    VL53L0X_WriteReg(addr, 0x94, 0x6B);
    VL53L0X_WriteReg(addr, 0x83, 0x00);

    uint32_t start = HAL_GetTick();
    while (VL53L0X_ReadReg(addr, 0x83) == 0x00) {
        if (HAL_GetTick() - start > 100) return false;
    }

    VL53L0X_WriteReg(addr, 0x83, 0x01);
    uint8_t tmp = VL53L0X_ReadReg(addr, 0x92);
    uint8_t spad_count = tmp & 0x7F;
    bool is_aperture = ((tmp >> 7) & 0x01) != 0;

    VL53L0X_WriteReg(addr, 0x81, 0x00);
    VL53L0X_WriteReg(addr, 0xFF, 0x06);
    VL53L0X_WriteReg(addr, 0x83, VL53L0X_ReadReg(addr, 0x83) & ~0x04);
    VL53L0X_WriteReg(addr, 0xFF, 0x01);
    VL53L0X_WriteReg(addr, 0x00, 0x01);
    VL53L0X_WriteReg(addr, 0xFF, 0x00);
    VL53L0X_WriteReg(addr, 0x80, 0x00);

    uint8_t ref_spad_map[6] = {0};
    HAL_I2C_Mem_Read(sensor_i2c, addr, 0xB0, I2C_MEMADD_SIZE_8BIT, ref_spad_map, 6, 10);

    VL53L0X_WriteReg(addr, 0xFF, 0x01);
    VL53L0X_WriteReg(addr, 0x4F, 0x00);
    VL53L0X_WriteReg(addr, 0x4E, 0x2C);
    VL53L0X_WriteReg(addr, 0xFF, 0x00);
    VL53L0X_WriteReg(addr, 0xB6, 0xB4);

    uint8_t first_spad = is_aperture ? 12 : 0;
    uint8_t enabled = 0;

    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad || enabled == spad_count) {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x01) {
            enabled++;
        }
    }

    HAL_I2C_Mem_Write(sensor_i2c, addr, 0xB0, I2C_MEMADD_SIZE_8BIT, ref_spad_map, 6, 10);
    return true;
}

// Khoi tao chi tiet cho tung cam bien
static HAL_StatusTypeDef VL53L0X_Init_Single(uint8_t new_addr_8bit) {
    // 1. Kiem tra dinh danh chip VL53L0X
    if (VL53L0X_ReadReg(VL53L0X_DEFAULT_ADDR, VL53L0X_REG_IDENTIFICATION_MODEL_ID) != 0xEE) {
        return HAL_ERROR;
    }

    // 2. Doi dia chi 7-bit sang dia chi moi
    VL53L0X_WriteReg(VL53L0X_DEFAULT_ADDR, 0x8A, new_addr_8bit >> 1);

    // 3. Cau hinh che do 2V8
    uint8_t vhv = VL53L0X_ReadReg(new_addr_8bit, 0x89);
    VL53L0X_WriteReg(new_addr_8bit, 0x89, vhv | 0x01);

    // 4. Doc bien stop_variable
    VL53L0X_WriteReg(new_addr_8bit, 0x88, 0x00);
    VL53L0X_WriteReg(new_addr_8bit, 0x80, 0x01);
    VL53L0X_WriteReg(new_addr_8bit, 0xFF, 0x01);
    VL53L0X_WriteReg(new_addr_8bit, 0x00, 0x00);
    uint8_t stop_variable = VL53L0X_ReadReg(new_addr_8bit, 0x91);
    VL53L0X_WriteReg(new_addr_8bit, 0x00, 0x01);
    VL53L0X_WriteReg(new_addr_8bit, 0xFF, 0x00);
    VL53L0X_WriteReg(new_addr_8bit, 0x80, 0x00);

    // 5. Nang nguong Signal Rate len 0.45 MCPS de loai bo tia phan xa yeu doc hanh lang dai
    VL53L0X_WriteReg(new_addr_8bit, 0x60, VL53L0X_ReadReg(new_addr_8bit, 0x60) | 0x12);
    VL53L0X_WriteReg16Bit(new_addr_8bit, 0x44, (uint16_t)(0.45f * (1 << 7))); // 0.45 MCPS
    VL53L0X_WriteReg(new_addr_8bit, 0x01, 0xFF);

    // 6. Cai dat bang SPAD tham chieu
    VL53L0X_SetSpads(new_addr_8bit);

    // 7. Nap toan bo bang thanh ghi Tuning Settings ST
    uint8_t num_tunings = sizeof(tuning_settings) / sizeof(tuning_settings[0]);
    for (uint8_t i = 0; i < num_tunings; i++) {
        VL53L0X_WriteReg(new_addr_8bit, tuning_settings[i].reg, tuning_settings[i].val);
    }

    // 8. Cau hinh ngat: New sample ready
    VL53L0X_WriteReg(new_addr_8bit, 0x0A, 0x04);
    VL53L0X_WriteReg(new_addr_8bit, 0x84, VL53L0X_ReadReg(new_addr_8bit, 0x84) & ~0x10);
    VL53L0X_WriteReg(new_addr_8bit, 0x0B, 0x01);

    // 9. CHU TRINH HIEU CHUAN PHAN CUNG (VHV & Phase Calib)
    VL53L0X_WriteReg(new_addr_8bit, 0x01, 0x01);
    VL53L0X_PerformSingleRefCalibration(new_addr_8bit, 0x40);

    VL53L0X_WriteReg(new_addr_8bit, 0x01, 0x02);
    VL53L0X_PerformSingleRefCalibration(new_addr_8bit, 0x00);

    // Khoi phuc Sequence Config buoc do chuan
    VL53L0X_WriteReg(new_addr_8bit, 0x01, 0xE8);

    // 10. Khoi dong che do do lien tuc (Continuous Back-to-Back)
    VL53L0X_WriteReg(new_addr_8bit, 0x80, 0x01);
    VL53L0X_WriteReg(new_addr_8bit, 0xFF, 0x01);
    VL53L0X_WriteReg(new_addr_8bit, 0x00, 0x00);
    VL53L0X_WriteReg(new_addr_8bit, 0x91, stop_variable);
    VL53L0X_WriteReg(new_addr_8bit, 0x00, 0x01);
    VL53L0X_WriteReg(new_addr_8bit, 0xFF, 0x00);
    VL53L0X_WriteReg(new_addr_8bit, 0x80, 0x00);

    VL53L0X_WriteReg(new_addr_8bit, 0x00, 0x02); // START CONTINUOUS
    return HAL_OK;
}

// Doc khoang cach: Kiem tra RangeStatus phan cung + loc nhieu crosstalk <= 43mm
static uint16_t VL53L0X_ReadRange(uint8_t addr, uint8_t idx) {
    uint8_t status = VL53L0X_ReadReg(addr, 0x13);
    if ((status & 0x07) != 0) {
        // Doc 12 byte tu 0x14 (bao gom RangeStatus tai 0x14 va RangeMM tai 0x1E..0x1F)
        uint8_t result_buf[12] = {0};
        if (HAL_I2C_Mem_Read(sensor_i2c, addr, 0x14, I2C_MEMADD_SIZE_8BIT, result_buf, 12, 5) == HAL_OK) {
            VL53L0X_WriteReg(addr, 0x0B, 0x01); // Xoa co ngat cho phep chip do tiep mau sau

            uint8_t range_status = (result_buf[0] >> 3) & 0x1F;
            uint16_t raw = (uint16_t)((result_buf[10] << 8) | result_buf[11]);

            // Chi chap nhan mau dat chuan RangeStatus (0 = Valid, 11 = Valid no wrap check)
            // Loai bo SigmaFail (1), SignalFail (2), PhaseWrap (4) va crosstalk <= 43mm
            bool hw_valid = (range_status == 0 || range_status == 11);

            if (!hw_valid || raw >= 8190 || raw <= VL53L0X_MIN_VALID_RAW_MM) {
                last_valid_range[idx] = VL53L0X_OUT_OF_RANGE;
            } else if (raw < 2000) {
                last_valid_range[idx] = raw - VL53L0X_OFFSET_MM;
            } else {
                last_valid_range[idx] = VL53L0X_OUT_OF_RANGE;
            }
        }
    }
    return last_valid_range[idx];
}

HAL_StatusTypeDef VL53L0X_Init_All(I2C_HandleTypeDef *hi2c) {
    sensor_i2c = hi2c;
    HAL_StatusTypeDef status = HAL_OK;

    for (int i = 0; i < 4; i++) {
        last_valid_range[i] = VL53L0X_OUT_OF_RANGE;
    }

    // 1. Reset tat ca cam bien (keo XSHUT ve LOW)
    HAL_GPIO_WritePin(XSHUT_L.port,  XSHUT_L.pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_FL.port, XSHUT_FL.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_FR.port, XSHUT_FR.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_R.port,  XSHUT_R.pin,  GPIO_PIN_RESET);
    HAL_Delay(15);

    // 2. Bat va khoi tao tung cam bien theo thu tu chan phan cung:
    HAL_GPIO_WritePin(XSHUT_L.port, XSHUT_L.pin, GPIO_PIN_SET);
    HAL_Delay(10);
    if (VL53L0X_Init_Single(ADDR_FRONT_L) != HAL_OK) status = HAL_ERROR;

    HAL_GPIO_WritePin(XSHUT_FL.port, XSHUT_FL.pin, GPIO_PIN_SET);
    HAL_Delay(10);
    if (VL53L0X_Init_Single(ADDR_DIAG_L) != HAL_OK) status = HAL_ERROR;

    HAL_GPIO_WritePin(XSHUT_FR.port, XSHUT_FR.pin, GPIO_PIN_SET);
    HAL_Delay(10);
    if (VL53L0X_Init_Single(ADDR_DIAG_R) != HAL_OK) status = HAL_ERROR;

    HAL_GPIO_WritePin(XSHUT_R.port, XSHUT_R.pin, GPIO_PIN_SET);
    HAL_Delay(10);
    if (VL53L0X_Init_Single(ADDR_FRONT_R) != HAL_OK) status = HAL_ERROR;

    return status;
}

void VL53L0X_Read_All(VL53L0X_Data *data) {
    if (!data) return;
    data->front_l = VL53L0X_ReadRange(ADDR_FRONT_L, 0);
    data->diag_l  = VL53L0X_ReadRange(ADDR_DIAG_L, 1);
    data->diag_r  = VL53L0X_ReadRange(ADDR_DIAG_R, 2);
    data->front_r = VL53L0X_ReadRange(ADDR_FRONT_R, 3);
}