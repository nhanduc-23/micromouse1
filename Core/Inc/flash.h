#ifndef FLASH_H
#define FLASH_H

#include "main.h"
#include <stdbool.h>

// Vi tri Sector 7 trong bo nho Flash STM32F411 (128KB cuoi cung: 0x08060000 - 0x0807FFFF)
#define FLASH_MAZE_ADDR       0x08060000
#define FLASH_MAGIC_HEADER    0x55AA1234 // Ma nhan dang tinh toan ven cua ban do

// Xoa toan bo Sector 7 truoc khi ghi du lieu moi
HAL_StatusTypeDef Flash_EraseMazeSector(void);

// Ghi du lieu mang byte vao Flash
HAL_StatusTypeDef Flash_SaveData(uint8_t *data, uint16_t size);

// Doc du lieu tu Flash ra bo dem RAM
void Flash_ReadData(uint8_t *data, uint16_t size);

#endif /* FLASH_H */