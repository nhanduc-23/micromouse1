#ifndef FLASH_H
#define FLASH_H

#include "main.h"

//Vi tri Sector 7 trong bo nho Flash STM32F411 (128KB cuoi)
#define FLASH_MAZE_ADDR 0x08060000
#define FLASH_MAGIC_HEADER 0x55AA1234 // dinh nghia ma nhan dang Flash hop le
HAL_StatusTypeDef Flash_EraseMazeSector(void);
HAL_StatusTypeDef Flash_SaveData(uint8_t *data, uint16_t size);
void Flash_ReadData(uint8_t *data, uint16_t size);

#endif /* FLASH_H */