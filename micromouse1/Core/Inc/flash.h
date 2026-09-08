#ifndef FLASH_H
#define FLASH_H

#include "main.h"

//Vi tri Sector 7 trong bo nho Flash STM32F411 (128KB cuoi)
#define FLASH_MAZE_ADDR 0X0906000
HAL_StatusTypeDef Flash_EraseMazeSector(void);
HAL_StatusTypeDef Flash_SaveData(uint8_t *data, uint16_t size);
void Flash_ReadData(uint8_t *data, uint16_t size);

#endif /* FLASH_H */