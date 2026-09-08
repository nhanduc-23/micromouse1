#include "flash.h"

HAL_StatusTypeDef Flash_EraseMazeSector(void) {
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error;

    HAL_FLASH_Unlock();
    erase_init.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase_init.Sector       = FLASH_SECTOR_7;
    erase_init.NbSectors    = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    HAL_FLASH_Lock();

    return status;
}

HAL_StatusTypeDef Flash_SaveData(uint8_t *data, uint16_t size) {
    if (Flash_EraseMazeSector() != HAL_OK) return HAL_ERROR;

    HAL_FLASH_Unlock();
    for (uint16_t i = 0; i < size; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_MAZE_ADDR + i, data[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }
    HAL_FLASH_Lock();
    return HAL_OK;
}

void Flash_ReadData(uint8_t *data, uint16_t size) {
    uint8_t *flash_ptr = (uint8_t *)FLASH_MAZE_ADDR;
    for (uint16_t i = 0; i < size; i++) {
        data[i] = flash_ptr[i];
    }
}