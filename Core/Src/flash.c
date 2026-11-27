#include "flash.h"

HAL_StatusTypeDef Flash_EraseMazeSector(void) {
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;

    HAL_FLASH_Unlock();

    // 1. Xoa toan bo cac co loi con dong lai truoc do
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    // 2. Cau hinh xoa Sector 7
    erase_init.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V - 3.6V (BlackPill dung 3.3V)
    erase_init.Sector       = FLASH_SECTOR_7;
    erase_init.NbSectors    = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    HAL_FLASH_Lock();

    return status;
}

HAL_StatusTypeDef Flash_SaveData(uint8_t *data, uint16_t size) {
    if (data == NULL || size == 0) return HAL_ERROR;

    // 1. Xoa Sector 7 truoc khi ghi
    if (Flash_EraseMazeSector() != HAL_OK) return HAL_ERROR;

    HAL_FLASH_Unlock();

    // Xoa co loi truoc khi nap tung byte
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    for (uint16_t i = 0; i < size; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_MAZE_ADDR + i, data[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }

    HAL_FLASH_Lock();

    // 2. Lam moi Data Cache & Instruction Cache cua ART Accelerator de tranh doc lai cache cu
    __HAL_FLASH_DATA_CACHE_DISABLE();
    __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
    __HAL_FLASH_DATA_CACHE_RESET();
    __HAL_FLASH_INSTRUCTION_CACHE_RESET();
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
    __HAL_FLASH_DATA_CACHE_ENABLE();

    return HAL_OK;
}

void Flash_ReadData(uint8_t *data, uint16_t size) {
    if (data == NULL) return;
    uint8_t *flash_ptr = (uint8_t *)FLASH_MAZE_ADDR;
    for (uint16_t i = 0; i < size; i++) {
        data[i] = flash_ptr[i];
    }
}