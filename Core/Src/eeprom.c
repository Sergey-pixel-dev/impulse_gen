#include "eeprom.h"

void EEPROM_Init(EEPROM_t *mem, uint8_t dev_addr, uint16_t block_size, uint16_t block_count, uint16_t page_size,
                 uint8_t mem_addr_size)
{
    mem->device_address = dev_addr;
    mem->block_size = block_size;
    mem->block_count = block_count;
    mem->page_size = page_size;
    mem->page_count = block_size / page_size;
    mem->mem_addr_size = mem_addr_size;
}

EEPROM_Status_t EEPROM_Write(EEPROM_t *mem, uint16_t addr, uint8_t *buffer, uint32_t n, uint32_t timeout)
{
    EEPROM_Status_t status;
    uint32_t bytes_written = 0;

    // Цикл постраничной записи
    while (bytes_written < n) {
        // 1. Вычислить текущий адрес и блок
        uint16_t current_addr = addr + bytes_written;
        uint16_t current_block = current_addr / mem->block_size;

        // 2. Определить сколько байт записать в текущую страницу
        // Вычисляем позицию внутри страницы
        uint16_t offset_in_page = current_addr % mem->page_size;
        // Сколько байт до конца страницы
        uint16_t bytes_to_page_end = mem->page_size - offset_in_page;
        // Записать минимум из: оставшиеся данные или место в странице
        uint16_t to_write = MIN(n - bytes_written, bytes_to_page_end);

        // 3. Сформировать адрес устройства для multi-block EEPROM
        // Для EEPROM < 4Кбит (24C02-24C16) младшие биты адреса блока
        // встраиваются в адрес устройства через биты A0, A1, A2
        uint8_t dev_addr = mem->device_address | current_block;

        // 4. Выполнить PAGE WRITE используя I2C примитивы (из datasheet):
        //    START → Device Address+W → ACK → Word Address → ACK → Data... → STOP

        status = I2C1_Start(timeout);
        if (status != I2C_OK)
            return status;

        status = I2C1_SendAddress(dev_addr, I2C_DIRECTION_WRITE, timeout);
        if (status != I2C_OK)
            return status;

        I2C1_ClearADDR();

        if (mem->mem_addr_size == I2C_MEMADD_SIZE_16BIT) {
            status = I2C1_SendByte((current_addr >> 8) & 0xFF, timeout);
            if (status != I2C_OK)
                return status;
        }
        status = I2C1_SendByte(current_addr & 0xFF, timeout);
        if (status != I2C_OK)
            return status;

        status = I2C1_SendBytes(buffer + bytes_written, to_write, timeout);
        if (status != I2C_OK)
            return status;

        status = I2C1_WaitBTF(timeout);
        if (status != I2C_OK)
            return status;

        I2C1_Stop(timeout);

        // ACK Polling - ожидание завершения записи
        status = I2C1_IsDeviceReady(dev_addr, 50, 1);
        if (status != I2C_OK)
            return status;
        bytes_written += to_write;
    }

    return I2C_OK;
}

EEPROM_Status_t EEPROM_Read(EEPROM_t *mem, uint16_t addr, uint8_t *buffer, uint32_t n, uint32_t timeout)
{
    EEPROM_Status_t status;
    uint32_t bytes_read = 0;

    // Цикл чтения по блокам
    while (bytes_read < n) {
        // 1. Вычислить текущий адрес и блок
        uint16_t current_addr = addr + bytes_read;
        uint16_t current_block = current_addr / mem->block_size;

        // 2. Определить сколько байт читать из текущего блока
        // Вычисляем позицию внутри блока
        uint16_t offset_in_block = current_addr % mem->block_size;
        // Сколько байт до конца блока
        uint16_t bytes_to_block_end = mem->block_size - offset_in_block;
        // Читать минимум из: оставшиеся данные или до конца блока
        uint16_t to_read = MIN(n - bytes_read, bytes_to_block_end);

        // 3. Сформировать адрес устройства для multi-block EEPROM
        uint8_t dev_addr = mem->device_address | current_block;

        // 4. Выполнить RANDOM READ используя I2C примитивы (из datasheet):
        //    START → Device Address+W → Word Address →
        //    REPEATED START → Device Address+R → Data... → NACK → STOP

        status = I2C1_Start(timeout);
        if (status != I2C_OK)
            return status;
        status = I2C1_SendAddress(dev_addr, I2C_DIRECTION_WRITE, timeout);
        if (status != I2C_OK)
            return status;

        I2C1_ClearADDR();

        if (mem->mem_addr_size == I2C_MEMADD_SIZE_16BIT) {
            status = I2C1_SendByte((current_addr >> 8) & 0xFF, timeout);
            if (status != I2C_OK)
                return status;
        }
        status = I2C1_SendByte(current_addr & 0xFF, timeout);
        if (status != I2C_OK)
            return status;
        status = I2C1_Start(timeout);
        if (status != I2C_OK)
            return status;
        status = I2C1_SendAddress(dev_addr, I2C_DIRECTION_READ, timeout);
        if (status != I2C_OK)
            return status;
        I2C1_ClearADDR();
        status = I2C1_ReceiveBytes(buffer + bytes_read, to_read, timeout);
        if (status != I2C_OK)
            return status;

        I2C1_Stop(timeout);

        // Переход к следующему блоку
        bytes_read += to_read;
    }

    return I2C_OK;
}
