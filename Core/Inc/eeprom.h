#ifndef EEPROM_LIB
#define EEPROM_LIB

#include "i2c.h"
#include "stm32f4xx.h"
#include <stdint.h>

#define I2C_MEMADD_SIZE_8BIT 1  // 1-байтный адрес памяти (для EEPROM до 2 Кбит)
#define I2C_MEMADD_SIZE_16BIT 2 // 2-байтный адрес памяти (для EEPROM >= 4 Кбит)

#define MIN(a, b) ((a) < (b) ? (a) : (b))

typedef I2C_Status_t EEPROM_Status_t;

// Структура EEPROM (универсальная для различных моделей 24Cxx)
typedef struct {
    uint8_t device_address; // 7-битный базовый адрес (например, 0x50 для 24Cxx)
    uint16_t block_size;    // Размер блока в байтах (например, 256, 4096)
    uint16_t page_size;     // Размер страницы в байтах (например, 8, 16, 32, 64)
    uint16_t block_count;   // Количество блоков (для multi-block EEPROM)
    uint16_t page_count;    // Количество страниц в одном блоке (block_size / page_size)
    uint8_t mem_addr_size;  // Размер адреса: I2C_MEMADD_SIZE_8BIT или I2C_MEMADD_SIZE_16BIT
} EEPROM_t;

/**
 * @brief Инициализация структуры EEPROM
 *
 * @param mem Указатель на структуру EEPROM
 * @param dev_addr 7-битный базовый адрес устройства (например, 0x50)
 * @param block_size Размер блока в байтах
 * @param block_count Количество блоков
 * @param page_size Размер страницы в байтах
 * @param mem_addr_size Размер адреса: I2C_MEMADD_SIZE_8BIT или 16BIT
 *
 * @note Примеры:
 *       24C02:  256B,  8B page,  1 block, 8-bit  addr
 *       24C32:  4KB,  32B page,  1 block, 16-bit addr
 *       24C64:  8KB,  32B page,  1 block, 16-bit addr
 *       24C256: 32KB, 64B page,  1 block, 16-bit addr
 */
void EEPROM_Init(EEPROM_t *mem, uint8_t dev_addr, uint16_t block_size, uint16_t block_count, uint16_t page_size,
                 uint8_t mem_addr_size);

/**
 * @brief Запись данных в EEPROM с автоматической обработкой границ страниц
 *
 * Автоматически разбивает данные на страницы и использует ACK polling
 * для ожидания завершения записи каждой страницы
 *
 * @param mem Указатель на структуру EEPROM
 * @param addr Начальный адрес записи
 * @param buffer Указатель на буфер данных
 * @param n Количество байт для записи
 * @param timeout Таймаут в миллисекундах
 *
 * @return EEPROM_Status_t Статус операции
 */
EEPROM_Status_t EEPROM_Write(EEPROM_t *mem, uint16_t addr, uint8_t *buffer, uint32_t n, uint32_t timeout);

/**
 * @brief Чтение данных из EEPROM с автоматической обработкой границ блоков
 *
 * @param mem Указатель на структуру EEPROM
 * @param addr Начальный адрес чтения
 * @param buffer Указатель на буфер для приема данных
 * @param n Количество байт для чтения
 * @param timeout Таймаут в миллисекундах
 *
 * @return EEPROM_Status_t Статус операции
 */
EEPROM_Status_t EEPROM_Read(EEPROM_t *mem, uint16_t addr, uint8_t *buffer, uint32_t n, uint32_t timeout);

#endif // EEPROM_LIB
