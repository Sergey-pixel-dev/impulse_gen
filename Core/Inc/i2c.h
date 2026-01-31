#ifndef I2C_LIB_H
#define I2C_LIB_H

#include "stm32f4xx.h"
#include <stdint.h>

typedef enum {
    I2C_OK = 0x00,      // Операция успешна
    I2C_ERROR = 0x01,   // Общая ошибка I2C (bus error, arbitration lost, overrun)
    I2C_BUSY = 0x02,    // Шина I2C занята
    I2C_TIMEOUT = 0x03, // Превышен таймаут операции
    I2C_NACK = 0x04     // Получен NACK (устройство не ответило)
} I2C_Status_t;

typedef enum {
    I2C_DIRECTION_WRITE = 0x00, // Запись в устройство
    I2C_DIRECTION_READ = 0x01   // Чтение из устройства
} I2C_Direction_t;

typedef enum {
    I2C_ACK_ENABLE = 0x01, // Отправить ACK после приема байта
    I2C_ACK_DISABLE = 0x00 // Отправить NACK после приема байта (для последнего байта)
} I2C_AckControl_t;

#define I2C_SPEED_STANDARD 100000 // 100 кГц - Standard Mode
#define I2C_SPEED_FAST 400000     // 400 кГц - Fast Mode

/**
 * @brief Инициализация I2C1 с GPIO (PB6=SCL, PB7=SDA)
 *
 * Настраивает GPIO в режим Alternate Function (AF4) с Open-Drain
 * Инициализирует I2C1 с заданной частотой
 *
 * @param clock_speed Частота I2C в Гц (например, 100000 для Standard Mode)
 *
 * @note APB1 должен быть настроен на 45 МГц (для правильного расчета CCR)
 * @note Open-Drain обязателен для I2C, pull-up рекомендуются
 */
void I2C1_Init(uint32_t clock_speed);

/**
 * @brief Генерация START condition
 *
 * Проверяет что шина не занята, генерирует START и ждет флага SB
 *
 * @param timeout Таймаут в миллисекундах
 * @return I2C_Status_t Статус операции
 *
 * @note START condition начинает новую передачу на шине I2C
 */
I2C_Status_t I2C1_Start(uint32_t timeout);

/**
 * @brief Генерация STOP condition
 *
 * Завершает текущую передачу на шине I2C
 *
 * @param timeout Таймаут в миллисекундах (не используется, для совместимости)
 * @return I2C_Status_t Всегда I2C_OK
 *
 * @note STOP condition освобождает шину I2C
 */
I2C_Status_t I2C1_Stop(uint32_t timeout);

/**
 * @brief Отправка адреса устройства I2C
 *
 * Отправляет 7-битный адрес устройства + бит R/W и ждет ACK
 *
 * @param address 7-битный адрес устройства (без сдвига)
 * @param direction Направление: I2C_DIRECTION_WRITE или I2C_DIRECTION_READ
 * @param timeout Таймаут в миллисекундах
 * @return I2C_Status_t I2C_OK или I2C_NACK если устройство не ответило
 *
 * @note Адрес автоматически сдвигается влево и добавляется бит R/W
 */
I2C_Status_t I2C1_SendAddress(uint8_t address, I2C_Direction_t direction, uint32_t timeout);

/**
 * @brief Отправка одного байта данных
 *
 * Ждет TXE и отправляет байт в регистр DR
 *
 * @param data Байт данных для отправки
 * @param timeout Таймаут в миллисекундах
 * @return I2C_Status_t Статус операции
 */
I2C_Status_t I2C1_SendByte(uint8_t data, uint32_t timeout);

/**
 * @brief Отправка массива байтов
 *
 * Циклически отправляет несколько байт подряд
 *
 * @param data Указатель на буфер данных
 * @param size Количество байт для отправки
 * @param timeout Таймаут в миллисекундах
 * @return I2C_Status_t Статус операции
 *
 * @note Использует I2C1_SendByte() для каждого байта
 */
I2C_Status_t I2C1_SendBytes(uint8_t *data, uint16_t size, uint32_t timeout);

/**
 * @brief Прием одного байта данных
 *
 * Настраивает ACK/NACK, ждет RXNE и читает байт из DR
 *
 * @param data Указатель для сохранения принятого байта
 * @param ack I2C_ACK_ENABLE или I2C_ACK_DISABLE (для последнего байта)
 * @param timeout Таймаут в миллисекундах
 * @return I2C_Status_t Статус операции
 *
 * @note Для последнего байта используйте I2C_ACK_DISABLE
 */
I2C_Status_t I2C1_ReceiveByte(uint8_t *data, I2C_AckControl_t ack, uint32_t timeout);

/**
 * @brief Прием массива байтов
 *
 * Циклически принимает несколько байт с автоматическим управлением ACK/NACK
 *
 * @param data Указатель на буфер для приема данных
 * @param size Количество байт для приема
 * @param timeout Таймаут в миллисекундах
 * @return I2C_Status_t Статус операции
 *
 * @note Автоматически отправляет NACK на последнем байте
 */
I2C_Status_t I2C1_ReceiveBytes(uint8_t *data, uint16_t size, uint32_t timeout);

/**
 * @brief Очистка флага ADDR
 *
 * Выполняет последовательность чтения SR1 и SR2 для очистки флага ADDR
 *
 * @note Это специфика STM32: ADDR очищается чтением SR1 затем SR2
 * @note Должна вызываться после SendAddress() перед началом передачи данных
 */
void I2C1_ClearADDR(void);

/**
 * @brief Ожидание завершения передачи байта (флаг BTF)
 *
 * Ждет пока флаг BTF установится (данные переданы и сдвиговый регистр пуст)
 *
 * @param timeout Таймаут в миллисекундах
 * @return I2C_Status_t Статус операции
 *
 * @note BTF=1 означает что и DR и shift register пусты (передача завершена)
 */
I2C_Status_t I2C1_WaitBTF(uint32_t timeout);

/**
 * @brief Проверка готовности устройства I2C (ACK Polling)
 *
 * Циклически отправляет адрес устройства и проверяет ACK
 * Используется для ожидания завершения записи в EEPROM
 *
 * @param address 7-битный адрес устройства (без сдвига)
 * @param trials Количество попыток опроса
 * @param timeout Таймаут на одну попытку в миллисекундах
 * @return I2C_Status_t I2C_OK если устройство ответило, иначе ошибка
 *
 * @note Типичное время записи страницы EEPROM: 5 мс
 * @note Рекомендуется: trials=50, timeout=1 (итого до 50 мс)
 */
I2C_Status_t I2C1_IsDeviceReady(uint8_t address, uint32_t trials, uint32_t timeout);

#endif // I2C_LIB_H
