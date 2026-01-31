#include "i2c.h"
#include "stm32f4xx_hal.h" // Для HAL_GetTick()

/**
 * @brief Проверка таймаута на основе HAL_GetTick()
 * @param start_tick Время начала операции
 * @param timeout Таймаут в миллисекундах
 * @return 1 если таймаут истек, 0 если еще не истек
 */
static inline uint8_t I2C_CheckTimeout(uint32_t start_tick, uint32_t timeout)
{
    return (HAL_GetTick() - start_tick) > timeout;
}

/**
 * @brief Проверка и обработка флагов ошибок I2C
 * @return I2C_Status_t Статус ошибки или I2C_OK
 */
static I2C_Status_t I2C_CheckErrors(void)
{
    uint32_t sr1 = I2C1->SR1;

    if (sr1 & I2C_SR1_BERR) {
        I2C1->SR1 &= ~I2C_SR1_BERR;
        return I2C_ERROR;
    }
    if (sr1 & I2C_SR1_ARLO) {
        I2C1->SR1 &= ~I2C_SR1_ARLO;
        return I2C_ERROR;
    }

    if (sr1 & I2C_SR1_AF) {
        I2C1->SR1 &= ~I2C_SR1_AF;
        return I2C_NACK;
    }
    if (sr1 & I2C_SR1_OVR) {
        I2C1->SR1 &= ~I2C_SR1_OVR;
        return I2C_ERROR;
    }
    return I2C_OK;
}

void I2C1_Init(uint32_t clock_speed)
{
    const uint32_t APB1_FREQ_MHZ = 45;

    // Включить тактирование GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Очистить биты режима для PB6 и PB7
    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    // Установить режим Alternate Function (10) для PB6 и PB7
    GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;

    // Очистить биты альтернативной функции
    GPIOB->AFR[0] &= ~((0xF << 24) | (0xF << 28)); // PB6 и PB7 в AFR[0]
    // Установить AF4 для I2C1
    GPIOB->AFR[0] |= (4 << 24) | (4 << 28);

    // Установить тип выхода Open-Drain
    GPIOB->OTYPER |= (1 << 6) | (1 << 7);

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Выполнить сброс I2C1 перед настройкой (рекомендуется по Reference Manual)
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    // Отключить I2C перед настройкой (обязательно по RM)
    I2C1->CR1 &= ~I2C_CR1_PE;

    // Установить частоту APB1 в регистр CR2 (для правильного timing)
    I2C1->CR2 &= ~I2C_CR2_FREQ;
    I2C1->CR2 |= APB1_FREQ_MHZ; // 45 МГц

    if (clock_speed <= I2C_SPEED_STANDARD) {
        // Standard Mode (до 100 кГц)
        // T_scl = 2 * CCR * T_pclk1
        // CCR = (PCLK1) / (2 * I2C_SPEED)
        // CCR = 45 000 000 / (2 * 100 000) = 225
        uint16_t ccr_value = APB1_FREQ_MHZ * 1000000 / (2 * clock_speed);

        I2C1->CCR &= ~I2C_CCR_FS; // Standard mode
        I2C1->CCR &= ~I2C_CCR_CCR;
        I2C1->CCR |= ccr_value;

        // TRISE = (T_rise_max / T_pclk1) + 1
        // Для Standard Mode: T_rise_max = 1000 ns
        // T_pclk1 = 1 / 45MHz = 22.22 ns
        // TRISE = (1000 / 22.22) + 1 = 46
        I2C1->TRISE = (APB1_FREQ_MHZ * 1000 / 1000) + 1; // 46
    } else {
        // Fast Mode (до 400 кГц)
        // Используем DUTY = 1 (t_low / t_high = 16/9)
        // CCR = (PCLK1) / (25 * I2C_SPEED) для DUTY=1
        // CCR = 45 000 000 / (25 * 400 000) = 4.5 ≈ 5
        uint16_t ccr_value = APB1_FREQ_MHZ * 1000000 / (25 * clock_speed);

        I2C1->CCR |= I2C_CCR_FS;   // Fast mode
        I2C1->CCR |= I2C_CCR_DUTY; // DUTY = 16/9
        I2C1->CCR &= ~I2C_CCR_CCR;
        I2C1->CCR |= ccr_value;

        // TRISE для Fast Mode: T_rise_max = 300 ns
        // TRISE = (300 / 22.22) + 1 = 14
        I2C1->TRISE = (APB1_FREQ_MHZ * 300 / 1000) + 1; // 14
    }

    I2C1->CR1 |= I2C_CR1_PE;
}

// ============================================================================
// Новые примитивные функции I2C
// ============================================================================

I2C_Status_t I2C1_Start(uint32_t timeout)
{
    uint32_t tickstart = HAL_GetTick();
    // При repeated start тут повиснет, так как шина не была освобождена (не было STOP), мб че-то придумать
    /* while (I2C1->SR2 & I2C_SR2_BUSY) {
        if (I2C_CheckTimeout(tickstart, timeout)) {
            return I2C_BUSY;
        }
    } */

    I2C1->CR1 |= I2C_CR1_START;
    tickstart = HAL_GetTick();
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (I2C_CheckTimeout(tickstart, timeout)) {
            return I2C_TIMEOUT;
        }
    }
    return I2C_OK;
}

I2C_Status_t I2C1_Stop(uint32_t timeout)
{
    (void)timeout;
    I2C1->CR1 |= I2C_CR1_STOP;
    return I2C_OK;
}

I2C_Status_t I2C1_SendAddress(uint8_t address, I2C_Direction_t direction, uint32_t timeout)
{
    I2C1->DR = (address << 1) | direction;

    // Ждать ADDR или AF (NACK)
    uint32_t tickstart = HAL_GetTick();
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        I2C_Status_t status = I2C_CheckErrors();
        if (status != I2C_OK) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return status;
        }

        if (I2C_CheckTimeout(tickstart, timeout)) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return I2C_TIMEOUT;
        }
    }

    return I2C_OK;
}

void I2C1_ClearADDR(void)
{
    // Чтение SR1 затем SR2 очищает ADDR
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}

I2C_Status_t I2C1_SendByte(uint8_t data, uint32_t timeout)
{
    uint32_t tickstart = HAL_GetTick();
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {
        if (I2C_CheckTimeout(tickstart, timeout)) {
            return I2C_TIMEOUT;
        }
    }
    I2C1->DR = data;
    return I2C_OK;
}

I2C_Status_t I2C1_SendBytes(uint8_t *data, uint16_t size, uint32_t timeout)
{
    for (uint16_t i = 0; i < size; i++) {
        I2C_Status_t status = I2C1_SendByte(data[i], timeout);
        if (status != I2C_OK) {
            return status;
        }
    }

    return I2C_OK;
}

I2C_Status_t I2C1_ReceiveByte(uint8_t *data, I2C_AckControl_t ack, uint32_t timeout)
{
    if (ack == I2C_ACK_ENABLE) {
        I2C1->CR1 |= I2C_CR1_ACK;
    } else {
        I2C1->CR1 &= ~I2C_CR1_ACK;
    }

    uint32_t tickstart = HAL_GetTick();
    while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
        if (I2C_CheckTimeout(tickstart, timeout)) {
            return I2C_TIMEOUT;
        }
    }
    *data = I2C1->DR;
    return I2C_OK;
}

I2C_Status_t I2C1_ReceiveBytes(uint8_t *data, uint16_t size, uint32_t timeout)
{
    I2C1->CR1 |= I2C_CR1_ACK;

    for (uint16_t i = 0; i < size; i++) {
        // Для последнего байта отправить NACK
        I2C_AckControl_t ack = (i == size - 1) ? I2C_ACK_DISABLE : I2C_ACK_ENABLE;

        I2C_Status_t status = I2C1_ReceiveByte(&data[i], ack, timeout);
        if (status != I2C_OK) {
            return status;
        }
    }

    return I2C_OK;
}

I2C_Status_t I2C1_WaitBTF(uint32_t timeout)
{
    uint32_t tickstart = HAL_GetTick();
    while (!(I2C1->SR1 & I2C_SR1_BTF)) {
        if (I2C_CheckTimeout(tickstart, timeout)) {
            return I2C_TIMEOUT;
        }
    }

    return I2C_OK;
}

I2C_Status_t I2C1_IsDeviceReady(uint8_t dev_addr, uint32_t trials, uint32_t timeout)
{
    uint32_t tickstart;
    I2C_Status_t status;

    for (uint32_t trial = 0; trial < trials; trial++) {
        // Ждать пока шина не освободится
        tickstart = HAL_GetTick();
        while (I2C1->SR2 & I2C_SR2_BUSY) {
            if (I2C_CheckTimeout(tickstart, timeout)) {
                return I2C_BUSY;
            }
        }

        // Генерация START
        I2C1->CR1 |= I2C_CR1_START;

        tickstart = HAL_GetTick();
        while (!(I2C1->SR1 & I2C_SR1_SB)) {
            if (I2C_CheckTimeout(tickstart, timeout)) {
                return I2C_TIMEOUT;
            }
        }

        // Отправить адрес устройства + Write bit
        I2C1->DR = (dev_addr << 1) | 0x00;

        // Ждать ADDR или AF (NACK)
        tickstart = HAL_GetTick();
        while (1) {
            uint32_t sr1 = I2C1->SR1;

            // Если ADDR установлен - устройство ответило (готово)
            if (sr1 & I2C_SR1_ADDR) {
                // Очистить ADDR
                (void)I2C1->SR1;
                (void)I2C1->SR2;

                // Генерация STOP
                I2C1->CR1 |= I2C_CR1_STOP;

                return I2C_OK;
            }

            // Если AF установлен - устройство не ответило (занято записью)
            if (sr1 & I2C_SR1_AF) {
                // Очистить AF
                I2C1->SR1 &= ~I2C_SR1_AF;

                // Генерация STOP
                I2C1->CR1 |= I2C_CR1_STOP;

                // Перейти к следующей попытке
                break;
            }

            // Проверить другие ошибки
            status = I2C_CheckErrors();
            if (status == I2C_ERROR) {
                I2C1->CR1 |= I2C_CR1_STOP;
                return status;
            }

            if (I2C_CheckTimeout(tickstart, timeout)) {
                I2C1->CR1 |= I2C_CR1_STOP;
                break; // Перейти к следующей попытке
            }
        }

        // Небольшая задержка между попытками (1 мс)
        HAL_Delay(1);
    }
    return I2C_TIMEOUT;
}
