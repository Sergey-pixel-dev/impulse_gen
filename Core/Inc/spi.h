#ifndef SPI_LIB_H
#define SPI_LIB_H

#include "stm32f4xx.h"
#include <stdint.h>

void SPI1_Init(void);
void SPI1_Transmit(uint8_t *data, uint16_t len);
void SPI1_TransmitByte(uint8_t data);

#endif // SPI_LIB_H
