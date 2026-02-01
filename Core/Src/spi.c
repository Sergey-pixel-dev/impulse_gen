#include "spi.h"

void SPI1_Init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  GPIOA->MODER &= ~GPIO_MODER_MODER5;
  GPIOA->MODER |= GPIO_MODER_MODER5_1;
  GPIOA->AFR[0] &= ~(0xF << 20);
  GPIOA->AFR[0] |= (5 << 20);
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5;

  GPIOA->MODER &= ~GPIO_MODER_MODER7;
  GPIOA->MODER |= GPIO_MODER_MODER7_1;
  GPIOA->AFR[0] &= ~(0xF << 28);
  GPIOA->AFR[0] |= (5 << 28);
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;

  // SPI1 config: master, 8-bit, software NSS, baud = fPCLK/16
  SPI1->CR1 = 0;
  SPI1->CR1 |=
      SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_BR_1 | SPI_CR1_BR_0;
  SPI1->CR1 |= SPI_CR1_SPE;
}

void SPI1_Transmit(uint8_t *data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    while (!(SPI1->SR & SPI_SR_TXE))
      ;
    SPI1->DR = data[i];
  }
  while (!(SPI1->SR & SPI_SR_TXE))
    ;
  while (SPI1->SR & SPI_SR_BSY)
    ;
}

void SPI1_TransmitByte(uint8_t data) {
  while (!(SPI1->SR & SPI_SR_TXE))
    ;
  SPI1->DR = data;
  while (!(SPI1->SR & SPI_SR_TXE))
    ;
  while (SPI1->SR & SPI_SR_BSY)
    ;
}
