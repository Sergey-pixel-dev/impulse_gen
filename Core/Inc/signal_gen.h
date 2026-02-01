#ifndef SIGNAL_GEN_H
#define SIGNAL_GEN_H

#include <stdint.h>

#define DAC_BUFFER_SIZE 50

extern uint16_t dac_buffer[DAC_BUFFER_SIZE];

void SignalGen_Init(void);
void SignalGen_Sine(uint16_t *buf, uint16_t n_points, uint16_t amplitude, uint16_t offset);
void SignalGen_Square(uint16_t *buf, uint16_t n_points, uint16_t amplitude, uint16_t offset);
void SignalGen_Triangle(uint16_t *buf, uint16_t n_points, uint16_t amplitude, uint16_t offset);
void SignalGen_Sawtooth(uint16_t *buf, uint16_t n_points, uint16_t amplitude, uint16_t offset);
void SignalGen_FromRecord(uint16_t *raw_points, uint8_t n_points);
void CalcTimerParams(uint32_t freq_hz, uint16_t n_points, uint16_t *psc, uint16_t *arr);

#endif // SIGNAL_GEN_H
