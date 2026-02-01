#include "signal_gen.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

uint16_t dac_buffer[DAC_BUFFER_SIZE];

void SignalGen_Init(void) {
  // Default: sine, 500Hz, amplitude=1024, offset=2048
  SignalGen_Sine(dac_buffer, DAC_BUFFER_SIZE, 1024, 2048);
}

void SignalGen_Sine(uint16_t *buf, uint16_t n_points, uint16_t amplitude,
                    uint16_t offset) {
  for (uint16_t i = 0; i < n_points; i++) {
    double angle = 2.0 * M_PI * i / n_points;
    int32_t val = (int32_t)(offset + amplitude * sin(angle));
    if (val < 0)
      val = 0;
    if (val > 4095)
      val = 4095;
    buf[i] = (uint16_t)val;
  }
}

void SignalGen_Square(uint16_t *buf, uint16_t n_points, uint16_t amplitude,
                      uint16_t offset) {
  for (uint16_t i = 0; i < n_points; i++) {
    int32_t val;
    if (i < n_points / 2) {
      val = offset + amplitude;
    } else {
      val = offset - amplitude;
    }
    if (val < 0)
      val = 0;
    if (val > 4095)
      val = 4095;
    buf[i] = (uint16_t)val;
  }
}

void SignalGen_Triangle(uint16_t *buf, uint16_t n_points, uint16_t amplitude,
                        uint16_t offset) {
  uint16_t half = n_points / 2;
  for (uint16_t i = 0; i < n_points; i++) {
    int32_t val;
    if (i < half) {
      val = offset - amplitude + (2 * amplitude * i) / half;
    } else {
      val = offset + amplitude - (2 * amplitude * (i - half)) / half;
    }
    if (val < 0)
      val = 0;
    if (val > 4095)
      val = 4095;
    buf[i] = (uint16_t)val;
  }
}

void SignalGen_Sawtooth(uint16_t *buf, uint16_t n_points, uint16_t amplitude,
                        uint16_t offset) {
  for (uint16_t i = 0; i < n_points; i++) {
    int32_t val = offset - amplitude + (2 * amplitude * i) / n_points;
    if (val < 0)
      val = 0;
    if (val > 4095)
      val = 4095;
    buf[i] = (uint16_t)val;
  }
}

void SignalGen_FromRecord(uint16_t *raw_points, uint8_t n_points) {
  for (uint8_t i = 0; i < n_points && i < DAC_BUFFER_SIZE; i++) {
    uint16_t val = raw_points[i];
    if (val > 4095)
      val = 4095;
    dac_buffer[i] = val;
  }
}

void CalcTimerParams(uint32_t freq_hz, uint16_t n_points, uint16_t *psc,
                     uint16_t *arr) {
  // Stub: PSC=2, ARR=1800
  (void)freq_hz;
  (void)n_points;
  *psc = 2;
  *arr = 1800;
}
