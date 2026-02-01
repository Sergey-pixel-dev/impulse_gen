#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>

#define NO_SELECTION 0xFF

void Display_Init(void);
void Display_UpdateFreq(uint8_t *freq_arr, uint8_t sel_digit, uint8_t is_selected_param);
void Display_UpdateAmplitude(uint8_t *amp_arr, uint8_t sel_digit, uint8_t is_selected_param);
void Display_UpdateOffset(uint8_t *off_arr, uint8_t sel_digit, uint8_t is_selected_param);
void Display_UpdateRecordName(const char *name);

#endif // DISPLAY_H
