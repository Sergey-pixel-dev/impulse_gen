/**
 * @file : fonts.h
 * @brief : Font definitions for ST7789 driver
 */

#ifndef FONTS_H
#define FONTS_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Font definitions */
#define FONT_SIZE_1 8 // Базовый размер шрифта в пикселях

/* Font data structures */
extern const uint8_t FONT_DATA[];          // Массив с битовыми данными шрифтов
extern const uint16_t FONT_CHAR_INFO[][2]; // Информация о символах [0]-ширина, [1]-индекс

/* Function prototypes */
uint8_t GetFontCharWidth(char c);
uint8_t GetFontCharIndex(char c);

#endif /* FONTS_H */
