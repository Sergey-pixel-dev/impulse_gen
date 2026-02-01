/**
 * @file : ST7789.h
 * @brief : ST7789 driver for STM32F103C8T6
 */

#ifndef ST7789_H
#define ST7789_H

/* Includes ------------------------------------------------------------------*/
#include "fonts.h"
#include "stm32f4xx_hal.h"

/* Declarations and definitions ----------------------------------------------*/
/* SPI1 used via direct register access (spi.h) */

/**
 * Цвета в формате RGB565
 */
#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define DARKGREEN 0x0300
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define GRAY 0x8430
#define BRED 0xF81F
#define GRED 0xFFE0
#define GBLUE 0x07FF
#define BROWN 0xBC40
#define BRRED 0xFC07
#define DARKBLUE 0x01CF
#define LIGHTBLUE 0x7D7C
#define GRAYBLUE 0x5458
#define LIGHTGREEN 0x841F
#define LGRAY 0xC618
#define LGRAYBLUE 0xA651
#define LBBLUE 0x2B12

/* Регистры и коды команд */
#define ST7789_NOP 0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID 0x04
#define ST7789_RDDST 0x09
#define ST7789_SLPIN 0x10
#define ST7789_SLPOUT 0x11
#define ST7789_PTLON 0x12
#define ST7789_NORON 0x13
#define ST7789_INVOFF 0x20
#define ST7789_INVON 0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON 0x29
#define ST7789_CASET 0x2A
#define ST7789_RASET 0x2B
#define ST7789_RAMWR 0x2C
#define ST7789_RAMRD 0x2E
#define ST7789_PTLAR 0x30
#define ST7789_COLMOD 0x3A
#define ST7789_MADCTL 0x36

/* Конфигурация аппаратной части */
#define ST7789_RST_PORT GPIOB
#define ST7789_RST_PIN GPIO_PIN_14
#define ST7789_CS_PORT GPIOB
#define ST7789_CS_PIN GPIO_PIN_13
#define ST7789_DC_PORT GPIOB
#define ST7789_DC_PIN GPIO_PIN_15

#define ST7789_SPI_TIMEOUT 100

/* Расширенные настройки */
#define ST7789_COLOR_MODE_16bit 0x55 // RGB565 (16bit)
#define ST7789_COLOR_MODE_18bit 0x66 // RGB666 (18bit)
#define ST7789_FONT_SIZE 8

/* Биты регистра MADCTL */
#define ST7789_MADCTL_MY 0x80  // Page Address Order
#define ST7789_MADCTL_MX 0x40  // Column Address Order
#define ST7789_MADCTL_MV 0x20  // Page/Column Order
#define ST7789_MADCTL_ML 0x10  // Line Address Order
#define ST7789_MADCTL_RGB 0x00 // RGB Order

/* Размеры дисплея */
#define ST7789_WIDTH 128  // 240
#define ST7789_HEIGHT 160 // 320

/* Значения смещения для позиционирования */
#define X_SHIFT 0
#define Y_SHIFT 0

/* Базовые операции */
#define ST7789_RST_Clr()                                                       \
  HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_RESET)
#define ST7789_RST_Set()                                                       \
  HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_SET)
#define ST7789_Select()                                                        \
  HAL_GPIO_WritePin(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_RESET)
#define ST7789_UnSelect()                                                      \
  HAL_GPIO_WritePin(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_SET)

/* Прототипы функций */
void ST7789_Init(void);
void ST7789_SetRotation(uint8_t m);
void ST7789_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1,
                             uint16_t y1);
void ST7789_Fill_Color(uint16_t color);
void ST7789_DrawRect(uint16_t cStart, uint16_t rStart, uint16_t cStop,
                     uint16_t rStop, uint16_t color);

/* Функции работы с текстом и шрифтами */
void ST7789_SetFontSize(uint8_t size);
uint8_t ST7789_GetFontSize(void);
void ST7789_PrintSymbol(uint16_t cStart, uint16_t rStart, uint8_t sb,
                        uint16_t color, uint16_t bg_color, uint8_t font_size);
void ST7789_PrintStr(uint16_t cStart, uint16_t rStart, const char *str,
                     uint8_t num, uint16_t color, uint16_t bg_color,
                     uint8_t font_size);
void ST7789_PrintCustomStr(uint16_t cStart, uint16_t rStart, uint8_t *str,
                           uint8_t buffer, uint8_t pos, uint16_t color,
                           uint16_t pos_color, uint16_t bg_color,
                           uint8_t font_size);
void ST7789_PrintNumber(uint16_t cStart, uint16_t rStart, uint16_t Number,
                        uint16_t color, uint16_t bg_color, uint8_t font_size);
void ST7789_PrintStrFieldWidth(uint16_t cStart, uint16_t rStart,
                               const char *str, uint8_t len, int8_t WidthField,
                               uint8_t Symbol, uint16_t color,
                               uint16_t bg_color, uint8_t font_size);
void ST7789_PrintNumberFieldWidth(uint16_t cStart, uint16_t rStart,
                                  uint16_t Number, int8_t WidthField,
                                  uint8_t Symbol, uint16_t color,
                                  uint16_t bg_color, uint8_t font_size);

/* Управление питанием */
void ST7789_EnterSLEEPMode(void);
void ST7789_WakeUpSLEEPMode(void);

#endif /* ST7789_H */
