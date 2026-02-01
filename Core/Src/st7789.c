/**
 * @file : ST7789.c
 * @brief : ST7789 driver for STM32F103C8T6
 */

/* Includes ------------------------------------------------------------------*/
#include "st7789.h"
#include <string.h>

/* Declarations and definitions ----------------------------------------------*/
static uint16_t CurrentRow = 0;
static uint16_t CurrentCol = 0;
static uint8_t CurrentFontSize = 1; // Default font size

/* Functions -----------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static void ST7789_SendByte(uint8_t data) {
  while ((SPI1->SR & SPI_SR_TXE) == RESET) {
  }
  SPI1->DR = data;
}

/*----------------------------------------------------------------------------*/
static void ST7789_WaitLastData(void) {
  while ((SPI1->SR & SPI_SR_TXE) == RESET) {
  }
  while ((SPI1->SR & SPI_SR_BSY) != RESET) {
  }
}

/*----------------------------------------------------------------------------*/
static void ST7789_SendCommand(uint8_t cmd) {
  ST7789_Select();
  HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_RESET);
  ST7789_SendByte(cmd);
  ST7789_WaitLastData();
  ST7789_UnSelect();
}

/*----------------------------------------------------------------------------*/
static void ST7789_SendData(uint8_t data) {
  ST7789_Select();
  HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_SET);
  ST7789_SendByte(data);
  ST7789_WaitLastData();
  ST7789_UnSelect();
}

/*----------------------------------------------------------------------------*/
static void ST7789_SendDataMultiple(const uint8_t *data, uint32_t num) {
  ST7789_Select();
  HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_SET);

  for (uint32_t i = 0; i < num; i++) {
    ST7789_SendByte(data[i]);
  }

  ST7789_WaitLastData();
  ST7789_UnSelect();
}

/*----------------------------------------------------------------------------*/
void ST7789_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1,
                             uint16_t y1) {
  ST7789_Select();

  uint16_t x_start = x0 + X_SHIFT, x_end = x1 + X_SHIFT;
  uint16_t y_start = y0 + Y_SHIFT, y_end = y1 + Y_SHIFT;

  /* Column Address set */
  ST7789_SendCommand(ST7789_CASET);
  {
    uint8_t data[] = {(x_start >> 8) & 0xFF, x_start & 0xFF,
                      (x_end >> 8) & 0xFF, x_end & 0xFF};
    ST7789_SendDataMultiple(data, sizeof(data));
  }

  /* Row Address set */
  ST7789_SendCommand(ST7789_RASET);
  {
    uint8_t data[] = {(y_start >> 8) & 0xFF, y_start & 0xFF,
                      (y_end >> 8) & 0xFF, y_end & 0xFF};
    ST7789_SendDataMultiple(data, sizeof(data));
  }

  /* Write to RAM */
  ST7789_SendCommand(ST7789_RAMWR);
  ST7789_UnSelect();
}

/*----------------------------------------------------------------------------*/
void ST7789_SetRotation(uint8_t m) {
  ST7789_SendCommand(ST7789_MADCTL);

  switch (m) {
  case 0:
    ST7789_SendData(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);
    break;
  case 1:
    ST7789_SendData(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
    break;
  case 2:
    ST7789_SendData(ST7789_MADCTL_RGB);
    break;
  case 3:
    ST7789_SendData(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
    break;
  default:
    break;
  }
}

/*----------------------------------------------------------------------------*/
void ST7789_Init() {
  SPI1->CR1 |= SPI_CR1_SPE;
  HAL_GPIO_WritePin(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(5);

  ST7789_SendCommand(0x01);
  HAL_Delay(150);
  ST7789_SendCommand(0x11);
  HAL_Delay(500);
  ST7789_SendCommand(0xB1);
  ST7789_SendData(0x01);
  ST7789_SendData(0x2C);
  ST7789_SendData(0x2D);
  ST7789_SendCommand(0xB2);
  ST7789_SendData(0x01);
  ST7789_SendData(0x2C);
  ST7789_SendData(0x2D);
  ST7789_SendCommand(0xB3);
  ST7789_SendData(0x01);
  ST7789_SendData(0x2C);
  ST7789_SendData(0x2D);
  ST7789_SendData(0x01);
  ST7789_SendData(0x2C);
  ST7789_SendData(0x2D);
  ST7789_SendCommand(0xB4);
  ST7789_SendData(0x07);
  ST7789_SendCommand(0xC0);
  ST7789_SendData(0xA2);
  ST7789_SendData(0x02);
  ST7789_SendData(0x84);
  ST7789_SendCommand(0xC1);
  ST7789_SendData(0xC5);
  ST7789_SendCommand(0xC2);
  ST7789_SendData(0x0A);
  ST7789_SendData(0x00);
  ST7789_SendCommand(0xC3);
  ST7789_SendData(0x8A);
  ST7789_SendData(0x2A);
  ST7789_SendCommand(0xC4);
  ST7789_SendData(0x8A);
  ST7789_SendData(0xEE);
  ST7789_SendCommand(0xC5);
  ST7789_SendData(0x0E);
  ST7789_SendCommand(0x20);
  ST7789_SendCommand(0x36);
  ST7789_SendData(0xC0);
  ST7789_SendCommand(0x3A);
  ST7789_SendData(0x05);
  ST7789_SendCommand(0xE0);
  ST7789_SendData(0x02);
  ST7789_SendData(0x1c);
  ST7789_SendData(0x07);
  ST7789_SendData(0x12);
  ST7789_SendData(0x37);
  ST7789_SendData(0x32);
  ST7789_SendData(0x29);
  ST7789_SendData(0x2d);
  ST7789_SendData(0x29);
  ST7789_SendData(0x25);
  ST7789_SendData(0x2B);
  ST7789_SendData(0x39);
  ST7789_SendData(0x00);
  ST7789_SendData(0x01);
  ST7789_SendData(0x03);
  ST7789_SendData(0x10);
  ST7789_SendCommand(0xE1);
  ST7789_SendData(0x03);
  ST7789_SendData(0x1d);
  ST7789_SendData(0x07);
  ST7789_SendData(0x06);
  ST7789_SendData(0x2E);
  ST7789_SendData(0x2C);
  ST7789_SendData(0x29);
  ST7789_SendData(0x2D);
  ST7789_SendData(0x2E);
  ST7789_SendData(0x2E);
  ST7789_SendData(0x37);
  ST7789_SendData(0x3F);
  ST7789_SendData(0x00);
  ST7789_SendData(0x00);
  ST7789_SendData(0x02);
  ST7789_SendData(0x10);
  ST7789_SendCommand(ST7789_MADCTL); // меняем вывод цветов с GBR на RGB
  ST7789_SendData(0b00001000);
  ST7789_SendCommand(ST7789_NORON);
  HAL_Delay(10);
  ST7789_SendCommand(ST7789_DISPON);
  HAL_Delay(100);

  HAL_GPIO_WritePin(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_SET);
}

/*----------------------------------------------------------------------------*/
void ST7789_Fill_Color(uint16_t color) {
  ST7789_SetAddressWindow(0, 0, ST7789_WIDTH - 1, ST7789_HEIGHT - 1);

  ST7789_Select();
  HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_SET);

  uint8_t color_high = (color >> 8) & 0xFF;
  uint8_t color_low = color & 0xFF;

  for (uint32_t i = 0; i < ST7789_WIDTH * ST7789_HEIGHT; i++) {
    ST7789_SendByte(color_high);
    ST7789_SendByte(color_low);
  }

  ST7789_WaitLastData();
  ST7789_UnSelect();
}

/*----------------------------------------------------------------------------*/
void ST7789_DrawRect(uint16_t cStart, uint16_t rStart, uint16_t cStop,
                     uint16_t rStop, uint16_t color) {
  if (cStart >= ST7789_WIDTH || rStart >= ST7789_HEIGHT ||
      cStop > ST7789_WIDTH || rStop > ST7789_HEIGHT)
    return;

  ST7789_SetAddressWindow(cStart, rStart, cStop - 1, rStop - 1);

  ST7789_Select();
  HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_SET);

  uint8_t color_high = (color >> 8) & 0xFF;
  uint8_t color_low = color & 0xFF;
  uint32_t size = (cStop - cStart) * (rStop - rStart);

  for (uint32_t i = 0; i < size; i++) {
    ST7789_SendByte(color_high);
    ST7789_SendByte(color_low);
  }

  ST7789_WaitLastData();
  ST7789_UnSelect();

  CurrentCol = cStop;
  CurrentRow = rStop - ST7789_FONT_SIZE * CurrentFontSize + 1;
}

/*----------------------------------------------------------------------------*/
void ST7789_PrintSymbol(uint16_t cStart, uint16_t rStart, uint8_t sb,
                        uint16_t color, uint16_t bg_color, uint8_t font_size) {
  if (font_size == 0)
    font_size = CurrentFontSize;

  uint8_t char_index = GetFontCharIndex(sb);
  if (char_index == 0xFF)
    return;

  uint8_t char_width = GetFontCharWidth(sb);

  uint16_t data_offset = FONT_CHAR_INFO[char_index][1];

  if (cStart == 0xFF && rStart == 0xFF) {
    cStart = CurrentCol;
    rStart = CurrentRow;
  }

  uint16_t font_height = ST7789_FONT_SIZE * font_size;
  ST7789_SetAddressWindow(cStart, rStart, cStart + (char_width * font_size) - 1,
                          rStart + font_height - 1);

  uint8_t color_high = (color >> 8) & 0xFF;
  uint8_t color_low = color & 0xFF;
  uint8_t bg_color_high = (bg_color >> 8) & 0xFF;
  uint8_t bg_color_low = bg_color & 0xFF;

  ST7789_Select();
  HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_SET);

  // Отрисовка символа с масштабированием
  for (int r = 0; r < ST7789_FONT_SIZE; r++) {
    // Повторяем каждую строку font_size раз для вертикального масштабирования
    for (int row_repeat = 0; row_repeat < font_size; row_repeat++) {
      for (int col = 0; col < char_width; col++) {
        // Получаем текущий байт данных шрифта
        uint8_t byte = FONT_DATA[data_offset + r];

        // Проверяем бит для текущего пикселя (слева направо)
        uint8_t pixel = (byte & (0x80 >> col)) ? 1 : 0;

        // Горизонтальное масштабирование
        for (int col_repeat = 0; col_repeat < font_size; col_repeat++) {
          if (pixel) {
            ST7789_SendByte(color_high);
            ST7789_SendByte(color_low);
          } else {
            ST7789_SendByte(bg_color_high);
            ST7789_SendByte(bg_color_low);
          }
        }
      }
    }
  }

  ST7789_WaitLastData();
  ST7789_UnSelect();

  // Обновление текущей позиции
  CurrentCol = cStart + (char_width + 1) * font_size;
}

/*----------------------------------------------------------------------------*/
void ST7789_PrintStr(uint16_t cStart, uint16_t rStart, const char *str,
                     uint8_t num, uint16_t color, uint16_t bg_color,
                     uint8_t font_size) {
  if (font_size == 0)
    font_size = CurrentFontSize;

  if (cStart == 0xFF && rStart == 0xFF) {
    cStart = CurrentCol;
    rStart = CurrentRow;
  }

  for (int i = 0; i < num; i++) {
    ST7789_PrintSymbol(cStart, rStart, str[i], color, bg_color, font_size);
    cStart = CurrentCol;
  }
}
/*----------------------------------------------------------------------------*/
void ST7789_PrintCustomStr(uint16_t cStart, uint16_t rStart, uint8_t *str,
                           uint8_t len, uint8_t pos, uint16_t color,
                           uint16_t pos_color, uint16_t bg_color,
                           uint8_t font_size) {
  if (font_size == 0)
    font_size = CurrentFontSize;

  for (int i = 0; i < len; i++) {
    if (i == pos) {

      ST7789_PrintSymbol(cStart, rStart, str[i], pos_color, bg_color,
                         font_size);
    } else {
      ST7789_PrintSymbol(cStart, rStart, str[i], color, bg_color, font_size);
    }
    cStart = CurrentCol; // исключительной сейчас;
  }
}

/*----------------------------------------------------------------------------*/
void ST7789_PrintNumber(uint16_t cStart, uint16_t rStart, uint16_t Number,
                        uint16_t color, uint16_t bg_color, uint8_t font_size) {
  if (font_size == 0)
    font_size = CurrentFontSize;

  // Обработка нуля
  if (Number == 0) {
    ST7789_PrintSymbol(cStart, rStart, '0', color, bg_color, font_size);
    return;
  }

  // Подсчет цифр
  uint8_t len = 0;
  uint16_t temp = Number;
  while (temp > 0) {
    len++;
    temp /= 10;
  }

  // Преобразование в строку
  char NumArray[10]; // Максимум 5 цифр для uint16_t
  int i = len - 1;

  temp = Number;
  while (temp > 0) {
    NumArray[i--] = '0' + (temp % 10);
    temp /= 10;
  }

  // Печать числа
  ST7789_PrintStr(cStart, rStart, NumArray, len, color, bg_color, font_size);
}

/*----------------------------------------------------------------------------*/
void ST7789_PrintStrFieldWidth(uint16_t cStart, uint16_t rStart,
                               const char *str, uint8_t len, int8_t WidthField,
                               uint8_t Symbol, uint16_t color,
                               uint16_t bg_color, uint8_t font_size) {
  if (font_size == 0)
    font_size = CurrentFontSize;

  if (WidthField > 0 && len <= WidthField) {
    // Выравнивание по правому краю
    char NewStr[32]; // Ограниченный размер буфера

    // Заполнение отступами
    for (uint8_t i = 0; i < WidthField - len; i++) {
      NewStr[i] = Symbol;
    }

    // Копирование строки
    for (uint8_t i = 0; i < len; i++) {
      NewStr[WidthField - len + i] = str[i];
    }

    ST7789_PrintStr(cStart, rStart, NewStr, WidthField, color, bg_color,
                    font_size);
  } else if (WidthField < 0 && len <= (-WidthField)) {
    // Выравнивание по левому краю
    char NewStr[32]; // Ограниченный размер буфера

    // Копирование строки
    for (uint8_t i = 0; i < len; i++) {
      NewStr[i] = str[i];
    }

    // Заполнение отступами
    for (uint8_t i = len; i < -WidthField; i++) {
      NewStr[i] = Symbol;
    }

    ST7789_PrintStr(cStart, rStart, NewStr, -WidthField, color, bg_color,
                    font_size);
  } else {
    // Ширина поля слишком мала, просто печатаем строку
    ST7789_PrintStr(cStart, rStart, str, len, color, bg_color, font_size);
  }
}

/*----------------------------------------------------------------------------*/
void ST7789_PrintNumberFieldWidth(uint16_t cStart, uint16_t rStart,
                                  uint16_t Number, int8_t WidthField,
                                  uint8_t Symbol, uint16_t color,
                                  uint16_t bg_color, uint8_t font_size) {
  if (font_size == 0)
    font_size = CurrentFontSize;

  // Обработка нуля
  if (Number == 0) {
    char zero[] = "0";
    ST7789_PrintStrFieldWidth(cStart, rStart, zero, 1, WidthField, Symbol,
                              color, bg_color, font_size);
    return;
  }

  // Подсчет цифр
  uint8_t len = 0;
  uint16_t temp = Number;
  while (temp > 0) {
    len++;
    temp /= 10;
  }

  // Преобразование в строку
  char NumArray[10];
  int i = len - 1;
  temp = Number;
  while (temp > 0) {
    NumArray[i--] = '0' + (temp % 10);
    temp /= 10;
  }

  ST7789_PrintStrFieldWidth(cStart, rStart, NumArray, len, WidthField, Symbol,
                            color, bg_color, font_size);
}

/*----------------------------------------------------------------------------*/
void ST7789_EnterSLEEPMode(void) { ST7789_SendCommand(ST7789_SLPIN); }

/*----------------------------------------------------------------------------*/
void ST7789_WakeUpSLEEPMode(void) { ST7789_SendCommand(ST7789_SLPOUT); }

/*----------------------------------------------------------------------------*/
void ST7789_SetFontSize(uint8_t size) {
  if (size > 0) {
    CurrentFontSize = size;
  }
}

/*----------------------------------------------------------------------------*/
uint8_t ST7789_GetFontSize(void) { return CurrentFontSize; }
