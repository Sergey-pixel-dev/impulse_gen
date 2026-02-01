#include "display.h"
#include "st7789.h"

#define DISPLAY_FONT_SIZE 2
#define LINE_H (8 * DISPLAY_FONT_SIZE)
#define BLOCK_GAP 6

#define COL_FREQ_LABEL 4
#define COL_FREQ_VALUE 50
#define ROW_FREQ_LABEL 4
#define ROW_FREQ_VALUE 24

#define COL_AMP_LABEL 4
#define COL_AMP_VALUE 66
#define ROW_AMP_LABEL 42
#define ROW_AMP_VALUE 62

#define COL_OFF_LABEL 4
#define COL_OFF_VALUE 66
#define ROW_OFF_LABEL 80
#define ROW_OFF_VALUE 100

#define COL_REC_LABEL 4
#define COL_REC_VALUE 10
#define ROW_REC_LABEL 118
#define ROW_REC_VALUE 138

void Display_Init(void) {
  ST7789_Fill_Color(WHITE);
  ST7789_PrintStr(COL_FREQ_LABEL, ROW_FREQ_LABEL, "Freq:", 5, BLACK, WHITE,
                  DISPLAY_FONT_SIZE);
  ST7789_PrintStr(COL_AMP_LABEL, ROW_AMP_LABEL, "Amp:", 4, BLACK, WHITE,
                  DISPLAY_FONT_SIZE);
  ST7789_PrintStr(COL_OFF_LABEL, ROW_OFF_LABEL, "Offs:", 5, BLACK, WHITE,
                  DISPLAY_FONT_SIZE);
  ST7789_PrintStr(COL_REC_LABEL, ROW_REC_LABEL, "Rec:", 4, BLACK, WHITE,
                  DISPLAY_FONT_SIZE);
}

static void DrawParamDigits(uint16_t col, uint16_t row, uint8_t *arr,
                            uint8_t len, uint8_t sel_digit,
                            uint8_t is_selected_param) {
  uint16_t base_color = is_selected_param ? GREEN : BLACK;
  for (uint8_t i = 0; i < len; i++) {
    uint16_t color =
        (is_selected_param && i == sel_digit && sel_digit != NO_SELECTION)
            ? BLUE
            : base_color;
    ST7789_PrintSymbol(col, row, arr[i], color, WHITE, DISPLAY_FONT_SIZE);
    col += (GetFontCharWidth(arr[i]) + 1) * DISPLAY_FONT_SIZE;
  }
}

void Display_UpdateFreq(uint8_t *freq_arr, uint8_t sel_digit,
                        uint8_t is_selected_param) {
  DrawParamDigits(COL_FREQ_VALUE, ROW_FREQ_VALUE, freq_arr, 5, sel_digit,
                  is_selected_param);
}

void Display_UpdateAmplitude(uint8_t *amp_arr, uint8_t sel_digit,
                             uint8_t is_selected_param) {
  DrawParamDigits(COL_AMP_VALUE, ROW_AMP_VALUE, amp_arr, 4, sel_digit,
                  is_selected_param);
}

void Display_UpdateOffset(uint8_t *off_arr, uint8_t sel_digit,
                          uint8_t is_selected_param) {
  DrawParamDigits(COL_OFF_VALUE, ROW_OFF_VALUE, off_arr, 4, sel_digit,
                  is_selected_param);
}

void Display_UpdateRecordName(const char *name) {
  ST7789_DrawRect(COL_REC_VALUE, ROW_REC_VALUE, ST7789_WIDTH,
                  ROW_REC_VALUE + 20, WHITE);
  uint8_t len = 0;
  while (name[len] != '\0' && len < 16)
    len++;
  ST7789_PrintStr(COL_REC_VALUE, ROW_REC_VALUE, name, len, BLACK, WHITE, 2);
}
