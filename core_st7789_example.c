#include "core.h"

// Forward declarations
static void foo1(void);
static void foo2(void);

uint8_t ParametersBuffer[16];
bool StateParamsError = false;
bool StateLoadError = false;
bool PulseEnabled = false;
bool LastPulseEnabled = false;
bool TimersEnabled = false;

uint8_t FreqArray[4];
uint8_t HEArray[4];
uint8_t LEArray[4];
uint8_t INArray[4];
const uint8_t fraction_ticks[10] = {0, 7, 14, 22, 29, 36, 43, 50, 58, 65};

uint8_t PlaceNumber;
uint8_t LastPlaceNumber;

struct EEPROM eeprom;
I2C_HandleTypeDef *phi2c2 = NULL;
TIM_HandleTypeDef *phtim1 = NULL;
TIM_HandleTypeDef *phtim2 = NULL;
TIM_HandleTypeDef *phtim4 = NULL;

bool ValueChanged = false; // чисто для экрана, к таймерам и и взаимодействию с
                           // ними отношение не имеет
uint16_t CurrentStateColor;
uint16_t DefaultFontColor;
int16_t ChangeDirection = 0;

enum States {
  HZ = 0,
  HE,
  LE,
  IN,
  PROFILE
} State = PROFILE,
  LastState = PROFILE;

enum Profiles { P1 = 0, P2, P3, P4 } Profile = P1, LastProfile = P1;

uint32_t pulse_count = 0;
uint32_t counted_freq = 0;
uint32_t last_counted_freq = 0; // чтоб первый counted != last_counted даже если
                                // частота = 0, проходил условие
bool IsCounting = false;
bool IsExternalSource = false;

// Логика и вспомогательные
void UpdateExternalSource() {
  counted_freq_to_array();
  CheckParameters();
  if (!StateParamsError) {
    SetHZ();
    LaunchTimers();
  } else {
    StopTimers();
  }
  // ставлю внешний триггер для тим2
  TIM2->SMCR = (TIM2->SMCR & ~TIM_SMCR_TS) | TIM_TS_ETRF;
  UpdateHZScreen();
  UpdateScreenPlaceNumber();
}

void SetExternalSource() {
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_EnableIRQ(TIM3_IRQn);

  if (State == HZ) {
    State++;
    UpdateScreenAfterDown();
  }
  foo1();
  counted_freq = 0;
  pulse_count = 0;
  UpdateExternalSource();
}

void UnsetExternalSource() {
  NVIC_DisableIRQ(EXTI0_IRQn);
  NVIC_DisableIRQ(TIM3_IRQn);
  TIM3->CR1 &= ~TIM_CR1_CEN;
  TIM3->CNT = 0;
  TIM3->SR = 0;
  // убираем тригер по ETR который был
  TIM2->SMCR = (TIM2->SMCR & ~TIM_SMCR_TS) | TIM_TS_ITR0;
  foo2();
}

void counted_freq_to_array() {
  if (counted_freq > 9999) {
    counted_freq = 9999;
  }
  int numDigits = 0;
  uint32_t temp = counted_freq;

  if (temp == 0) {
    numDigits = 1;
  } else {
    while (temp > 0) {
      temp /= 10;
      numDigits++;
    }
  }

  for (int i = 0; i < 4 - numDigits; i++) {
    FreqArray[i] = 48;
  }
  temp = counted_freq;
  for (int i = 3; i >= 4 - numDigits; i--) {
    FreqArray[i] = '0' + (temp % 10); // ASCII код цифры
    temp /= 10;
  }
}
bool IsChanged() {
  for (uint8_t i = 0; i < 4; i++) {
    if (!(FreqArray[i] == ParametersBuffer[i] &&
          HEArray[i] == ParametersBuffer[i + 4] &&
          LEArray[i] == ParametersBuffer[i + 8] &&
          INArray[i] == ParametersBuffer[i + 12])) {
      return true;
    }
  }
  return false;
}
void LoadParametersFromProfile() {
  for (uint8_t i = 0; i < 3; i++) {
    LoadParametersFromEEPROM(Profile);
    CheckParameters();
    if (!StateParamsError) {
      if (IsExternalSource)
        ST7789_PrintStr(SECOND_POSITION, 2, FreqArray, 4,
                        EXT_DEFAULT_FONT_COLOR, BACKGROUND_COLOR, 0);
      ST7789_Fill_Color(BACKGROUND_COLOR);
      StateLoadError = false;
      break;
    }
    if (IsExternalSource) {
      StopTimers();
      ST7789_Fill_Color(BACKGROUND_COLOR);
      ST7789_PrintStr(SECOND_POSITION, 2, FreqArray, 4, EXT_DEFAULT_FONT_COLOR,
                      ERROR_DEFAULT_FONT_COLOR, 0);
      break;
    }
    StateLoadError = true;
  }

  if (StateLoadError) {
    StopTimers();
    ST7789_DrawRect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, BACKGROUND_COLOR);
    ST7789_PrintStr(2, 2, "SAVE..ERR", 9, DefaultFontColor, BACKGROUND_COLOR,
                    3);
    ST7789_PrintStr(2, 52, "ERASE..MEM", 10, DefaultFontColor, BACKGROUND_COLOR,
                    3);

    while (1) {

      if (!StateParamsError && !StateLoadError)
        return;
    }
  }
  if (!StateParamsError) {
    SetTimers();
    if (TimersEnabled)
      LaunchTimers();
  }
}
void CoreInit(I2C_HandleTypeDef *_phi2c2, TIM_HandleTypeDef *_phtim1,
              TIM_HandleTypeDef *_phtim2, TIM_HandleTypeDef *_phtim4) {
  phi2c2 = _phi2c2;
  phtim1 = _phtim1;
  phtim2 = _phtim2;
  phtim4 = _phtim4;
  PlaceNumber = 1;
  LastPlaceNumber = 1;
  ValueChanged = false;
  DefaultFontColor = DEFAULT_FONT_COLOR;
  CurrentStateColor = BG_CURRENT_STATE_COLOR;

  EEPROM_Init(&eeprom, phi2c2, I2C1_DEVICE_ADDRESS, 256, 1, 8,
              I2C_MEMADD_SIZE_8BIT);
  ST7789_Init();
  ST7789_SetFontSize(5);
  ST7789_Fill_Color(BACKGROUND_COLOR);
}

// Проверка на ошибки

void CheckParameters() {
  if (HZRequestNum == 0) {
    StateParamsError = false;
    return;
  }
  // INRequestNum < 500000 / HZRequestNum --> 1/ HZRequstNum / 2 + 100/ половина
  // периода + 100 мкс (почему 100? просто чутка рандомное число)
  StateParamsError = !(
      (INRequestNum > (MIN_DELAY + HERequestNum / 10 + 1)) &&
      (INRequestNum <= 5400) && (INRequestNum <= 500000 / HZRequestNum + 100) &&
      (HERequestNum / 10 < SYNC_DELAY) && (LERequestNum / 10 < SYNC_DELAY));
}
// Работа с памятью

void LoadParametersFromEEPROM() {
  EEPROM_Read(&eeprom, Profile * 16, ParametersBuffer, 16, 40);
  for (uint8_t i = 0; i < 4; i++) {
    if (!IsExternalSource)
      FreqArray[i] = ParametersBuffer[i];
    HEArray[i] = ParametersBuffer[i + 4];
    LEArray[i] = ParametersBuffer[i + 8];
    INArray[i] = ParametersBuffer[i + 12];
  }
}
void LoadProfilesFromEEPROM() {
  EEPROM_Read(&eeprom, 64, &Profile, 1, 15);
  if (Profile > 3) {
    Profile = 0;
  }
}
void EraseProfile() {
  // Инициализация значений по умолчанию
  FreqArray[0] = 0 + 48;
  FreqArray[1] = 1 + 48;
  FreqArray[2] = 0 + 48;
  FreqArray[3] = 0 + 48;
  HEArray[0] = 0 + 48;
  HEArray[1] = 3 + 48;
  HEArray[2] = 46;
  HEArray[3] = 0 + 48;
  LEArray[0] = 0 + 48;
  LEArray[1] = 3 + 48;
  LEArray[2] = 46;
  LEArray[3] = 0 + 48;
  INArray[0] = 1 + 48;
  INArray[1] = 0 + 48;
  INArray[2] = 0 + 48;
  INArray[3] = 0 + 48;

  for (uint8_t i = 0; i < 4; i++) {
    ParametersBuffer[i] = FreqArray[i];
    ParametersBuffer[i + 4] = HEArray[i];
    ParametersBuffer[i + 8] = LEArray[i];
    ParametersBuffer[i + 12] = INArray[i];
  }

  EEPROM_Write(&eeprom, (uint16_t)Profile * 16, ParametersBuffer, 16, 40);
}
void SaveProfile() { EEPROM_Write(&eeprom, 64, &Profile, 1, 15); }

// Работа с таймерами

void SetStateParameters() {
  switch (State) {
  case HZ:
    SetHZ();
    break;
  case HE:
    SetHE();
    break;
  case LE:
    SetINandLE();
    break;
  case IN:
    SetINandLE();
    break;
  }
}
void SetHZ() {
  uint16_t hz = HZRequestNum;
  if (hz == 0) {
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->CCER &= ~TIM_CCER_CC2E;
  } else {
    phtim1->Instance->CNT = 0;
    phtim1->Instance->PSC = 1099 / hz;
    phtim1->Instance->ARR = (72 * 1000000 / (1099 / hz + 1)) / hz;
    phtim1->Instance->CCR2 = phtim1->Instance->ARR / 2;
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->BDTR |= TIM_BDTR_MOE;
  }
}
void SetHE() {
  uint16_t he = HERequestNum;
  if (he == 0) {
    TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
  } else {
    phtim2->Instance->CNT = 0;
    phtim2->Instance->CCR3 =
        72 * SYNC_DELAY - 72 * (he / 10) - fraction_ticks[he % 10];
  }
}
void SetINandLE() {
  uint16_t le = LERequestNum;
  uint16_t he = HERequestNum;
  uint16_t in = INRequestNum;
  if (le == 0) {
    TIM4->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
  } else {
    phtim4->Instance->CNT = 0;
    phtim4->Instance->ARR =
        12 * in + 12 * (le / 10) + le % 10 - 12 * (he / 10) - he % 10 - 1;
    phtim4->Instance->CCR1 = phtim4->Instance->ARR - 12 * (le / 10) - le % 10;
    phtim4->Instance->CCR2 = phtim4->Instance->ARR - 12 * SYNC_DELAY;
  }
}
void TryToSetStateParams() {
  CheckParameters();
  if (!StateParamsError) {
    if (IsExternalSource) // при каждом изменнении будет печаться, ходя лучше
                          // печатать, когда убрали ошибку
    // но тогда надо будет вводить буферную переменную LastStateParamsError....,
    // подумать короче
    {
      ST7789_PrintStr(SECOND_POSITION, 2, FreqArray, 4, EXT_DEFAULT_FONT_COLOR,
                      BACKGROUND_COLOR, 0);
      SetTimers();
      LaunchTimers();
      return;
    }

    SetStateParameters();
    LaunchTimers();
  } else {
    StopTimers();
  }
}
void SetTimers() {
  SetHZ();
  SetHE();
  SetINandLE();
}

void LaunchTimers(void) {
  if (HZRequestNum == 0)
    return;
  // Сброс счетчиков таймеров
  TIM1->CNT = 0;
  TIM2->CNT = 0;
  TIM4->CNT = 0;

  // Запуск PWM каналов

  // TIM1 Channel 2

  TIM1->CCER |= TIM_CCER_CC2E; // Включить выход канала 2
  TIM1->CR1 |= TIM_CR1_CEN;    // Запустить таймер TIM1
  // Специальная настройка для TIM1 (расширенный таймер)
  TIM1->BDTR |= TIM_BDTR_MOE; // Включение основного выхода (Main Output Enable)
  // TIM2 Channels 1 и 2
  if (HERequestNum != 0) {
    TIM2->CCER |=
        (TIM_CCER_CC2E | TIM_CCER_CC3E); // Включить выходы каналов 2 и 3
    TIM2->CR1 |= TIM_CR1_CEN;            // Запустить таймер TIM2
  }
  // TIM4 Channels 1 и 2
  if (LERequestNum != 0) {
    TIM4->CCER |=
        (TIM_CCER_CC1E | TIM_CCER_CC2E); // Включить выходы каналов 1 и 2
    TIM4->CR1 |= TIM_CR1_CEN;            // Запустить таймер TIM4
  }
}
void StopTimers(void) {
  // Остановка PWM каналов

  // TIM1 Channel 2
  TIM1->CCER &= ~TIM_CCER_CC2E; // Выключить выход канала 2

  // TIM2 Channels 2 и 3
  TIM2->CCER &=
      ~(TIM_CCER_CC2E | TIM_CCER_CC3E); // Выключить выходы каналов 2 и 3

  // TIM4 Channels 1 и 2
  TIM4->CCER &=
      ~(TIM_CCER_CC1E | TIM_CCER_CC2E); // Выключить выходы каналов 1 и 2

  // останавливаем сами таймеры CNT
  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM2->CR1 &= ~TIM_CR1_CEN;
  TIM4->CR1 &= ~TIM_CR1_CEN;
}

// Отрисовка на экране

void PrintScreen() {
  ValueChanged = false;
  if (!IsExternalSource) {
    ST7789_PrintStr(FIRST_POSITION, 2, "HZ", 2, DefaultFontColor,
                    BACKGROUND_COLOR, 0);

    ST7789_PrintStr(SECOND_POSITION, 2, FreqArray, 4, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
  } else
    foo1();
  ST7789_PrintStr(FIRST_POSITION, 57, "HE", 2, DefaultFontColor,
                  BACKGROUND_COLOR, 0);

  ST7789_PrintStr(SECOND_POSITION, 57, HEArray, 4, DefaultFontColor,
                  BACKGROUND_COLOR, 0);

  ST7789_PrintStr(FIRST_POSITION, 112, "LE", 2, DefaultFontColor,
                  BACKGROUND_COLOR, 0);

  ST7789_PrintStr(SECOND_POSITION, 112, LEArray, 4, DefaultFontColor,
                  BACKGROUND_COLOR, 0);

  ST7789_PrintStr(FIRST_POSITION, 167, "IN", 2, DefaultFontColor,
                  BACKGROUND_COLOR, 0);

  ST7789_PrintStr(SECOND_POSITION, 167, INArray, 4, DefaultFontColor,
                  BACKGROUND_COLOR, 0);

  ST7789_DrawRect(FIRST_POSITION + 8 + 55 * (Profile - 1) + 36,
                  ROW_PROFILE - 18,
                  FIRST_POSITION + 8 + 55 * (Profile - 1) + 36 + 12,
                  ROW_PROFILE - 8, WHITE);

  // ST7789_DrawRect(FIRST_POSITION + 8 + 55 * (Profile + 1) + 36, ROW_PROFILE -
  // 18,
  //                 FIRST_POSITION + 8 + 55 * (Profile + 1) + 36 + 12,
  //                 ROW_PROFILE - 8, WHITE);

  UpdateScreenAfterDown();
  UpdateScreenPlaceNumber();
}
void UpdateScreenAfterUp() {
  switch (State) {
  case HZ:
    // обновляем новую позицию
    ST7789_DrawRect(0, 0, ST7789_WIDTH, FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT,
                    BG_CURRENT_STATE_COLOR); // PIXELS_HIGHLIGHT пикселей вверх
                                             // и вниз от строки
    ST7789_PrintStr(FIRST_POSITION, 2, "HZ", 2, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);
    ST7789_PrintStr(SECOND_POSITION, 2, FreqArray, 4, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);

    // очищаем старую
    ST7789_DrawRect(
        0, 57 - PIXELS_HIGHLIGHT, ST7789_WIDTH,
        57 + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT,
        BACKGROUND_COLOR); // PIXELS_HIGHLIGHT пикселей вверх и вниз от строки
    ST7789_PrintStr(FIRST_POSITION, 57, "HE", 2, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
    ST7789_PrintStr(SECOND_POSITION, 57, HEArray, 4, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
    break;
  case HE:
    ST7789_DrawRect(0, 57 - PIXELS_HIGHLIGHT, ST7789_WIDTH,
                    57 + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT,
                    BG_CURRENT_STATE_COLOR);
    ST7789_PrintStr(FIRST_POSITION, 57, "HE", 2, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);
    ST7789_PrintStr(SECOND_POSITION, 57, HEArray, 4, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);

    ST7789_DrawRect(0, 112 - PIXELS_HIGHLIGHT, ST7789_WIDTH,
                    112 + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT, BACKGROUND_COLOR);
    ST7789_PrintStr(FIRST_POSITION, 112, "LE", 2, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
    ST7789_PrintStr(SECOND_POSITION, 112, LEArray, 4, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
    break;
  case LE:
    ST7789_DrawRect(0, 112 - PIXELS_HIGHLIGHT, ST7789_WIDTH,
                    112 + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT,
                    BG_CURRENT_STATE_COLOR);
    ST7789_PrintStr(FIRST_POSITION, 112, "LE", 2, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);
    ST7789_PrintStr(SECOND_POSITION, 112, LEArray, 4, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);

    ST7789_DrawRect(0, 167 - PIXELS_HIGHLIGHT, ST7789_WIDTH,
                    167 + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT, BACKGROUND_COLOR);
    ST7789_PrintStr(FIRST_POSITION, 167, "IN", 2, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
    ST7789_PrintStr(SECOND_POSITION, 167, INArray, 4, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
    break;
  case IN:
    ST7789_DrawRect(0, 167 - PIXELS_HIGHLIGHT, ST7789_WIDTH,
                    167 + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT,
                    BG_CURRENT_STATE_COLOR);
    ST7789_PrintStr(FIRST_POSITION, 167, "IN", 2, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);
    ST7789_PrintStr(SECOND_POSITION, 167, INArray, 4, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);

    // очищаем полностью профили
    ST7789_DrawRect(0, ROW_PROFILE - PIXELS_HIGHLIGHT, ST7789_WIDTH,
                    ROW_PROFILE + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT,
                    BACKGROUND_COLOR);
    ST7789_PrintStr(FIRST_POSITION + 8, ROW_PROFILE, "1", 1, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
    ST7789_PrintStr(FIRST_POSITION + 63, ROW_PROFILE, "2", 1, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
    ST7789_PrintStr(FIRST_POSITION + 118, ROW_PROFILE, "3", 1, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
    ST7789_PrintStr(FIRST_POSITION + 173, ROW_PROFILE, "4", 1, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
    // но мы должны оставить выбранный профиль, его размер на 1 больше
    switch (Profile) {

    case P1:
      ST7789_PrintStr(FIRST_POSITION + 8, ROW_PROFILE, "1", 1, CURRENT_PROFILE,
                      BACKGROUND_COLOR, 6); // на 1 размер больше обычного
      break;
    case P2:
      ST7789_PrintStr(FIRST_POSITION + 63, ROW_PROFILE, "2", 1, CURRENT_PROFILE,
                      BACKGROUND_COLOR, 6);
      break;
    case P3:
      ST7789_PrintStr(FIRST_POSITION + 118, ROW_PROFILE, "3", 1,
                      CURRENT_PROFILE, BACKGROUND_COLOR, 6);
      break;
    case P4:
      ST7789_PrintStr(FIRST_POSITION + 173, ROW_PROFILE, "4", 1,
                      CURRENT_PROFILE, BACKGROUND_COLOR, 6);
      break;
    }
    break;
  case PROFILE:
    break;
  }
}
void UpdateScreenAfterDown() {
  switch (State) {
  case HZ:
    break;
  case HE:
    // очищаем старую позицию
    if (!IsExternalSource) {
      ST7789_DrawRect(
          0, 0, ST7789_WIDTH, FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT,
          BACKGROUND_COLOR); // PIXELS_HIGHLIGHT пикселей вверх и вниз от строки
      ST7789_PrintStr(FIRST_POSITION, 2, "HZ", 2, DefaultFontColor,
                      BACKGROUND_COLOR, 0);
      ST7789_PrintStr(SECOND_POSITION, 2, FreqArray, 4, DefaultFontColor,
                      BACKGROUND_COLOR, 0);
    }
    // обновляем новую
    ST7789_DrawRect(0, 57 - PIXELS_HIGHLIGHT, ST7789_WIDTH,
                    57 + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT,
                    BG_CURRENT_STATE_COLOR);
    ST7789_PrintStr(FIRST_POSITION, 57, "HE", 2, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);
    ST7789_PrintStr(SECOND_POSITION, 57, HEArray, 4, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);
    break;
  case LE:
    ST7789_DrawRect(0, 57 - PIXELS_HIGHLIGHT, ST7789_WIDTH,
                    57 + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT, BACKGROUND_COLOR);
    ST7789_PrintStr(FIRST_POSITION, 57, "HE", 2, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
    ST7789_PrintStr(SECOND_POSITION, 57, HEArray, 4, DefaultFontColor,
                    BACKGROUND_COLOR, 0);

    ST7789_DrawRect(0, 112 - PIXELS_HIGHLIGHT, ST7789_WIDTH,
                    112 + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT,
                    BG_CURRENT_STATE_COLOR);
    ST7789_PrintStr(FIRST_POSITION, 112, "LE", 2, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);
    ST7789_PrintStr(SECOND_POSITION, 112, LEArray, 4, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);
    break;
  case IN:
    ST7789_DrawRect(0, 112 - PIXELS_HIGHLIGHT, ST7789_WIDTH,
                    112 + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT, BACKGROUND_COLOR);
    ST7789_PrintStr(FIRST_POSITION, 112, "LE", 2, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
    ST7789_PrintStr(SECOND_POSITION, 112, LEArray, 4, DefaultFontColor,
                    BACKGROUND_COLOR, 0);

    ST7789_DrawRect(0, 167 - PIXELS_HIGHLIGHT, ST7789_WIDTH,
                    167 + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT,
                    BG_CURRENT_STATE_COLOR);
    ST7789_PrintStr(FIRST_POSITION, 167, "IN", 2, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);
    ST7789_PrintStr(SECOND_POSITION, 167, INArray, 4, CURRENT_STATE_COLOR,
                    BG_CURRENT_STATE_COLOR, 0);

    break;
  case PROFILE:
    ST7789_DrawRect(0, 167 - PIXELS_HIGHLIGHT, ST7789_WIDTH,
                    167 + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT, BACKGROUND_COLOR);
    ST7789_PrintStr(FIRST_POSITION, 167, "IN", 2, DefaultFontColor,
                    BACKGROUND_COLOR, 0);
    ST7789_PrintStr(SECOND_POSITION, 167, INArray, 4, DefaultFontColor,
                    BACKGROUND_COLOR, 0);

    ST7789_DrawRect(0, ROW_PROFILE - PIXELS_HIGHLIGHT, ST7789_WIDTH,
                    ROW_PROFILE + FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT,
                    BG_CURRENT_STATE_COLOR);
    ST7789_PrintStr(FIRST_POSITION + 8, ROW_PROFILE, "1", 1,
                    CURRENT_STATE_COLOR, BG_CURRENT_STATE_COLOR, 0);
    ST7789_PrintStr(FIRST_POSITION + 63, ROW_PROFILE, "2", 1,
                    CURRENT_STATE_COLOR, BG_CURRENT_STATE_COLOR, 0);
    ST7789_PrintStr(FIRST_POSITION + 118, ROW_PROFILE, "3", 1,
                    CURRENT_STATE_COLOR, BG_CURRENT_STATE_COLOR, 0);
    ST7789_PrintStr(FIRST_POSITION + 173, ROW_PROFILE, "4", 1,
                    CURRENT_STATE_COLOR, BG_CURRENT_STATE_COLOR, 0);

    // уже должно быть PrintScreen(), и выполнено UpdateScreen(), профили уже
    // должны быть отрисованы
    break;
  }
}
void UpdateScreenPlaceNumber() {
  uint16_t color = StateParamsError ? CURRENT_STATE_COLOR
                                    : CURRENT_STATE_COLOR; // пока оставим так
  uint16_t bg = StateParamsError ? ERROR_DEFAULT_FONT_COLOR
                                 : BG_CURRENT_STATE_COLOR; // а там посмотрим

  switch (State) {
  case HZ:
    ST7789_PrintCustomStr(SECOND_POSITION, 2, FreqArray, 4, 3 - PlaceNumber,
                          color, CURRENT_NUMBER_COLOR, bg, 0);
    break;
  case HE:
    ST7789_PrintCustomStr(SECOND_POSITION, 57, HEArray, 4, 3 - PlaceNumber,
                          color, CURRENT_NUMBER_COLOR, bg, 0);
    break;
  case LE:
    ST7789_PrintCustomStr(SECOND_POSITION, 112, LEArray, 4, 3 - PlaceNumber,
                          color, CURRENT_NUMBER_COLOR, bg, 0);
    break;
  case IN:
    ST7789_PrintCustomStr(SECOND_POSITION, 167, INArray, 4, 3 - PlaceNumber,
                          color, CURRENT_NUMBER_COLOR, bg, 0);
    break;
  case PROFILE: {
    ST7789_PrintStr(FIRST_POSITION + 8, ROW_PROFILE, "1", 1,
                    CURRENT_STATE_COLOR, BG_CURRENT_STATE_COLOR, 0);
    ST7789_PrintStr(FIRST_POSITION + 63, ROW_PROFILE, "2", 1,
                    CURRENT_STATE_COLOR, BG_CURRENT_STATE_COLOR, 0);
    ST7789_PrintStr(FIRST_POSITION + 118, ROW_PROFILE, "3", 1,
                    CURRENT_STATE_COLOR, BG_CURRENT_STATE_COLOR, 0);
    ST7789_PrintStr(FIRST_POSITION + 173, ROW_PROFILE, "4", 1,
                    CURRENT_STATE_COLOR, BG_CURRENT_STATE_COLOR, 0);
  }
    switch (Profile) {

    case P1:
      ST7789_PrintStr(FIRST_POSITION + 8, ROW_PROFILE, "1", 1,
                      CURRENT_NUMBER_COLOR, BG_CURRENT_STATE_COLOR, 0);
      break;
    case P2:
      ST7789_PrintStr(FIRST_POSITION + 63, ROW_PROFILE, "2", 1,
                      CURRENT_NUMBER_COLOR, BG_CURRENT_STATE_COLOR, 0);
      break;
    case P3:
      ST7789_PrintStr(FIRST_POSITION + 118, ROW_PROFILE, "3", 1,
                      CURRENT_NUMBER_COLOR, BG_CURRENT_STATE_COLOR, 0);
      break;
    case P4:
      ST7789_PrintStr(FIRST_POSITION + 173, ROW_PROFILE, "4", 1,
                      CURRENT_NUMBER_COLOR, BG_CURRENT_STATE_COLOR, 0);
      break;
    }
  }
  // Обработка нажатий на кнопки
}
void UpdateHZScreen() {

  if (StateParamsError)
    ST7789_PrintStr(SECOND_POSITION, 2, FreqArray, 4, EXT_DEFAULT_FONT_COLOR,
                    ERROR_DEFAULT_FONT_COLOR, 0);
  else
    ST7789_PrintStr(SECOND_POSITION, 2, FreqArray, 4, EXT_DEFAULT_FONT_COLOR,
                    BACKGROUND_COLOR, 0);
}

static void foo1(void) {
  ST7789_DrawRect(0, 0, ST7789_WIDTH, FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT,
                  BACKGROUND_COLOR);
  ST7789_PrintStr(4, 18, "e", 2, EXT_DEFAULT_FONT_COLOR, BACKGROUND_COLOR, 3);
  ST7789_PrintStr(30, 10, "HZ", 2, EXT_DEFAULT_FONT_COLOR, BACKGROUND_COLOR, 4);
  ST7789_PrintStr(SECOND_POSITION, 2, FreqArray, 4, EXT_DEFAULT_FONT_COLOR,
                  BACKGROUND_COLOR, 0);
}
static void foo2(void) {
  ST7789_DrawRect(0, 0, ST7789_WIDTH, FONT_SIZE_1 * 5 + PIXELS_HIGHLIGHT,
                  BACKGROUND_COLOR);
  ST7789_PrintStr(FIRST_POSITION, 2, "HZ", 2, DEFAULT_FONT_COLOR,
                  BACKGROUND_COLOR, 0);
  ST7789_PrintStr(SECOND_POSITION, 2, FreqArray, 4, DEFAULT_FONT_COLOR,
                  BACKGROUND_COLOR, 0);
}

// Обработка кнопок

void HandleButtonLeft(void) {
  if (State == HZ || State == IN) {
    if (PlaceNumber != 3)
      PlaceNumber++;
  } else if (State == HE || State == LE) {
    if (PlaceNumber != 3)
      PlaceNumber += !PlaceNumber ? 2 : 1;
  } else {
    if (Profile != 0) {
      Profile--;
      LoadParametersFromProfile();
      PrintScreen();
      SaveProfile();
    }
  }
}
void HandleButtonRight(void) {
  if (State == HZ || State == IN) {
    if (PlaceNumber != 0) {
      PlaceNumber--;
    }
  } else if (State == HE || State == LE) {
    if (PlaceNumber != 0) {
      PlaceNumber -= PlaceNumber == 2 ? 2 : 1;
    }
  } else {
    if (Profile != 3) {
      Profile++;
      LoadParametersFromProfile();
      PrintScreen();
      SaveProfile();
    }
  }
}
void HandleButtonUp(void) {
  if (State != HZ) {
    if (IsExternalSource) {
      State -= State > 1 ? 1 : 0;
    } else {
      State--;
    }
    PlaceNumber = 0;
  }
  UpdateScreenAfterUp();
  UpdateScreenPlaceNumber();
}
void HandleButtonDown(void) {
  if (State != PROFILE) {
    State++;
    PlaceNumber = 0;
  }
  UpdateScreenAfterDown();
  UpdateScreenPlaceNumber();
}

void HandleButtonIncrease(void) {
  switch (State) {
  case HZ:
    if (IsExternalSource)
      return;
    if (FreqArray[3 - PlaceNumber] < '9')
      FreqArray[3 - PlaceNumber]++;

    break;
  case HE:
    if (HEArray[3 - PlaceNumber] < '9')
      HEArray[3 - PlaceNumber]++;
    break;
  case LE:
    if (LEArray[3 - PlaceNumber] < '9')
      LEArray[3 - PlaceNumber]++;
    break;
  case IN:
    if (INArray[3 - PlaceNumber] < '9')
      INArray[3 - PlaceNumber]++;
    break;
  }
  if (!ValueChanged && State != PROFILE) {
    ST7789_DrawRect(
        FIRST_POSITION + 8 + 55 * Profile + 36, ROW_PROFILE - 18,
        FIRST_POSITION + 8 + 55 * Profile + 36 + 12, ROW_PROFILE - 8,
        CURRENT_PROFILE); // 36 = 6 * 6 (размер цифр * размер шрифта)
    ValueChanged = true;
  }
  TryToSetStateParams();
  UpdateScreenPlaceNumber();
}
void HandleButtonDecrease(void) {
  switch (State) {
  case HZ:
    if (IsExternalSource)
      return;
    if (FreqArray[3 - PlaceNumber] > '0')
      FreqArray[3 - PlaceNumber]--;

    break;
  case HE:
    if (HEArray[3 - PlaceNumber] > '0')
      HEArray[3 - PlaceNumber]--;
    break;
  case LE:
    if (LEArray[3 - PlaceNumber] > '0')
      LEArray[3 - PlaceNumber]--;
    break;
  case IN:
    if (INArray[3 - PlaceNumber] > '0')
      INArray[3 - PlaceNumber]--;
    break;
  }
  if (!ValueChanged && State != PROFILE) {
    ST7789_DrawRect(
        FIRST_POSITION + 8 + 55 * Profile + 36, ROW_PROFILE - 18,
        FIRST_POSITION + 8 + 55 * Profile + 36 + 12, ROW_PROFILE - 8,
        CURRENT_PROFILE); // 36 = 6 * 6 (размер цифр * размер шрифта)
    ValueChanged = true;
  }
  TryToSetStateParams();
  UpdateScreenPlaceNumber();
}

void HandleButtonHalfIN(void) {
  if (State == IN) {
    if (!ValueChanged && State != PROFILE) {
      ST7789_DrawRect(
          FIRST_POSITION + 8 + 55 * Profile + 36, ROW_PROFILE - 18,
          FIRST_POSITION + 8 + 55 * Profile + 36 + 12, ROW_PROFILE - 8,
          CURRENT_PROFILE); // 36 = 6 * 6 (размер цифр * размер шрифта)
      ValueChanged = true;
    }
    if (HZRequestNum <= 95)
      return;

    uint32_t hz = (500000 / HZRequestNum);
    INArray[0] = hz / 1000 + 48;
    hz %= 1000;
    INArray[1] = hz / 100 + 48;
    hz %= 100;
    INArray[2] = hz / 10 + 48;
    hz %= 10;
    INArray[3] = hz + 48;

    TryToSetStateParams();
    UpdateScreenAfterUp();
    UpdateScreenPlaceNumber();
  }
}

void HandleButtonSave(void) {
  // Действие при нажатии кнопки "сохранить" (PC15)
  if (!StateParamsError && !StateLoadError) {
    for (uint8_t i = 0; i < 4; i++) {
      ParametersBuffer[i] = FreqArray[i];
      ParametersBuffer[i + 4] = HEArray[i];
      ParametersBuffer[i + 8] = LEArray[i];
      ParametersBuffer[i + 12] = INArray[i];
    }
    EEPROM_Write(&eeprom, Profile * 16, ParametersBuffer, 16, 40);
    ST7789_DrawRect(FIRST_POSITION + 8 + 55 * Profile + 36, ROW_PROFILE - 18,
                    FIRST_POSITION + 8 + 55 * Profile + 36 + 12,
                    ROW_PROFILE - 8, WHITE);
    ValueChanged = false;
  }
}
void HandleButtonErase(void) {
  if (State == PROFILE) {
    EraseProfile(Profile);
    LoadParametersFromProfile();
    PrintScreen();
    SaveProfile();
  }
}