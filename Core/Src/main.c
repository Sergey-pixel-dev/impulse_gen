/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f446xx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "display.h"
#include "eeprom.h"
#include "i2c.h"
#include "protocol.h"
#include "signal_gen.h"
#include "spi.h"
#include "st7789.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_STANDARD_WAVEFORMS 4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* USER CODE END PD */

/* Private macro
   -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables
   ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RxBuffer[RX_BUFFER_SIZE];
uint8_t packet_ready = 0;
uint8_t packet_len = 0;

volatile uint8_t encoder_direction = 0; // 0=none, 1=increment, 2=decrement
volatile uint8_t btn_encoder_pressed = 0;
volatile uint8_t btn_record_pressed = 0;
volatile uint8_t btn_param_pressed = 0;

EEPROM_t eeprom;

static uint8_t freq_arr[5] = {'0', '0', '5', '0', '0'}; // 500 Hz
static uint8_t amp_arr[4] = {'1', '0', '2', '4'};       // 1024
static uint8_t off_arr[4] = {'2', '0', '4', '8'};       // 2048

static uint8_t current_param = 0;    // 0=freq, 1=amp, 2=offset
static uint8_t selected_digit = 4;   // selected digit index
static uint8_t current_waveform = 0; // 0=sin, 1=square, 2=tri, 3=saw
static uint8_t current_record = 0;   // 0=standard waveforms, 1+=EEPROM records
static uint16_t current_n_points = DAC_BUFFER_SIZE;

static char record_name[6]; // 5 chars + null terminator for EEPROM record name

static const char *waveform_names[] = {"Sine", "Square", "Triangle",
                                       "Sawtooth"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
void DAC_Init(void);
void DMA_Init(void);
void TIM2_Init(void);
void UART5_Init(void);
void UART5_Transmit(uint8_t *data, uint16_t size);
static uint16_t ArrayToNumber_5(uint8_t *arr);
static uint16_t ArrayToNumber_4(uint8_t *arr);
static void RegenerateSignal(void);
static void UpdateAllDisplay(void);
static uint8_t GetParamLen(uint8_t param);
static void LoadEepromRecord(uint8_t record_idx);
static void EEPROM_WriteTestData(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t ArrayToNumber_5(uint8_t *arr) {
  return (arr[0] - '0') * 10000 + (arr[1] - '0') * 1000 + (arr[2] - '0') * 100 +
         (arr[3] - '0') * 10 + (arr[4] - '0');
}

static uint16_t ArrayToNumber_4(uint8_t *arr) {
  return (arr[0] - '0') * 1000 + (arr[1] - '0') * 100 + (arr[2] - '0') * 10 +
         (arr[3] - '0');
}

static uint8_t GetParamLen(uint8_t param) { return (param == 0) ? 5 : 4; }

static uint8_t *GetParamArr(uint8_t param) {
  switch (param) {
  case 0:
    return freq_arr;
  case 1:
    return amp_arr;
  case 2:
    return off_arr;
  default:
    return freq_arr;
  }
}

static void LoadEepromRecord(uint8_t record_idx) {
  uint16_t addr =
      EEPROM_RECORDS_BASE + (uint16_t)record_idx * EEPROM_RECORD_SIZE;
  uint8_t header[9];
  EEPROM_Read(&eeprom, addr, header, 9, 2000);

  uint8_t n_pts = header[1];
  for (uint8_t i = 0; i < 5; i++) {
    record_name[i] = (char)header[4 + i];
  }
  record_name[5] = '\0';

  if (n_pts > 100)
    n_pts = 100;
  current_n_points = n_pts;

  uint8_t points_data[200];
  uint16_t points_len = (uint16_t)n_pts * 2;
  EEPROM_Read(&eeprom, addr + 9, points_data, points_len, 2000);

  uint16_t pts[100];
  for (uint8_t i = 0; i < n_pts; i++) {
    pts[i] = points_data[2 * i] | ((uint16_t)points_data[2 * i + 1] << 8);
  }

  SignalGen_FromRecord(pts, n_pts);

  DMA1_Stream5->CR &= ~DMA_SxCR_EN;
  while (DMA1_Stream5->CR & DMA_SxCR_EN)
    ;
  DMA1->HIFCR = DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 |
                DMA_HIFCR_CTCIF5 | DMA_HIFCR_CFEIF5;
  DMA1_Stream5->NDTR = n_pts;
  DMA1_Stream5->CR |= DMA_SxCR_EN;
}

static void RegenerateSignal(void) {
  if (current_record == 0) {
    uint16_t amp = ArrayToNumber_4(amp_arr);
    uint16_t off = ArrayToNumber_4(off_arr);
    switch (current_waveform) {
    case 0:
      SignalGen_Sine(dac_buffer, DAC_BUFFER_SIZE, amp, off);
      break;
    case 1:
      SignalGen_Square(dac_buffer, DAC_BUFFER_SIZE, amp, off);
      break;
    case 2:
      SignalGen_Triangle(dac_buffer, DAC_BUFFER_SIZE, amp, off);
      break;
    case 3:
      SignalGen_Sawtooth(dac_buffer, DAC_BUFFER_SIZE, amp, off);
      break;
    }
  }
}

static void UpdateAllDisplay(void) {

  Display_UpdateFreq(freq_arr,
                     (current_param == 0) ? selected_digit : NO_SELECTION,
                     current_param == 0);
  if (current_record == 0) {
    Display_UpdateAmplitude(
        amp_arr, (current_param == 1) ? selected_digit : NO_SELECTION,
        current_param == 1);
    Display_UpdateOffset(off_arr,
                         (current_param == 2) ? selected_digit : NO_SELECTION,
                         current_param == 2);
    Display_UpdateRecordName(waveform_names[current_waveform]);
  } else
    Display_UpdateRecordName(record_name);
}
static void EEPROM_WriteTestData(void) {
  // Write record count = 2
  uint8_t count = 2;
  EEPROM_Status_t a = EEPROM_Write(&eeprom, EEPROM_COUNT_ADDR, &count, 1, 2000);
  HAL_Delay(10);

  // Record 0: Sine 10 points, max_freq=1000, name="Test1"
  {
    uint16_t addr = EEPROM_RECORDS_BASE + 0 * EEPROM_RECORD_SIZE;
    uint8_t header[9] = {0xAA, 10, 0, 0, 'T', 'e', 's', 't', '1'};
    // max_freq = 1000 in little-endian at header[2..3]
    header[2] = (uint8_t)(1000 & 0xFF);
    header[3] = (uint8_t)(1000 >> 8);
    a = EEPROM_Write(&eeprom, addr, header, 9, 2000);
    HAL_Delay(10);

    // 10 sine points: dac[i] = 2048 + 1024*sin(2*pi*i/10)
    uint8_t pts[20];
    for (uint8_t i = 0; i < 10; i++) {
      double angle = 2.0 * M_PI * i / 10.0;
      int32_t val = (int32_t)(2048.0 + 1024.0 * sin(angle));
      if (val < 0)
        val = 0;
      if (val > 4095)
        val = 4095;
      pts[2 * i] = (uint8_t)(val & 0xFF);
      pts[2 * i + 1] = (uint8_t)(val >> 8);
    }
    a = EEPROM_Write(&eeprom, addr + 9, pts, 20, 2000);
    HAL_Delay(10);
  }

  // Record 1: Ramp 20 points, max_freq=2000, name="Ramp1"
  {
    uint16_t addr = EEPROM_RECORDS_BASE + 1 * EEPROM_RECORD_SIZE;
    uint8_t header[9] = {0xAA, 20, 0, 0, 'R', 'a', 'm', 'p', '1'};
    header[2] = (uint8_t)(2000 & 0xFF);
    header[3] = (uint8_t)(2000 >> 8);
    EEPROM_Write(&eeprom, addr, header, 9, 2000);
    HAL_Delay(10);

    // 20 ramp points: dac[i] = 4095 * i / 19
    uint8_t pts[40];
    for (uint8_t i = 0; i < 20; i++) {
      uint16_t val = (uint16_t)(4095UL * i / 19);
      pts[2 * i] = (uint8_t)(val & 0xFF);
      pts[2 * i + 1] = (uint8_t)(val >> 8);
    }
    EEPROM_Write(&eeprom, addr + 9, pts, 40, 2000);
    HAL_Delay(10);
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU
   * Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the
   * Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DAC_Init();
  DMA_Init();
  TIM2_Init();
  UART5_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  I2C1_Init(I2C_SPEED_STANDARD);
  EEPROM_Init(&eeprom, 0x50, 256, 2, 16, I2C_MEMADD_SIZE_8BIT);
  SPI1_Init();
  ST7789_Init();
  Display_Init();
  SignalGen_Init();

  DMA1_Stream5->CR &= ~DMA_SxCR_EN;
  while (DMA1_Stream5->CR & DMA_SxCR_EN)
    ;
  DMA1->HIFCR = DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 |
                DMA_HIFCR_CTCIF5 | DMA_HIFCR_CFEIF5;
  DMA1_Stream5->M0AR = (uint32_t)dac_buffer;
  DMA1_Stream5->NDTR = DAC_BUFFER_SIZE;

  DAC->CR |= DAC_CR_EN1;
  DMA1_Stream5->CR |= DMA_SxCR_EN;
  TIM2->CR1 |= TIM_CR1_CEN;

  EEPROM_WriteTestData();
  UpdateAllDisplay();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (packet_ready) {
      Protocol_ProcessPacket(RxBuffer, packet_len);
      packet_ready = 0;
    }

    if (encoder_direction != 0) {
      uint8_t dir = encoder_direction;
      encoder_direction = 0;

      uint8_t *arr = GetParamArr(current_param);
      uint8_t digit_val = arr[selected_digit] - '0';

      if (dir == 1) {
        digit_val++;
        if (digit_val > 9)
          digit_val = 0;
      } else {
        if (digit_val == 0)
          digit_val = 9;
        else
          digit_val--;
      }
      arr[selected_digit] = '0' + digit_val;

      RegenerateSignal();

      if (current_param == 0) {
        uint16_t freq = ArrayToNumber_5(freq_arr);
        uint16_t psc, arr_val;
        CalcTimerParams(freq, current_n_points, &psc, &arr_val);
        // TIM2->PSC = psc - 1;
        // TIM2->ARR = arr_val;
        // TIM2->EGR |= TIM_EGR_UG;
      }

      switch (current_param) {
      case 0:
        Display_UpdateFreq(freq_arr, selected_digit, 1);
        break;
      case 1:
        Display_UpdateAmplitude(amp_arr, selected_digit, 1);
        break;
      case 2:
        Display_UpdateOffset(off_arr, selected_digit, 1);
        break;
      }
    }

    if (btn_encoder_pressed) {
      btn_encoder_pressed = 0;
      uint8_t len = GetParamLen(current_param);
      if (selected_digit == 0)
        selected_digit = len - 1;
      else
        selected_digit--;

      switch (current_param) {
      case 0:
        Display_UpdateFreq(freq_arr, selected_digit, 1);
        break;
      case 1:
        Display_UpdateAmplitude(amp_arr, selected_digit, 1);
        break;
      case 2:
        Display_UpdateOffset(off_arr, selected_digit, 1);
        break;
      }
    }

    if (btn_param_pressed) {
      btn_param_pressed = 0;
      current_param = (current_param + 1) % 3;
      selected_digit = GetParamLen(current_param) - 1;
      UpdateAllDisplay();
    }

    if (btn_record_pressed) {
      btn_record_pressed = 0;
      if (current_record == 0) {
        current_waveform++;
        if (current_waveform >= NUM_STANDARD_WAVEFORMS) {
          uint8_t count = 0;
          EEPROM_Status_t a =
              EEPROM_Read(&eeprom, EEPROM_COUNT_ADDR, &count, 1, 2000);
          if (count > 0) {
            current_record = 1;
            current_waveform = 0;
            LoadEepromRecord(0);
          } else {
            current_waveform = 0;
          }
        }
      } else {
        uint8_t count = 0;
        EEPROM_Read(&eeprom, EEPROM_COUNT_ADDR, &count, 1, 2000);
        current_record++;
        if (current_record > count) {
          current_record = 0;
          current_waveform = 0;
          current_n_points = DAC_BUFFER_SIZE;
          DMA1_Stream5->CR &= ~DMA_SxCR_EN;
          while (DMA1_Stream5->CR & DMA_SxCR_EN)
            ;
          DMA1->HIFCR = DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 |
                        DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5 | DMA_HIFCR_CFEIF5;
          DMA1_Stream5->NDTR = DAC_BUFFER_SIZE;
          DMA1_Stream5->CR |= DMA_SxCR_EN;
        } else {
          LoadEepromRecord(current_record - 1);
        }
      }

      RegenerateSignal();
      UpdateAllDisplay();
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                  RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;

  // DAC, PA4 - Analog
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4;
  GPIOA->MODER &= ~GPIO_MODER_MODER4;
  GPIOA->MODER |= GPIO_MODER_MODER4; // Analog

  // PA5 (SPI1 SCK) and PA7 (SPI1 MOSI) configured in SPI1_Init()

  // UART5, PC12 (TX), PD2 (RX)
  GPIOC->MODER &= ~GPIO_MODER_MODER12;
  GPIOC->MODER |= GPIO_MODER_MODER12_1;
  GPIOC->AFR[1] &= ~(0xF << 16);
  GPIOC->AFR[1] |= (8 << 16);
  GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12;

  GPIOD->MODER &= ~GPIO_MODER_MODER2;
  GPIOD->MODER |= GPIO_MODER_MODER2_1;
  GPIOD->AFR[0] &= ~(0xF << 8);
  GPIOD->AFR[0] |= (8 << 8);
  GPIOD->PUPDR &= ~GPIO_PUPDR_PUPDR2;
  GPIOD->PUPDR |= GPIO_PUPDR_PUPDR2_0;

  // ST7789 control pins: PB13(CS), PB14(RST), PB15(DC) — GPIO output
  // push-pull
  GPIOB->MODER &=
      ~(GPIO_MODER_MODER13 | GPIO_MODER_MODER14 | GPIO_MODER_MODER15);
  GPIOB->MODER |=
      GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;
  GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR14 |
                    GPIO_OSPEEDER_OSPEEDR15;
  // Set CS high (deselect), RST high
  GPIOB->BSRR = GPIO_PIN_13 | GPIO_PIN_14;

  // Encoder and buttons: PB0(S1), PB1(S2), PB2(BTN), PB3(Params),
  // PB4(Records) Input mode with pull-up
  GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 |
                    GPIO_MODER_MODER3 | GPIO_MODER_MODER4);
  // Pull-up
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR2 |
                    GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR4);
  GPIOB->PUPDR |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 |
                  GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0 |
                  GPIO_PUPDR_PUPDR4_0;

  // EXTI configuration for PB0-PB4
  // SYSCFG clock
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // Map EXTI0..4 to port B
  SYSCFG->EXTICR[0] &= ~(0xF << 0); // EXTI0 -> PB0
  SYSCFG->EXTICR[0] |= (1 << 0);
  SYSCFG->EXTICR[0] &= ~(0xF << 4); // EXTI1 -> PB1
  SYSCFG->EXTICR[0] |= (1 << 4);
  SYSCFG->EXTICR[0] &= ~(0xF << 8); // EXTI2 -> PB2
  SYSCFG->EXTICR[0] |= (1 << 8);
  SYSCFG->EXTICR[0] &= ~(0xF << 12); // EXTI3 -> PB3
  SYSCFG->EXTICR[0] |= (1 << 12);
  SYSCFG->EXTICR[1] &= ~(0xF << 0); // EXTI4 -> PB4
  SYSCFG->EXTICR[1] |= (1 << 0);

  // Falling edge trigger (active low buttons)
  EXTI->FTSR |= EXTI_FTSR_TR0 | EXTI_FTSR_TR1 | EXTI_FTSR_TR2 | EXTI_FTSR_TR3 |
                EXTI_FTSR_TR4;
  // Unmask
  EXTI->IMR |=
      EXTI_IMR_MR0 | EXTI_IMR_MR1 | EXTI_IMR_MR2 | EXTI_IMR_MR3 | EXTI_IMR_MR4;

  // Enable NVIC for EXTI0..4
  NVIC_SetPriority(EXTI0_IRQn, 3);
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_SetPriority(EXTI1_IRQn, 3);
  NVIC_EnableIRQ(EXTI1_IRQn);
  NVIC_SetPriority(EXTI2_IRQn, 3);
  NVIC_EnableIRQ(EXTI2_IRQn);
  NVIC_SetPriority(EXTI3_IRQn, 3);
  NVIC_EnableIRQ(EXTI3_IRQn);
  NVIC_SetPriority(EXTI4_IRQn, 3);
  NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DAC_Init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  DAC->CR = DAC_CR_DMAEN1 | DAC_CR_TSEL1_2 | DAC_CR_TEN1;
}

void DMA_Init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  DMA1_Stream5->CR |= DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0 |
                      DMA_SxCR_PL_1 | DMA_SxCR_PL_0 | DMA_SxCR_MSIZE_0 |
                      DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC |
                      DMA_SxCR_DIR_0;
  DMA1_Stream5->PAR = (uint32_t)&DAC->DHR12R1;
  DMA1_Stream5->M0AR = (uint32_t)dac_buffer;
  DMA1_Stream5->NDTR = DAC_BUFFER_SIZE;
  DMA1_Stream5->FCR = 0;
}

void TIM2_Init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_URS;
  TIM2->CR2 = TIM_CR2_MMS_1;
  TIM2->PSC = 2 - 1;
  TIM2->ARR = 1800;
  TIM2->CNT = 0;

  TIM2->EGR |= TIM_EGR_UG;
  TIM2->SR &= ~TIM_SR_UIF;
}

void UART5_Init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
  UART5->BRR = 0x187; // 115200 baud, 45MHz APB1
  UART5->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE |
               USART_CR1_IDLEIE;
  NVIC_SetPriority(UART5_IRQn, 2);
  NVIC_EnableIRQ(UART5_IRQn);
}

void UART5_Transmit(uint8_t *data, uint16_t size) {
  for (uint16_t i = 0; i < size; i++) {
    UART5->DR = data[i];
    while (!(UART5->SR & USART_SR_TXE))
      ;
  }
  while (!(UART5->SR & USART_SR_TC))
    ;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
