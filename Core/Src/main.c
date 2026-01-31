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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro
   -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE ENsD PM */

/* Private variables
   ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t sine_values[6] = {
    512, 734, 734, 512, 290, 290,
};
uint8_t RxBuffer[RX_BUFFER_SIZE];
uint8_t packet_ready = 0;
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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
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
  // TIM2->CR1 |= TIM_CR1_CEN;
  // DMA1_Stream5->CR |= DMA_SxCR_EN;
  DMA1->HIFCR = DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 |
                DMA_HIFCR_CTCIF5 | DMA_HIFCR_CFEIF5;
  // DAC1->CR |= DAC_CR_EN1;
  // uint8_t buf[4] = {55, 56, 57, 58};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // UART5_Transmit(buf, 4);
    HAL_Delay(1000);
    UART5_Transmit(RxBuffer, 4);
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
  RCC->AHB1ENR |=
      RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
  // DAC, PA4, PA5
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5); // без подтяжки
  GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
  GPIOA->MODER |= GPIO_MODER_MODER4 | GPIO_MODER_MODER5;
  // UART5, PC12, PD2
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
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DAC_Init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  DAC->CR = DAC_CR_DMAEN1 | DAC_CR_TSEL1_2 | DAC_CR_TEN1; //  | BOF1;
}

void DMA_Init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  DMA1_Stream5->CR |= DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0 |
                      DMA_SxCR_PL_1 | DMA_SxCR_PL_0 | DMA_SxCR_MSIZE_0 |
                      DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC |
                      DMA_SxCR_DIR_0;
  DMA1_Stream5->PAR = (uint32_t)&DAC->DHR12R1;
  DMA1_Stream5->M0AR = (uint32_t)&sine_values;
  DMA1_Stream5->NDTR = 6;
  DMA1_Stream5->FCR = 0;
}

void TIM2_Init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_URS;
  TIM2->CR2 = TIM_CR2_MMS_1;
  TIM2->PSC = 1 - 1;
  TIM2->ARR = 9;
  TIM2->CNT = 0;

  TIM2->EGR |= TIM_EGR_UG;
  TIM2->SR &= ~TIM_SR_UIF;
}

void UART5_Init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
  UART5->BRR = 0x187; // 115200 бод, 45мгц apb1
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
  /* User can add his own implementation to report the HAL error return state */
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
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
