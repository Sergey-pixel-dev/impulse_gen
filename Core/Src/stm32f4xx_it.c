/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_MS 250
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint8_t enc_s1_flag = 0;
static uint8_t enc_s2_flag = 0;
static uint32_t last_enc_tick = 0;
static uint32_t last_btn_enc_tick = 0;
static uint32_t last_btn_param_tick = 0;
static uint32_t last_btn_rec_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1) {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void) {
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
void UART5_IRQHandler(void) {
  if (UART5->SR & USART_SR_RXNE) {
    uint8_t i = 0;
    RxBuffer[i++] = UART5->DR;
    while (!(UART5->SR & USART_SR_IDLE)) {
      if (UART5->SR & USART_SR_RXNE) {
        if (i < RX_BUFFER_SIZE) {
          RxBuffer[i++] = UART5->DR;
        }
      }
    }
    (void)UART5->SR;
    (void)UART5->DR;
    if (i >= 6 && RxBuffer[0] == 0xAA && RxBuffer[1] == 0x55) {
      uint8_t expected_len = RxBuffer[2];
      if (expected_len <= i && expected_len >= 6 &&
          RxBuffer[expected_len - 2] == 0x55 &&
          RxBuffer[expected_len - 1] == 0xAA) {
        packet_len = expected_len;
        packet_ready = 1;
      }
    }
  }
}

void EXTI0_IRQHandler(void) {
  EXTI->PR = EXTI_PR_PR0;
  uint32_t now = HAL_GetTick();
  if (now - last_enc_tick < DEBOUNCE_MS)
    return;
  if (enc_s2_flag) {
    encoder_direction = 2;
    enc_s1_flag = 0;
    enc_s2_flag = 0;
    last_enc_tick = now;
  } else {
    enc_s1_flag = 1;
  }
}

void EXTI1_IRQHandler(void) {
  EXTI->PR = EXTI_PR_PR1;
  uint32_t now = HAL_GetTick();
  if (now - last_enc_tick < DEBOUNCE_MS)
    return;

  if (enc_s1_flag) {
    encoder_direction = 1;
    enc_s1_flag = 0;
    enc_s2_flag = 0;
    last_enc_tick = now;
  } else {
    enc_s2_flag = 1;
  }
}

void EXTI2_IRQHandler(void) {
  EXTI->PR = EXTI_PR_PR2;
  uint32_t now = HAL_GetTick();
  if (now - last_btn_enc_tick < DEBOUNCE_MS)
    return;
  last_btn_enc_tick = now;
  btn_encoder_pressed = 1;
}

void EXTI3_IRQHandler(void) {
  EXTI->PR = EXTI_PR_PR3;
  uint32_t now = HAL_GetTick();
  if (now - last_btn_param_tick < DEBOUNCE_MS)
    return;
  last_btn_param_tick = now;
  btn_param_pressed = 1;
}

void EXTI4_IRQHandler(void) {
  EXTI->PR = EXTI_PR_PR4;
  uint32_t now = HAL_GetTick();
  if (now - last_btn_rec_tick < DEBOUNCE_MS)
    return;
  last_btn_rec_tick = now;
  btn_record_pressed = 1;
}
/* USER CODE END 1 */
