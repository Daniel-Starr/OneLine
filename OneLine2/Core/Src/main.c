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
#include "gpdma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Board 2 bridge: USART3_RX -> shared TX FIFO -> LPUART1_TX -> PC.
   PURE REGISTER-LEVEL mirror of board 1: no DMA, no HAL UART runtime calls, no
   interrupts, no handles -- nothing for the HAL DMA/handle machinery to corrupt (that
   machinery was the HardFault source).
     - RX: poll RXNE on USART3, read each byte straight from RDR into the FIFO.
     - TX: poll TXE on LPUART1, write one FIFO byte to TDR.
   Fully non-blocking, byte-for-byte transparent. The main loop polls far faster than one
   byte time (8.7us @ 115200) so nothing is lost. RX and TX both run in the main loop (no
   ISR touches the FIFO), so no critical sections are needed. FIFO full -> drop. */
#define TX_FIFO_SIZE    4096u

/* Shared TX FIFO. Producer: rx_poll. Consumer: tx_kick. Both run in the main loop. */
static uint8_t  tx_fifo[TX_FIFO_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint16_t tx_count;

static volatile uint32_t rx_bytes;
static volatile uint32_t tx_bytes;
static volatile uint32_t drop_cnt;
static volatile uint32_t err_cnt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void fifo_push_byte(uint8_t b);
static void rx_poll(void);
static void tx_kick(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_LPUART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Board 2: USART3_RX -> LPUART1_TX -> PC. Nothing to arm -- RX is polled (RXNE) and TX
     is polled (TXE) in the main loop. USART3/LPUART1 are in TX_RX mode (RE/TE) from init. */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    rx_poll();
    tx_kick();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* Push one received byte into the shared TX FIFO. FIFO full -> drop and count. Only the
   main loop touches the FIFO (rx_poll produces, tx_kick consumes), so no locking needed. */
static void fifo_push_byte(uint8_t b)
{
  if (tx_count >= TX_FIFO_SIZE)
  {
    drop_cnt++;
    return;
  }
  tx_fifo[tx_head] = b;
  tx_head++;
  if (tx_head >= TX_FIFO_SIZE)
  {
    tx_head = 0u;
  }
  tx_count++;
}

/* Poll the USART3 RX input: read any waiting byte straight from RDR into the FIFO (reading
   RDR clears RXNE), and clear a stuck overrun so RX keeps flowing. No DMA, no HAL, no handle. */
static void rx_poll(void)
{
  uint32_t isr = USART3->ISR;

  if ((isr & USART_ISR_ORE) != 0u)
  {
    USART3->ICR = USART_ICR_ORECF;
    err_cnt++;
  }
  if ((isr & USART_ISR_RXNE_RXFNE) != 0u)
  {
    fifo_push_byte((uint8_t)(USART3->RDR & 0xFFu));
    rx_bytes++;
  }
}

/* Drain the FIFO to LPUART1 by writing directly to TDR when TXE is set. Non-blocking and
   handle-free; the fast main loop keeps TDR fed for full line rate. */
static void tx_kick(void)
{
  if (((LPUART1->ISR & USART_ISR_TXE_TXFNF) == 0u) || (tx_count == 0u))
  {
    return;
  }

  LPUART1->TDR = tx_fifo[tx_tail];
  tx_tail++;
  if (tx_tail >= TX_FIFO_SIZE)
  {
    tx_tail = 0u;
  }
  tx_count = (uint16_t)(tx_count - 1u);
  tx_bytes++;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
