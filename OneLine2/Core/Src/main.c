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

/* ===== OneLine UART bridge (Board 2): nonblocking, backpressure-safe forward ===
   USART3_RX (fast, 921600) -> TX FIFO -> LPUART1_TX (slow, 115200) -> PC.
   The TX FIFO absorbs the rate mismatch; when it fills, new data is dropped
   (drop_cnt). Slow is allowed; the data path must never block/deadlock. */
#define RX_DMA_BUF_SIZE   256u   /* USART3_RX DMA circular buffer (ReceiveToIdle)  */
#define TX_DMA_BUF_SIZE   256u   /* staging buffer for one LPUART1_TX DMA burst    */
#define TX_FIFO_SIZE      4096u  /* software TX FIFO: absorbs RX>TX rate mismatch  */

/* DMA-accessed buffers. STM32U575 is Cortex-M33: no data cache, so DMA needs no
   cache maintenance / 32-byte alignment for coherency. */
static uint8_t rx_dma_buf[RX_DMA_BUF_SIZE];
static uint8_t tx_dma_buf[TX_DMA_BUF_SIZE];

/* RX side: USART3_RX. Positions tracked across HT/TC/IDLE events. */
static UART_HandleTypeDef *huart_rx;      /* = &huart3   */
static volatile uint16_t rx_old_pos;      /* drained up to here (bridge_poll)   */
static volatile uint16_t rx_write_pos;    /* latest DMA write position (Size)   */

/* TX side: LPUART1_TX + software FIFO. FIFO is touched only by the main loop. */
static UART_HandleTypeDef *huart_tx;      /* = &hlpuart1 */
static uint8_t  tx_fifo[TX_FIFO_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint16_t tx_count;
static volatile uint16_t tx_dma_len;
static volatile uint8_t  tx_busy;

/* Debug counters (inspect live in the debugger). */
static volatile uint32_t rx_bytes;
static volatile uint32_t tx_bytes;
static volatile uint32_t drop_cnt;        /* bytes dropped because TX FIFO full  */
static volatile uint32_t err_cnt;
static volatile uint32_t tx_err_cnt;
static volatile uint32_t rx_event_idle_cnt;
static volatile uint32_t rx_event_ht_cnt;
static volatile uint32_t rx_event_tc_cnt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void fifo_push(const uint8_t *data, uint16_t len);
static void tx_kick(void);
static void bridge_poll(void);
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

  /* Board 2 link: Board 1 -> USART3(RX) -> LPUART1(TX) -> PC */
  huart_rx = &huart3;
  huart_tx = &hlpuart1;

  /* Start the RX side: ReceiveToIdle over circular DMA. HT/TC/IDLE events all
     arrive in HAL_UARTEx_RxEventCallback. Keep HT enabled (do not disable it). */
  if (HAL_UARTEx_ReceiveToIdle_DMA(huart_rx, rx_dma_buf, RX_DMA_BUF_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    bridge_poll();
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

/* Push received bytes into the software TX FIFO. The FIFO is produced and consumed
   only by the main loop (bridge_poll / tx_kick), so no critical section is needed
   here. When the FIFO is full, new bytes are dropped - never block. */
static void fifo_push(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  for (i = 0u; i < len; i++)
  {
    if (tx_count >= TX_FIFO_SIZE)
    {
      drop_cnt++;                  /* FIFO full: drop new data (loss ok, stall not) */
      continue;
    }
    tx_fifo[tx_head] = data[i];
    tx_head++;
    if (tx_head >= TX_FIFO_SIZE)
    {
      tx_head = 0u;
    }
    tx_count++;
  }
}

/* Start one LPUART1_TX DMA burst (<= 256 bytes) from the FIFO when TX is idle.
   Called only from the main loop. Never waits on tx_busy - returns immediately if
   a burst is already in flight (no blocking, no spin). */
static void tx_kick(void)
{
  uint16_t n;
  uint16_t i;
  uint16_t t;
  uint32_t primask;

  /* Claim the TX path. tx_busy is also cleared by the TX ISRs, so guard the test. */
  primask = __get_PRIMASK();
  __disable_irq();
  if (tx_busy || (tx_count == 0u))
  {
    __set_PRIMASK(primask);
    return;
  }
  tx_busy = 1u;
  __set_PRIMASK(primask);

  n = (tx_count < TX_DMA_BUF_SIZE) ? tx_count : TX_DMA_BUF_SIZE;
  tx_dma_len = n;

  t = tx_tail;
  for (i = 0u; i < n; i++)
  {
    tx_dma_buf[i] = tx_fifo[t];
    t++;
    if (t >= TX_FIFO_SIZE)
    {
      t = 0u;
    }
  }

  if (HAL_UART_Transmit_DMA(huart_tx, tx_dma_buf, n) == HAL_OK)
  {
    tx_tail = t;                                  /* commit consumption */
    tx_count = (uint16_t)(tx_count - n);
  }
  else
  {
    tx_dma_len = 0u;                              /* start failed: keep FIFO data */
    primask = __get_PRIMASK();
    __disable_irq();
    tx_busy = 0u;
    __set_PRIMASK(primask);
  }
}

/* Main-loop pump: drain the USART3_RX DMA ring into the TX FIFO, then kick TX.
   The byte copying lives here (thread mode), never in an ISR, so the RX->TX path
   cannot block. 'rx_write_pos' is the ABSOLUTE DMA write position (0..RX_DMA_BUF_SIZE)
   recorded by the RX-event ISR. */
static void bridge_poll(void)
{
  uint16_t old;
  uint16_t pos;

  pos = rx_write_pos;
  if (pos > RX_DMA_BUF_SIZE)
  {
    pos = 0u;
  }
  old = rx_old_pos;

  if (pos != old)
  {
    if (pos > old)
    {
      uint16_t len = (uint16_t)(pos - old);            /* contiguous [old, pos) */
      rx_bytes += len;
      fifo_push(&rx_dma_buf[old], len);
    }
    else
    {
      if (old < RX_DMA_BUF_SIZE)                        /* tail [old, end)       */
      {
        uint16_t len1 = (uint16_t)(RX_DMA_BUF_SIZE - old);
        rx_bytes += len1;
        fifo_push(&rx_dma_buf[old], len1);
      }
      if (pos > 0u)                                     /* head [0, pos)         */
      {
        rx_bytes += pos;
        fifo_push(&rx_dma_buf[0], pos);
      }
    }
    rx_old_pos = pos;
  }

  tx_kick();
}

/* RX: HT / TC / IDLE land here. Minimal by design - only record the DMA write
   position and event counters. The copy is done by bridge_poll() in the main loop.
   Never re-arm ReceiveToIdle here (the circular DMA keeps running). */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == huart_rx)
  {
    uint16_t pos = Size;

    if (pos > RX_DMA_BUF_SIZE)
    {
      pos = 0u;
    }

    switch (HAL_UARTEx_GetRxEventType(huart))
    {
      case HAL_UART_RXEVENT_IDLE: rx_event_idle_cnt++; break;
      case HAL_UART_RXEVENT_HT:   rx_event_ht_cnt++;   break;
      case HAL_UART_RXEVENT_TC:   rx_event_tc_cnt++;   break;
      default: break;
    }

    rx_write_pos = pos;
  }
}

/* TX burst finished: only release the claim and count bytes. Do NOT kick here -
   the next bridge_poll() iteration in the main loop starts the following burst. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == huart_tx)
  {
    tx_bytes += tx_dma_len;
    tx_dma_len = 0u;
    tx_busy = 0u;
  }
}

/* RX error (e.g. ORE stops the DMA): synchronous abort + restart ReceiveToIdle and
   reset positions. TX error: just release the claim; bridge_poll() resumes on the
   next iteration. No long work in either branch. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == huart_rx)
  {
    err_cnt++;
    (void)HAL_UART_AbortReceive(huart_rx);
    rx_old_pos = 0u;
    rx_write_pos = 0u;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(huart_rx, rx_dma_buf, RX_DMA_BUF_SIZE);
  }
  else if (huart == huart_tx)
  {
    tx_err_cnt++;
    tx_busy = 0u;
    tx_dma_len = 0u;
  }
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
