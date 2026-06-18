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

/* ===== OneLine UART bridge (Board 1): three RX ports -> one USART3_TX ===== */
#define RX_BUF_SIZE       256u   /* per-port RX DMA circular buffer (ReceiveToIdle) */
#define TX_DMA_BUF_SIZE   256u   /* staging buffer for one USART3_TX DMA burst      */
#define TX_FIFO_SIZE      4096u  /* shared TX software FIFO (3 RX merge into 1 TX)  */

/* One receive port: its own UART, circular DMA buffer, positions and counters. */
typedef struct
{
  UART_HandleTypeDef *huart;
  uint8_t  buf[RX_BUF_SIZE];
  volatile uint16_t old_pos;            /* forwarded up to here                     */
  volatile uint16_t write_pos;          /* latest DMA write position (Size)         */
  volatile uint32_t rx_bytes;
  volatile uint32_t drop_cnt;           /* bytes dropped because shared FIFO full   */
  volatile uint32_t err_cnt;
  volatile uint32_t rx_event_idle_cnt;
  volatile uint32_t rx_event_ht_cnt;
  volatile uint32_t rx_event_tc_cnt;
} RxPort_t;

/* Three independent RX ports. STM32U575 is Cortex-M33: no data cache, so DMA needs
   no cache maintenance / 32-byte alignment for coherency. */
static RxPort_t rx_usart1;   /* USART1_RX  -> huart1   */
static RxPort_t rx_uart4;    /* UART4_RX   -> huart4   */
static RxPort_t rx_lpuart1;  /* LPUART1_RX -> hlpuart1 */

/* Single TX side: USART3_TX -> Board 2. */
static UART_HandleTypeDef *huart_tx;     /* = &huart3 */

/* Shared TX software FIFO. Producers: the three RX callbacks. Consumer: tx_kick. */
static uint8_t  tx_fifo[TX_FIFO_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint16_t tx_count;

/* TX DMA burst staging. Must be static/global: DMA reads it after the call returns. */
static uint8_t  tx_dma_buf[TX_DMA_BUF_SIZE];
static volatile uint16_t tx_dma_len;
static volatile uint8_t  tx_busy;

/* TX-side debug counters. */
static volatile uint32_t tx_bytes;
static volatile uint32_t tx_err_cnt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void fifo_push(const uint8_t *data, uint16_t len, RxPort_t *p);
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
  MX_UART4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Board 1 link: USART1_RX / UART4_RX / LPUART1_RX -> USART3_TX -> Board 2 */
  huart_tx         = &huart3;
  rx_usart1.huart  = &huart1;
  rx_uart4.huart   = &huart4;
  rx_lpuart1.huart = &hlpuart1;

  /* Start all three RX ports: ReceiveToIdle over circular DMA. HT/TC/IDLE events
     all arrive in HAL_UARTEx_RxEventCallback. Keep HT enabled; never re-arm inside
     that callback (the circular DMA keeps running). */
  if (HAL_UARTEx_ReceiveToIdle_DMA(rx_usart1.huart,  rx_usart1.buf,  RX_BUF_SIZE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_ReceiveToIdle_DMA(rx_uart4.huart,   rx_uart4.buf,   RX_BUF_SIZE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_ReceiveToIdle_DMA(rx_lpuart1.huart, rx_lpuart1.buf, RX_BUF_SIZE) != HAL_OK)
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
    /* Fallback drive: the real work happens in the DMA/UART callbacks. */
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

static void fifo_push(const uint8_t *data, uint16_t len, RxPort_t *p)
{
  uint16_t i;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  for (i = 0u; i < len; i++)
  {
    if (tx_count >= TX_FIFO_SIZE)
    {
      p->drop_cnt++;               /* shared FIFO full: drop this byte */
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
  __set_PRIMASK(primask);
}

static void tx_kick(void)
{
  uint16_t n;
  uint16_t i;
  uint16_t t;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (tx_busy || (tx_count == 0u))
  {
    __set_PRIMASK(primask);
    return;
  }
  n = (tx_count < TX_DMA_BUF_SIZE) ? tx_count : TX_DMA_BUF_SIZE;
  tx_busy = 1u;                                   /* claim TX, burst <= 256 bytes */
  tx_dma_len = n;
  __set_PRIMASK(primask);

  /* Copy n bytes out of the FIFO. They stay in the FIFO (tx_tail/tx_count not yet
     advanced) until the DMA actually starts, so a failed start loses nothing. */
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
    primask = __get_PRIMASK();
    __disable_irq();
    tx_tail = t;                                  /* commit consumption */
    tx_count = (uint16_t)(tx_count - n);
    __set_PRIMASK(primask);
  }
  else
  {
    primask = __get_PRIMASK();
    __disable_irq();
    tx_busy = 0u;                                 /* roll back, keep FIFO intact */
    tx_dma_len = 0u;
    __set_PRIMASK(primask);
  }
}

/* RX: HT / TC / IDLE for all three ports land here. 'Size' is the ABSOLUTE DMA
   write position in that port's circular buffer (0..RX_BUF_SIZE), not a length.
   Never re-arm ReceiveToIdle here (the circular DMA keeps running). */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  RxPort_t *p;
  uint16_t pos;
  uint16_t old;

  if (huart == rx_usart1.huart)
  {
    p = &rx_usart1;
  }
  else if (huart == rx_uart4.huart)
  {
    p = &rx_uart4;
  }
  else if (huart == rx_lpuart1.huart)
  {
    p = &rx_lpuart1;
  }
  else
  {
    return;
  }

  switch (HAL_UARTEx_GetRxEventType(huart))
  {
    case HAL_UART_RXEVENT_IDLE: p->rx_event_idle_cnt++; break;
    case HAL_UART_RXEVENT_HT:   p->rx_event_ht_cnt++;   break;
    case HAL_UART_RXEVENT_TC:   p->rx_event_tc_cnt++;   break;
    default: break;
  }

  pos = Size;
  if (pos > RX_BUF_SIZE)
  {
    pos = 0u;
  }
  p->write_pos = pos;

  old = p->old_pos;
  if (pos != old)
  {
    if (pos > old)
    {
      uint16_t len = (uint16_t)(pos - old);          /* contiguous [old, pos)     */
      p->rx_bytes += len;
      fifo_push(&p->buf[old], len, p);
    }
    else
    {
      uint16_t len1 = (uint16_t)(RX_BUF_SIZE - old);  /* wrap: [old,end)+[0,pos)  */
      p->rx_bytes += len1;
      fifo_push(&p->buf[old], len1, p);
      if (pos > 0u)
      {
        p->rx_bytes += pos;
        fifo_push(&p->buf[0], pos, p);
      }
    }
    p->old_pos = pos;
    if (p->old_pos >= RX_BUF_SIZE)                     /* Size == RX_BUF_SIZE -> 0 */
    {
      p->old_pos = 0u;
    }
  }

  tx_kick();
}

/* USART3_TX burst finished: release the claim, count bytes, send the next chunk. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == huart_tx)
  {
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    tx_bytes += tx_dma_len;
    tx_dma_len = 0u;
    tx_busy = 0u;
    __set_PRIMASK(primask);
    tx_kick();
  }
}

/* "DMA on RX Error" is Enabled in CubeMX: an overrun (ORE) stops that port's RX
   DMA. Abort (synchronous) and restart ReceiveToIdle, reset positions, or the port
   stalls forever. A TX error just releases the claim so forwarding continues. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  RxPort_t *p;

  if (huart == rx_usart1.huart)
  {
    p = &rx_usart1;
  }
  else if (huart == rx_uart4.huart)
  {
    p = &rx_uart4;
  }
  else if (huart == rx_lpuart1.huart)
  {
    p = &rx_lpuart1;
  }
  else
  {
    p = NULL;
  }

  if (p != NULL)
  {
    p->err_cnt++;
    (void)HAL_UART_AbortReceive(p->huart);          /* synchronous abort */
    p->old_pos = 0u;
    p->write_pos = 0u;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(p->huart, p->buf, RX_BUF_SIZE);
  }
  else if (huart == huart_tx)
  {
    uint32_t primask = __get_PRIMASK();

    tx_err_cnt++;
    __disable_irq();
    tx_busy = 0u;
    tx_dma_len = 0u;
    __set_PRIMASK(primask);
    tx_kick();
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
