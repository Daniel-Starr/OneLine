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

/* ===== OneLine UART bridge (Board 2): direct RX chunk -> TX DMA forwarding === */
#define RX_DMA_BUF_SIZE   256u   /* RX DMA circular buffer (ReceiveToIdle)         */
#define TX_DMA_BUF_SIZE   256u   /* temp buffer handed to TX DMA per burst         */
#define DROP_ALL_ZERO_RX_CHUNKS 1u /* Debug guard for floating/held-low RX lines   */

/* DMA-accessed buffers. STM32U575 is Cortex-M33: no data cache, so no cache
   maintenance / 32-byte alignment is required for DMA coherency. */
static uint8_t rx_dma_buf[RX_DMA_BUF_SIZE];
static uint8_t tx_dma_buf[TX_DMA_BUF_SIZE];

static volatile uint16_t tx_dma_len;

/* RX circular-buffer positions, tracked across HT/TC/IDLE events and thread mode. */
static volatile uint16_t rx_old_pos;
static volatile uint16_t rx_write_pos;

/* Role handles (assigned in USER CODE BEGIN 2). Board 2: RX=USART3, TX=LPUART1. */
static UART_HandleTypeDef *huart_rx;
static UART_HandleTypeDef *huart_tx;

/* Debug counters (inspect live in the debugger). */
static volatile uint32_t rx_bytes;
static volatile uint32_t tx_bytes;
static volatile uint32_t drop_cnt;
static volatile uint32_t err_cnt;
static volatile uint32_t zero_burst_cnt;
static volatile uint32_t zero_drop_bytes;
static volatile uint32_t rx_event_idle_cnt;
static volatile uint32_t rx_event_ht_cnt;
static volatile uint32_t rx_event_tc_cnt;
static volatile uint8_t  tx_busy;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t chunk_is_all_zero(const uint8_t *data, uint16_t len);
static void bridge_poll_rx(void);
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
    bridge_poll_rx();
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

static uint8_t chunk_is_all_zero(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  if (len == 0u)
  {
    return 0u;
  }
  for (i = 0u; i < len; i++)
  {
    if (data[i] != 0u)
    {
      return 0u;
    }
  }
  return 1u;
}

static void bridge_poll_rx(void)
{
  uint16_t old;
  uint16_t pos;
  uint16_t len;
  uint16_t new_old;
  uint16_t i;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  if (tx_busy != 0u)
  {
    __set_PRIMASK(primask);
    return;
  }
  old = rx_old_pos;
  pos = rx_write_pos;
  if (pos > RX_DMA_BUF_SIZE)
  {
    pos = 0u;
  }
  if (pos == old)
  {
    __set_PRIMASK(primask);
    return;
  }

  if (pos > old)
  {
    len = (uint16_t)(pos - old);
  }
  else
  {
    len = (uint16_t)(RX_DMA_BUF_SIZE - old);
  }

  if (len > TX_DMA_BUF_SIZE)
  {
    len = TX_DMA_BUF_SIZE;
  }
  tx_busy = 1u;
  tx_dma_len = len;
  __set_PRIMASK(primask);

  rx_bytes += len;

#if (DROP_ALL_ZERO_RX_CHUNKS != 0u)
  if (chunk_is_all_zero(&rx_dma_buf[old], len) != 0u)
  {
    zero_burst_cnt++;
    zero_drop_bytes += len;
    new_old = (uint16_t)(old + len);
    if (new_old >= RX_DMA_BUF_SIZE)
    {
      new_old = 0u;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    rx_old_pos = new_old;
    tx_busy = 0u;
    tx_dma_len = 0u;
    if (rx_write_pos >= RX_DMA_BUF_SIZE)
    {
      rx_write_pos = 0u;
    }
    __set_PRIMASK(primask);
    return;
  }
#endif

  for (i = 0u; i < len; i++)
  {
    tx_dma_buf[i] = rx_dma_buf[(uint16_t)(old + i)];
  }

  new_old = (uint16_t)(old + len);
  if (new_old >= RX_DMA_BUF_SIZE)
  {
    new_old = 0u;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  rx_old_pos = new_old;
  if (rx_write_pos >= RX_DMA_BUF_SIZE)
  {
    rx_write_pos = 0u;
  }
  __set_PRIMASK(primask);

  if (HAL_UART_Transmit_DMA(huart_tx, tx_dma_buf, len) != HAL_OK)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    rx_old_pos = old;
    tx_busy = 0u;
    tx_dma_len = 0u;
    err_cnt++;
    __set_PRIMASK(primask);
    return;
  }
}

/* RX: HT / TC / IDLE all land here. 'Size' is the ABSOLUTE DMA write position
   inside the circular buffer (0..RX_DMA_BUF_SIZE), not this chunk's length. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == huart_rx)
  {
    HAL_UART_RxEventTypeTypeDef event_type = HAL_UARTEx_GetRxEventType(huart);
    uint16_t pos = Size;

    if (pos > RX_DMA_BUF_SIZE)
    {
      pos = 0u;
    }

    if (event_type == HAL_UART_RXEVENT_IDLE)
    {
      rx_event_idle_cnt++;
    }
    else if (event_type == HAL_UART_RXEVENT_HT)
    {
      rx_event_ht_cnt++;
    }
    else if (event_type == HAL_UART_RXEVENT_TC)
    {
      rx_event_tc_cnt++;
    }

    rx_write_pos = pos;
    bridge_poll_rx();
  }
}

/* TX DMA burst finished: release the claim. New RX chunks start fresh DMA sends. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == huart_tx)
  {
    uint16_t n;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    n = tx_dma_len;
    tx_dma_len = 0u;
    tx_busy = 0u;
    tx_bytes += n;
    __set_PRIMASK(primask);
    bridge_poll_rx();
  }
}

/* "DMA on RX Error" is Enabled in CubeMX, so an overrun (ORE) stops the RX DMA.
   Abort and restart ReceiveToIdle and reset the position tracker, otherwise the
   RX path stalls permanently. */
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
    /* Release the TX claim so a TX-side error cannot stall forwarding forever. */
    uint32_t primask = __get_PRIMASK();

    err_cnt++;
    __disable_irq();
    tx_busy = 0u;
    tx_dma_len = 0u;
    __set_PRIMASK(primask);
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
