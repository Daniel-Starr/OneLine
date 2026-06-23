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

/* Board 1 bridge: LPUART1_RX + USART1_RX -> shared TX FIFO -> USART3_TX -> Board 2.
   The two RX (ReceiveToIdle_IT) callbacks push each frame into the FIFO; the main loop
   drains the FIFO to USART3 via TX DMA. TX is started ONLY from the main loop and is
   gated on the HAL TX state (gState) - HAL resets it to READY on completion/error, so
   a missed/raced completion cannot wedge forwarding. FIFO full -> drop (drop_cnt). */
#define BRIDGE_BUF_SIZE 256u
#define TX_FIFO_SIZE    4096u

static uint8_t lpuart1_rx_buf[BRIDGE_BUF_SIZE];   /* LPUART1 RX (ReceiveToIdle_IT) */
static uint8_t usart1_rx_buf[BRIDGE_BUF_SIZE];    /* USART1  RX (ReceiveToIdle_IT) */
static uint8_t usart3_tx_buf[BRIDGE_BUF_SIZE];    /* USART3 TX DMA staging         */

/* Shared TX FIFO. Producers: the two RX callbacks (ISR). Consumer: tx_kick (main). */
static uint8_t  tx_fifo[TX_FIFO_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint16_t tx_count;
static volatile uint16_t tx_dma_len;

static volatile uint32_t rx_bytes;       /* LPUART1 received bytes */
static volatile uint32_t rx_bytes_u1;    /* USART1  received bytes */
static volatile uint32_t tx_bytes;
static volatile uint32_t drop_cnt;
static volatile uint32_t err_cnt;
static volatile uint32_t tx_err_cnt;
static volatile uint32_t rx_event_idle_cnt;
static volatile uint32_t rx_event_tc_cnt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void bridge_rx_arm(UART_HandleTypeDef *huart, uint8_t *buf);
static void fifo_push(const uint8_t *data, uint16_t len);
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

  /* Board 1: LPUART1_RX and USART1_RX both forward to USART3_TX -> Board 2. */
  bridge_rx_arm(&hlpuart1, lpuart1_rx_buf);
  bridge_rx_arm(&huart1, usart1_rx_buf);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

/* Arm one RX port with ReceiveToIdle in INTERRUPT mode (one-shot; re-armed after each
   frame in the RX-event callback). Never call Error_Handler (no deadlock): on failure
   abort once and retry, then just count the error. */
static void bridge_rx_arm(UART_HandleTypeDef *huart, uint8_t *buf)
{
  if (HAL_UARTEx_ReceiveToIdle_IT(huart, buf, BRIDGE_BUF_SIZE) == HAL_OK)
  {
    return;
  }
  (void)HAL_UART_AbortReceive(huart);
  if (HAL_UARTEx_ReceiveToIdle_IT(huart, buf, BRIDGE_BUF_SIZE) != HAL_OK)
  {
    err_cnt++;
  }
}

/* Push a received frame into the shared TX FIFO (producer side, called from the RX
   ISRs). FIFO full -> drop the overflow bytes and count them. Short critical section
   guards the FIFO against the main-loop consumer. */
static void fifo_push(const uint8_t *data, uint16_t len)
{
  uint16_t i;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  for (i = 0u; i < len; i++)
  {
    if (tx_count >= TX_FIFO_SIZE)
    {
      drop_cnt++;
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

/* Drain the FIFO to USART3 via TX DMA. Called ONLY from the main loop. Gated on the
   HAL TX state, which HAL sets back to READY on completion/error - so it can never get
   stuck on a manual busy flag. Non-blocking: returns if TX is busy or the FIFO empty. */
static void tx_kick(void)
{
  uint16_t n;
  uint16_t i;
  uint16_t t;
  uint32_t primask;

  if (huart3.gState != HAL_UART_STATE_READY)
  {
    return;                                  /* a TX DMA is still in flight */
  }

  primask = __get_PRIMASK();
  __disable_irq();
  n = (tx_count < BRIDGE_BUF_SIZE) ? tx_count : BRIDGE_BUF_SIZE;
  __set_PRIMASK(primask);
  if (n == 0u)
  {
    return;
  }

  /* Peek n bytes out of the FIFO (don't advance until the DMA actually starts). */
  t = tx_tail;
  for (i = 0u; i < n; i++)
  {
    usart3_tx_buf[i] = tx_fifo[t];
    t++;
    if (t >= TX_FIFO_SIZE)
    {
      t = 0u;
    }
  }

  if (HAL_UART_Transmit_DMA(&huart3, usart3_tx_buf, n) == HAL_OK)
  {
    tx_dma_len = n;
    primask = __get_PRIMASK();
    __disable_irq();
    tx_tail = t;
    tx_count = (uint16_t)(tx_count - n);
    __set_PRIMASK(primask);
  }
  else
  {
    tx_err_cnt++;                            /* keep FIFO data, retry next loop */
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t len = Size;

  if (len > BRIDGE_BUF_SIZE)
  {
    len = BRIDGE_BUF_SIZE;
  }

  if (huart == &hlpuart1)
  {
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE)
    {
      rx_event_idle_cnt++;
    }
    else
    {
      rx_event_tc_cnt++;
    }
    if (len > 0u)
    {
      rx_bytes += len;
      fifo_push(lpuart1_rx_buf, len);
    }
    bridge_rx_arm(&hlpuart1, lpuart1_rx_buf);
  }
  else if (huart == &huart1)
  {
    if (len > 0u)
    {
      rx_bytes_u1 += len;
      fifo_push(usart1_rx_buf, len);
    }
    bridge_rx_arm(&huart1, usart1_rx_buf);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    tx_bytes += tx_dma_len;
    tx_dma_len = 0u;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &hlpuart1)
  {
    err_cnt++;
    (void)HAL_UART_AbortReceive(&hlpuart1);
    bridge_rx_arm(&hlpuart1, lpuart1_rx_buf);
  }
  else if (huart == &huart1)
  {
    err_cnt++;
    (void)HAL_UART_AbortReceive(&huart1);
    bridge_rx_arm(&huart1, usart1_rx_buf);
  }
  else if (huart == &huart3)
  {
    tx_err_cnt++;                            /* HAL resets gState->READY; tx_kick resumes */
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
