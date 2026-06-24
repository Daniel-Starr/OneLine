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
   Mirrors Board 1: RX uses ReceiveToIdle + CIRCULAR DMA, armed once and never
   re-armed, so the RX callback never calls any HAL DMA function from interrupt
   context. The callback copies the new slice of the circular buffer into the shared
   FIFO (position-based, wrap-aware). The main loop drains the FIFO to LPUART1 with a
   bounded BLOCKING transmit, so a stuck TX times out instead of hanging forever.
   DMA is only re-armed from the main loop (error recovery), never from an ISR, and
   nothing depends on a TX-complete interrupt, so it cannot wedge or HardFault. FIFO
   full -> drop (drop_cnt). */
#define BRIDGE_BUF_SIZE 256u
#define TX_FIFO_SIZE    4096u
#define TX_TIMEOUT_MS   100u    /* bounded so a stuck TX can never block forever */

static uint8_t usart3_rx_buf[BRIDGE_BUF_SIZE];    /* USART3 RX circular DMA buffer */
static uint8_t lpuart1_tx_buf[BRIDGE_BUF_SIZE];   /* LPUART1 TX staging           */

/* Shared TX FIFO. Producer: the RX callback (ISR). Consumer: tx_kick (main loop). */
static uint8_t  tx_fifo[TX_FIFO_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint16_t tx_count;

static volatile uint32_t rx_bytes;
static volatile uint32_t tx_bytes;
static volatile uint32_t drop_cnt;
static volatile uint32_t err_cnt;
static volatile uint32_t tx_err_cnt;
static volatile uint32_t rx_event_idle_cnt;
static volatile uint32_t rx_event_tc_cnt;
static volatile uint16_t usart3_rx_pos;   /* last consumed pos in USART3 circular buf */
static volatile uint8_t  usart3_rx_err;   /* RX error -> main loop re-arms the DMA      */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef bridge_rx_arm(UART_HandleTypeDef *huart, uint8_t *buf);
static void bridge_rx_recover(void);
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
  /* USER CODE BEGIN 2 */

  /* Board 2: USART3_RX -> LPUART1_TX -> PC. Circular DMA: arm ONCE; it then runs
     continuously (no re-arm), so the RX callback never calls any HAL DMA function. */
  (void)bridge_rx_arm(&huart3, usart3_rx_buf);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    bridge_rx_recover();
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

/* Arm USART3 RX with ReceiveToIdle + CIRCULAR DMA. Called once at startup; the
   circular DMA then runs continuously, so the RX callback never re-arms and never
   calls any HAL DMA function from interrupt context. HT is left ENABLED so the
   half-transfer event lets us drain the buffer before the DMA wraps over it. */
static HAL_StatusTypeDef bridge_rx_arm(UART_HandleTypeDef *huart, uint8_t *buf)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, BRIDGE_BUF_SIZE) == HAL_OK)
  {
    return HAL_OK;
  }

  err_cnt++;
  return HAL_ERROR;
}

/* Recover the port whose circular DMA the HAL aborted on a UART error (the HAL aborts
   RX DMA on ANY error during DMA reception). Without this the port stays dead until
   reset. Runs in the MAIN LOOP (thread context) so calling HAL DMA here is safe; the
   ISR only sets the flag. Clear the flag first, then re-arm; if the abort isn't
   finished yet (ReceiveToIdle returns !=OK) set the flag again to retry next loop. */
static void bridge_rx_recover(void)
{
  if (usart3_rx_err != 0u)
  {
    usart3_rx_err = 0u;
    usart3_rx_pos = 0u;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_rx_buf, BRIDGE_BUF_SIZE) != HAL_OK)
    {
      usart3_rx_err = 1u;
    }
  }
}

/* Push a received slice into the shared TX FIFO (producer, called from the RX ISR).
   FIFO full -> drop the overflow bytes and count them. Short critical section guards
   the FIFO against the main-loop consumer. */
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

/* Drain the FIFO to LPUART1 with a bounded BLOCKING transmit. Called ONLY from the
   main loop. No TX-complete interrupt or busy flag is involved, so it cannot wedge;
   a stuck TX just times out (tx_err_cnt) and is retried on the next loop. */
static void tx_kick(void)
{
  uint16_t n;
  uint16_t i;
  uint16_t t;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  n = (tx_count < BRIDGE_BUF_SIZE) ? tx_count : BRIDGE_BUF_SIZE;
  __set_PRIMASK(primask);
  if (n == 0u)
  {
    return;
  }

  /* Copy n bytes out of the FIFO (don't advance until the transmit succeeds). */
  t = tx_tail;
  for (i = 0u; i < n; i++)
  {
    lpuart1_tx_buf[i] = tx_fifo[t];
    t++;
    if (t >= TX_FIFO_SIZE)
    {
      t = 0u;
    }
  }

  if (HAL_UART_Transmit(&hlpuart1, lpuart1_tx_buf, n, TX_TIMEOUT_MS) == HAL_OK)
  {
    tx_bytes += n;
    primask = __get_PRIMASK();
    __disable_irq();
    tx_tail = t;
    tx_count = (uint16_t)(tx_count - n);
    __set_PRIMASK(primask);
  }
  else
  {
    tx_err_cnt++;                            /* timeout/error: keep data, retry next loop */
  }
}

/* Circular-DMA RX event: Size is the current DMA write position in the circular
   buffer (0..BRIDGE_BUF_SIZE). Copy everything new since our last position into the
   shared FIFO, handling the wrap. No re-arm -- the circular DMA keeps running. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart3)
  {
    uint16_t pos = usart3_rx_pos;

    if (Size > BRIDGE_BUF_SIZE)
    {
      Size = BRIDGE_BUF_SIZE;
    }

    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE)
    {
      rx_event_idle_cnt++;
    }
    else
    {
      rx_event_tc_cnt++;
    }

    if (Size != pos)
    {
      if (Size > pos)
      {
        rx_bytes += (uint32_t)(Size - pos);
        fifo_push(&usart3_rx_buf[pos], (uint16_t)(Size - pos));
      }
      else
      {
        rx_bytes += (uint32_t)(BRIDGE_BUF_SIZE - pos) + Size;
        fifo_push(&usart3_rx_buf[pos], (uint16_t)(BRIDGE_BUF_SIZE - pos));
        if (Size > 0u)
        {
          fifo_push(&usart3_rx_buf[0], Size);
        }
      }
      usart3_rx_pos = (Size >= BRIDGE_BUF_SIZE) ? 0u : Size;
    }
  }
}

/* The HAL aborts the RX DMA on ANY error during DMA reception, so the port would stay
   dead. Don't re-arm here (ISR context) -- just flag it; bridge_rx_recover() re-arms it
   from the main loop. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    err_cnt++;
    usart3_rx_err = 1u;
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
