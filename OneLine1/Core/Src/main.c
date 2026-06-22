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

/* Board 1 bring-up bridge: LPUART1_RX -> USART3_TX.
   This intentionally follows the known-good single_chuankou_test_01 receive model:
   one ReceiveToIdle DMA frame is copied into a static TX buffer, then the main loop
   starts a USART3 TX DMA burst. */
#define BRIDGE_BUF_SIZE 256u

static uint8_t lpuart1_rx_buf[BRIDGE_BUF_SIZE];
static uint8_t usart3_tx_buf[BRIDGE_BUF_SIZE];

static volatile uint16_t pending_tx_len;
static volatile uint16_t tx_dma_len;
static volatile uint8_t frame_ready;
static volatile uint8_t tx_busy;

static volatile uint32_t rx_bytes;
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
static void bridge_rx_start(void);
static void bridge_tx_poll(void);
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

  /* Board 1 minimal link: LPUART1_RX -> USART3_TX -> Board 2. */
  bridge_rx_start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    bridge_tx_poll();
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

static void bridge_rx_start(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, lpuart1_rx_buf, BRIDGE_BUF_SIZE) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);
}

static void bridge_tx_poll(void)
{
  uint16_t len;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  if ((frame_ready == 0u) || (tx_busy != 0u))
  {
    __set_PRIMASK(primask);
    return;
  }
  len = pending_tx_len;
  frame_ready = 0u;
  tx_busy = 1u;
  tx_dma_len = len;
  __set_PRIMASK(primask);

  if ((len == 0u) || (len > BRIDGE_BUF_SIZE))
  {
    primask = __get_PRIMASK();
    __disable_irq();
    tx_busy = 0u;
    tx_dma_len = 0u;
    __set_PRIMASK(primask);
    return;
  }

  if (HAL_UART_Transmit_DMA(&huart3, usart3_tx_buf, len) != HAL_OK)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    frame_ready = 1u;        /* keep the frame for the next main-loop attempt */
    tx_busy = 0u;
    tx_dma_len = 0u;
    __set_PRIMASK(primask);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t len;
  uint16_t i;

  if (huart != &hlpuart1)
  {
    return;
  }

  if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE)
  {
    rx_event_idle_cnt++;
  }
  else
  {
    rx_event_tc_cnt++;
  }

  len = Size;
  if (len > BRIDGE_BUF_SIZE)
  {
    len = BRIDGE_BUF_SIZE;
  }

  if (len > 0u)
  {
    if ((frame_ready != 0u) || (tx_busy != 0u))
    {
      drop_cnt += len;
    }
    else
    {
      for (i = 0u; i < len; i++)
      {
        usart3_tx_buf[i] = lpuart1_rx_buf[i];
      }
      pending_tx_len = len;
      frame_ready = 1u;
      rx_bytes += len;
    }
  }

  bridge_rx_start();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    tx_bytes += tx_dma_len;
    tx_dma_len = 0u;
    tx_busy = 0u;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &hlpuart1)
  {
    err_cnt++;
    (void)HAL_UART_AbortReceive(&hlpuart1);
    pending_tx_len = 0u;
    frame_ready = 0u;
    bridge_rx_start();
  }
  else if (huart == &huart3)
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
