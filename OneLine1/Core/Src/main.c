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

/* Board 1 bridge: LPUART1_RX + USART1_RX --(each: circular DMA HT/TC/IDLE)--> 512KB queue
   --> USART3_TX(poll) -> Board 2. 经典 "DMA(半/全传输中断)+串口空闲中断+环形队列",两路合并:
     - RX: LPUART1 与 USART1 各用一路 circular DMA 连续接进各自的 dma_rx_* 缓冲;ReceiveToIdle_DMA
       各装一次、永不重装。HAL 在 HT/TC/IDLE 回调 RxEventCallback(按 huart 区分两路),把新字节
       (环绕处理)搬进同一个 512KB 应用环形队列(两路在此按到达顺序合并)。RX 不在 ISR 调 HAL DMA。
     - TX: 主循环轮询 USART3 TXE,从队列取一字节写 TDR(直写寄存器,无句柄,最稳)。
   关键安全点(上次 wild-write 的根因):每路 circular 链表节点的 DstAddress 显式绑死到对应缓冲,
   且 Size clamp 到 [0,DMA_RX_SIZE] -> DMA 只在各自缓冲内回绕、不越界(已在 board2 单路验证)。
   溢出只丢(drop_cnt),不卡死。U575 RAM 768KB,缓冲开大尽量不丢。 */
#define DMA_RX_SIZE   1024u          /* 每路 RX circular DMA 硬件缓冲 */
#define TX_QUEUE_SIZE (512u * 1024u) /* 应用环形队列 512KB(U575 RAM 768KB,够) */

/* 两路 RX 循环 DMA 缓冲(DMA 硬件写,RxEventCallback 读)+ 各自消费位置 */
static uint8_t dma_rx_lp[DMA_RX_SIZE];         /* LPUART1 */
static uint8_t dma_rx_u1[DMA_RX_SIZE];         /* USART1  */
static volatile uint16_t rx_last_pos_lp;
static volatile uint16_t rx_last_pos_u1;

/* 应用环形队列。生产者:两路 RxEventCallback(ISR)。消费者:tx_kick(主循环)。 */
static uint8_t  txq[TX_QUEUE_SIZE];
static volatile uint32_t txq_head;
static volatile uint32_t txq_tail;
static volatile uint32_t txq_count;

static volatile uint32_t rx_bytes;       /* LPUART1 received bytes */
static volatile uint32_t rx_bytes_u1;    /* USART1  received bytes */
static volatile uint32_t tx_bytes;
static volatile uint32_t drop_cnt;
static volatile uint32_t err_cnt;

/* usart.c 里的 DMA 句柄/节点/队列(Ch0=LPUART1_RX, Ch4=USART1_RX);在 main 里重配成 circular */
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
extern DMA_NodeTypeDef   Node_GPDMA1_Channel0;
extern DMA_HandleTypeDef handle_GPDMA1_Channel4;
extern DMA_NodeTypeDef   Node_GPDMA1_Channel4;

typedef struct
{
  DMA_HandleTypeDef *hdma;
  DMA_NodeTypeDef *node;
  DMA_Channel_TypeDef *channel;
  USART_TypeDef *uart;
  uint8_t *buffer;
  volatile uint16_t *last_pos;
  volatile uint32_t *rx_count;
  volatile uint32_t *ht_count;
  volatile uint32_t *tc_count;
  volatile uint32_t *idle_count;
  volatile uint32_t *dma_error_count;
} Board1_RxPath;

static volatile uint32_t rx_ht_lp;
static volatile uint32_t rx_tc_lp;
static volatile uint32_t rx_idle_lp;
static volatile uint32_t rx_dma_err_lp;
static volatile uint32_t rx_ht_u1;
static volatile uint32_t rx_tc_u1;
static volatile uint32_t rx_idle_u1;
static volatile uint32_t rx_dma_err_u1;
static volatile uint32_t rx_start_stage;
static volatile uint32_t rx_start_error;

static Board1_RxPath rx_path_lp = {
  &handle_GPDMA1_Channel0, &Node_GPDMA1_Channel0, GPDMA1_Channel0, LPUART1,
  dma_rx_lp, &rx_last_pos_lp, &rx_bytes, &rx_ht_lp, &rx_tc_lp, &rx_idle_lp, &rx_dma_err_lp
};
static Board1_RxPath rx_path_u1 = {
  &handle_GPDMA1_Channel4, &Node_GPDMA1_Channel4, GPDMA1_Channel4, USART1,
  dma_rx_u1, &rx_last_pos_u1, &rx_bytes_u1, &rx_ht_u1, &rx_tc_u1, &rx_idle_u1, &rx_dma_err_u1
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef board1_rx_start(Board1_RxPath *path);
static uint16_t board1_rx_write_pos(const Board1_RxPath *path);
static void board1_rx_drain(Board1_RxPath *path, uint16_t pos);
static void board1_rx_dma_irq(Board1_RxPath *path);
static void board1_rx_uart_irq(Board1_RxPath *path);
static void txq_push(const uint8_t *data, uint16_t len);
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

  /* Board 1: LPUART1_RX + USART1_RX 各装一次 circular DMA(永不重装),合并进 512KB 队列;
     TX 轮询 USART3。两路循环 DMA 节点都显式绑定到各自缓冲(circular wild-write 的修复)。 */
  rx_start_stage = 1U;
  if (board1_rx_start(&rx_path_lp) != HAL_OK)
  {
    Error_Handler();
  }
  rx_start_stage = 2U;
  if (board1_rx_start(&rx_path_u1) != HAL_OK)
  {
    Error_Handler();
  }
  rx_start_stage = 3U;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    tx_kick();   /* both RX are DMA + interrupt-driven; the loop only drains the queue to TX */
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

/* Bind an already-linked circular DMA channel, then start it exactly once. */
static HAL_StatusTypeDef board1_rx_start(Board1_RxPath *path)
{
  path->node->LinkRegisters[NODE_CBR1_DEFAULT_OFFSET] = DMA_RX_SIZE;
  path->node->LinkRegisters[NODE_CSAR_DEFAULT_OFFSET] = (uint32_t)&path->uart->RDR;
  path->node->LinkRegisters[NODE_CDAR_DEFAULT_OFFSET] = (uint32_t)path->buffer;
  path->channel->CFCR = DMA_CFCR_TCF | DMA_CFCR_HTF | DMA_CFCR_DTEF |
                        DMA_CFCR_ULEF | DMA_CFCR_USEF | DMA_CFCR_TOF;

  if (HAL_DMAEx_List_Start_IT(path->hdma) != HAL_OK)
  {
    rx_start_error++;
    return HAL_ERROR;
  }

  SET_BIT(path->channel->CCR, DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_DTEIE |
                              DMA_CCR_ULEIE | DMA_CCR_USEIE | DMA_CCR_TOIE);
  SET_BIT(path->uart->CR3, USART_CR3_DMAR | USART_CR3_EIE);
  SET_BIT(path->uart->CR1, USART_CR1_IDLEIE);
  return HAL_OK;
}

/* Push a slice into the shared 512KB queue (producer = RX events, ISR context). Full -> drop.
   All RX ISRs are NVIC priority 0 (serialized), so the only race is vs the main-loop consumer,
   guarded by this short critical section. */
static void txq_push(const uint8_t *data, uint16_t len)
{
  uint16_t i;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  for (i = 0u; i < len; i++)
  {
    if (txq_count >= TX_QUEUE_SIZE)
    {
      drop_cnt++;
      continue;
    }
    txq[txq_head] = data[i];
    txq_head++;
    if (txq_head >= TX_QUEUE_SIZE)
    {
      txq_head = 0u;
    }
    txq_count++;
  }
  __set_PRIMASK(primask);
}

static uint16_t board1_rx_write_pos(const Board1_RxPath *path)
{
  uint32_t remaining = path->channel->CBR1 & DMA_CBR1_BNDT;

  if (remaining > DMA_RX_SIZE)
  {
    remaining = DMA_RX_SIZE;
  }
  return (uint16_t)(DMA_RX_SIZE - remaining);
}

/* Copy only bytes produced since the last HT, TC or IDLE event. */
static void board1_rx_drain(Board1_RxPath *path, uint16_t pos)
{
  uint16_t last = *path->last_pos;

  if (pos > DMA_RX_SIZE)
  {
    pos = DMA_RX_SIZE;
  }
  if (pos == last)
  {
    return;
  }
  if (pos > last)
  {
    *path->rx_count += (uint32_t)(pos - last);
    txq_push(&path->buffer[last], (uint16_t)(pos - last));
  }
  else
  {
    *path->rx_count += (uint32_t)(DMA_RX_SIZE - last) + pos;
    txq_push(&path->buffer[last], (uint16_t)(DMA_RX_SIZE - last));
    if (pos != 0U)
    {
      txq_push(&path->buffer[0], pos);
    }
  }
  *path->last_pos = (pos == DMA_RX_SIZE) ? 0U : pos;
}

static void board1_rx_dma_irq(Board1_RxPath *path)
{
  uint32_t status = path->channel->CSR;
  uint32_t clear = 0U;

  if ((status & DMA_CSR_HTF) != 0U)
  {
    clear |= DMA_CFCR_HTF;
    (*path->ht_count)++;
    board1_rx_drain(path, DMA_RX_SIZE / 2U);
  }
  if ((status & DMA_CSR_TCF) != 0U)
  {
    clear |= DMA_CFCR_TCF;
    (*path->tc_count)++;
    board1_rx_drain(path, DMA_RX_SIZE);
  }
  if ((status & (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF | DMA_CSR_TOF)) != 0U)
  {
    clear |= DMA_CFCR_DTEF | DMA_CFCR_ULEF | DMA_CFCR_USEF | DMA_CFCR_TOF;
    (*path->dma_error_count)++;
    err_cnt++;
  }
  if (clear != 0U)
  {
    path->channel->CFCR = clear;
  }
}

static void board1_rx_uart_irq(Board1_RxPath *path)
{
  uint32_t status = path->uart->ISR;
  uint32_t clear = 0U;

  if ((status & USART_ISR_IDLE) != 0U)
  {
    clear |= USART_ICR_IDLECF;
  }
  if ((status & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) != 0U)
  {
    clear |= USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
    err_cnt++;
  }
  if (clear != 0U)
  {
    path->uart->ICR = clear;
  }
  if ((status & USART_ISR_IDLE) != 0U)
  {
    (*path->idle_count)++;
    board1_rx_drain(path, board1_rx_write_pos(path));
  }
}

void Board1_Lpuart1DmaIrq(void)
{
  board1_rx_dma_irq(&rx_path_lp);
}

void Board1_Usart1DmaIrq(void)
{
  board1_rx_dma_irq(&rx_path_u1);
}

void Board1_Lpuart1UartIrq(void)
{
  board1_rx_uart_irq(&rx_path_lp);
}

void Board1_Usart1UartIrq(void)
{
  board1_rx_uart_irq(&rx_path_u1);
}

/* Drain the queue to USART3 by writing TDR directly when TXE is set. Non-blocking and
   handle-free; the fast main loop keeps TDR fed for full line rate. */
static void tx_kick(void)
{
  uint32_t primask;

  if (((USART3->ISR & USART_ISR_TXE_TXFNF) == 0u) || (txq_count == 0u))
  {
    return;
  }

  USART3->TDR = txq[txq_tail];
  txq_tail++;
  if (txq_tail >= TX_QUEUE_SIZE)
  {
    txq_tail = 0u;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  txq_count--;
  __set_PRIMASK(primask);
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
