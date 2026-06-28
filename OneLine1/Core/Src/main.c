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

/* Board 1 bridge: LPUART1_RX(PC0) + USART1_RX(PA10) --(each: circular DMA, HT/TC/IDLE)-->
   512KB ring queue --> USART3_TX(PA7, poll) -> Board 2. 经典 "DMA(半/全传输中断)+串口空闲中断+
   环形队列",两路按到达顺序合并到一条 TX:
     - RX: LPUART1 与 USART1 各用一路 circular DMA 连续接进各自的 dma_rx_* 缓冲;ReceiveToIdle_DMA
       各装一次、永不重装。HAL 在 半满(HT)/满(TC)/空闲(IDLE) 回调 HAL_UARTEx_RxEventCallback
       (按 huart 区分两路),把新到字节(环绕处理)搬进同一个 512KB 应用环形队列。RX 全程不在
       ISR 里调任何 HAL DMA 函数。
     - TX: 主循环轮询 USART3 TXE,从队列取一字节写 TDR(直写寄存器,无句柄,最稳)。
   关键安全点(上次 wild-write/HardFault 的根因):每路 circular 链表节点都用 HAL_DMAEx_List_BuildNode
   带 DstAddress/DataSize 重新构建并绑死到对应缓冲(与已验证的 board2 完全相同的修复),DMA 只在
   各自缓冲内回绕、绝不越界写。溢出只丢(drop_cnt)不卡死。U575 RAM 768KB,缓冲开大尽量不丢。 */
#define DMA_RX_SIZE   1024u          /* 每路 RX circular DMA 硬件缓冲 */
#define TX_QUEUE_SIZE (512u * 1024u) /* 应用环形队列 512KB(U575 RAM 768KB,够) */

/* 两路 RX 循环 DMA 缓冲(DMA 硬件写,RxEventCallback 读)+ 各自消费位置 */
static uint8_t dma_rx_lp[DMA_RX_SIZE];          /* LPUART1 (PC0)  */
static uint8_t dma_rx_u1[DMA_RX_SIZE];          /* USART1  (PA10) */
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
static volatile uint32_t rx_start_stage; /* 启动进度:1=LP起,2=U1起,3=两路都起好 */

/* 两路 RX DMA 通道句柄(来自 usart.c)。MspInit 把它们建成 circular 链表并已把节点插进
   List_GPDMA1_Channel0/4(NodeNumber 已=1);若 main 再往这些表里 InsertNode 会把 NodeNumber
   重复计成 2 -> 链表遍历走飞 -> DMA 回绕时按错误节点重载目标地址 -> 野写崩溃。
   因此这里 DeInit 后改用我们"自己的全新空队列"重建(等价于 board2:它的 MspInit 用 DMA_NORMAL,
   List 本就是空的)。 */
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;   /* LPUART1_RX */
extern DMA_HandleTypeDef handle_GPDMA1_Channel4;   /* USART1_RX  */

/* 两路 RX 用的全新链表队列 + 节点(在 main 里构建)。static => 整个运行期都在,供 DMA 读取。 */
static DMA_NodeTypeDef  rx_node_lp;
static DMA_QListTypeDef rx_list_lp;
static DMA_NodeTypeDef  rx_node_u1;
static DMA_QListTypeDef rx_list_u1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef rx_dma_circular_start(UART_HandleTypeDef *huart,
                                               DMA_HandleTypeDef *hdma,
                                               DMA_NodeTypeDef *node,
                                               DMA_QListTypeDef *list,
                                               DMA_Channel_TypeDef *channel,
                                               uint32_t request,
                                               uint8_t *buffer);
static void rx_event_drain(const uint8_t *buf, volatile uint16_t *last_pos,
                           volatile uint32_t *rx_count, uint16_t Size);
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

  /* A previous run's circular DMA can still be active across a warm reset (debugger reflash, a
     core-only RST, or a watchdog/fault reset do NOT reset GPDMA). Re-arming an already-running
     channel makes HAL_DMAEx_List_Start_IT dereference stale list state and fault intermittently
     at startup. Force-reset GPDMA1 first so we always arm from a clean, idle peripheral. */
  __HAL_RCC_GPDMA1_CLK_ENABLE();
  __HAL_RCC_GPDMA1_FORCE_RESET();
  __HAL_RCC_GPDMA1_RELEASE_RESET();

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

  /* Board 1: 两路 RX 各装一次 circular DMA(永不重装),合并进 512KB 队列;TX 轮询 USART3。
     两路节点都带 DstAddress 重建并绑死到各自缓冲(circular wild-write/HardFault 的根治)。 */
  rx_start_stage = 1U;
  if (rx_dma_circular_start(&hlpuart1, &handle_GPDMA1_Channel0, &rx_node_lp,
                            &rx_list_lp, GPDMA1_Channel0,
                            GPDMA1_REQUEST_LPUART1_RX, dma_rx_lp) != HAL_OK)
  {
    Error_Handler();
  }
  rx_start_stage = 2U;
  if (rx_dma_circular_start(&huart1, &handle_GPDMA1_Channel4, &rx_node_u1,
                            &rx_list_u1, GPDMA1_Channel4,
                            GPDMA1_REQUEST_USART1_RX, dma_rx_u1) != HAL_OK)
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

/* Configure one UART's RX as CIRCULAR DMA into `buffer` and start it once. The fix vs the old
   crash: the linked-list node's DstAddress/DataSize are bound EXPLICITLY to `buffer` (rebuilt
   here, not patched), so on every wrap the GPDMA reloads the correct destination -- the unbound
   node built in MspInit was what wild-wrote RAM and HardFaulted before. ReceiveToIdle then gives
   HT/TC/IDLE events via HAL_UARTEx_RxEventCallback. Identical recipe to board2, used for both
   LPUART1 and USART1. */
static HAL_StatusTypeDef rx_dma_circular_start(UART_HandleTypeDef *huart,
                                               DMA_HandleTypeDef *hdma,
                                               DMA_NodeTypeDef *node,
                                               DMA_QListTypeDef *list,
                                               DMA_Channel_TypeDef *channel,
                                               uint32_t request,
                                               uint8_t *buffer)
{
  DMA_NodeConfTypeDef nc = {0};

  /* MspInit built this channel as circular LL but with an UNBOUND node; tear down & rebuild. */
  (void)HAL_DMA_DeInit(hdma);

  nc.NodeType                         = DMA_GPDMA_LINEAR_NODE;
  nc.Init.Request                     = request;
  nc.Init.BlkHWRequest                = DMA_BREQ_SINGLE_BURST;
  nc.Init.Direction                   = DMA_PERIPH_TO_MEMORY;
  nc.Init.SrcInc                      = DMA_SINC_FIXED;
  nc.Init.DestInc                     = DMA_DINC_INCREMENTED;
  nc.Init.SrcDataWidth                = DMA_SRC_DATAWIDTH_BYTE;
  nc.Init.DestDataWidth               = DMA_DEST_DATAWIDTH_BYTE;
  nc.Init.SrcBurstLength              = 1;
  nc.Init.DestBurstLength             = 1;
  nc.Init.TransferAllocatedPort       = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  nc.Init.TransferEventMode           = DMA_TCEM_BLOCK_TRANSFER;
  nc.Init.Mode                        = DMA_NORMAL;   /* circular-ness comes from SetCircularMode */
  nc.TriggerConfig.TriggerPolarity    = DMA_TRIG_POLARITY_MASKED;
  nc.DataHandlingConfig.DataExchange  = DMA_EXCHANGE_NONE;
  nc.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  nc.SrcAddress                       = (uint32_t)&huart->Instance->RDR;   /* BIND src = RDR        */
  nc.DstAddress                       = (uint32_t)buffer;                  /* BIND dst = buffer (FIX)*/
  nc.DataSize                         = DMA_RX_SIZE;                       /* BIND length            */

  if (HAL_DMAEx_List_BuildNode(&nc, node) != HAL_OK)                        { return HAL_ERROR; }
  if (HAL_DMAEx_List_InsertNode(list, NULL, node) != HAL_OK)                { return HAL_ERROR; }
  if (HAL_DMAEx_List_SetCircularMode(list) != HAL_OK)                       { return HAL_ERROR; }

  hdma->Instance                         = channel;
  hdma->InitLinkedList.Priority          = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  hdma->InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
  hdma->InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  hdma->InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma->InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(hdma) != HAL_OK)                                  { return HAL_ERROR; }
  if (HAL_DMAEx_List_LinkQ(hdma, list) != HAL_OK)                           { return HAL_ERROR; }

  __HAL_LINKDMA(huart, hdmarx, *hdma);
  (void)HAL_DMA_ConfigChannelAttributes(hdma, DMA_CHANNEL_NPRIV);

  /* last_pos for both paths is zero-initialized (.bss) and this runs once at boot. */
  return HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, DMA_RX_SIZE);
}

/* Push a slice into the shared 512KB queue (producer = RX events, ISR context). Full -> drop.
   Both RX paths' ISRs are NVIC priority 0 (serialized), so the only race is vs the main-loop
   consumer, guarded by this short critical section. */
static void txq_push(const uint8_t *data, uint16_t len)
{
  uint32_t primask = __get_PRIMASK();
  uint32_t space;
  uint32_t first;

  __disable_irq();

  /* On overflow, drop the part that doesn't fit (never overrun the ring). */
  space = TX_QUEUE_SIZE - txq_count;
  if ((uint32_t)len > space)
  {
    drop_cnt += (uint32_t)len - space;
    len = (uint16_t)space;
  }

  if (len != 0u)
  {
    /* Bulk copy with at most one wrap; update head/count ONCE (no per-byte loop -> the
       optimizer can't reuse a counter value as a pointer, which was the bus-fault bug). */
    first = TX_QUEUE_SIZE - txq_head;
    if (first > (uint32_t)len)
    {
      first = (uint32_t)len;
    }
    memcpy(&txq[txq_head], data, first);
    if ((uint32_t)len > first)
    {
      memcpy(&txq[0], data + first, (uint32_t)len - first);
    }

    txq_head += len;
    if (txq_head >= TX_QUEUE_SIZE)
    {
      txq_head -= TX_QUEUE_SIZE;
    }
    txq_count += len;
  }

  __set_PRIMASK(primask);
}

/* Copy bytes produced since this path's last event (wrap-aware) into the shared queue.
   pos = current DMA write position in `buf` [0..DMA_RX_SIZE]. No HAL DMA call here. */
static void rx_event_drain(const uint8_t *buf, volatile uint16_t *last_pos,
                           volatile uint32_t *rx_count, uint16_t Size)
{
  uint16_t pos  = (Size > DMA_RX_SIZE) ? DMA_RX_SIZE : Size;   /* clamp: never index OOB */
  uint16_t last = *last_pos;

  if (pos == last)
  {
    return;
  }
  if (pos > last)
  {
    *rx_count += (uint32_t)(pos - last);
    txq_push(&buf[last], (uint16_t)(pos - last));
  }
  else                                                          /* wrapped past the end */
  {
    *rx_count += (uint32_t)(DMA_RX_SIZE - last) + pos;
    txq_push(&buf[last], (uint16_t)(DMA_RX_SIZE - last));
    if (pos > 0u)
    {
      txq_push(&buf[0], pos);
    }
  }
  *last_pos = (pos >= DMA_RX_SIZE) ? 0u : pos;
}

/* RX event from a circular DMA (half-transfer / transfer-complete / UART idle). Distinguish the
   two paths by huart and drain each into the shared queue (merged by arrival order). */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &hlpuart1)
  {
    rx_event_drain(dma_rx_lp, &rx_last_pos_lp, &rx_bytes, Size);
  }
  else if (huart == &huart1)
  {
    rx_event_drain(dma_rx_u1, &rx_last_pos_u1, &rx_bytes_u1, Size);
  }
}

/* Circular DMA keeps running across UART errors; just count them. No HAL DMA call here. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart == &hlpuart1) || (huart == &huart1))
  {
    err_cnt++;
  }
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
