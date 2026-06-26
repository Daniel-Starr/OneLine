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

/* Board 2 bridge: USART3_RX --(circular DMA: HT/TC/IDLE)--> 32KB ring queue --> LPUART1_TX(poll) -> PC.
   经典 "DMA(半/全传输中断) + 串口空闲中断 + 环形队列" 方案:
     - RX: USART3 用 circular DMA 连续接进 dma_rx_buf[DMA_RX_SIZE];ReceiveToIdle_DMA 装一次、
       永不重装。HAL 在 半满(HT)/满(TC)/空闲(IDLE) 都回调 RxEventCallback(Size=当前写位置),
       回调把新到的字节(环绕处理)搬进应用环形队列。RX 全程不在 ISR 里调任何 HAL DMA 函数。
     - TX: 主循环轮询 LPUART1 TXE,从队列取一字节写 TDR(直写寄存器,无句柄,最稳)。
   关键安全点(上次 wild-write 的根因):circular 链表节点的 DstAddress 显式绑死到 dma_rx_buf,
   且对 Size clamp 到 [0,DMA_RX_SIZE] -> DMA 永远只在 dma_rx_buf 内回绕、不越界、不会写坏句柄。
   U575 内存大,缓冲开大尽量不丢;真溢出只丢数据(drop_cnt),不卡死。 */
#define DMA_RX_SIZE   1024u          /* USART3 RX circular DMA 硬件缓冲 */
#define TX_QUEUE_SIZE (512u * 1024u) /* 应用环形队列 512KB(U575 RAM 768KB,够);兜底约 44s @115200 */

/* USART3 RX 循环 DMA 缓冲:DMA 硬件写,RxEventCallback 读 */
static uint8_t dma_rx_buf[DMA_RX_SIZE];
static volatile uint16_t rx_last_pos;          /* 上次已消费到的 dma_rx_buf 位置 */

/* 应用环形队列。生产者:RxEventCallback(ISR)。消费者:tx_kick(主循环)。 */
static uint8_t  txq[TX_QUEUE_SIZE];
static volatile uint32_t txq_head;
static volatile uint32_t txq_tail;
static volatile uint32_t txq_count;

static volatile uint32_t rx_bytes;
static volatile uint32_t tx_bytes;
static volatile uint32_t drop_cnt;
static volatile uint32_t err_cnt;

/* usart.c 里的 DMA 句柄/节点/队列(在 main 里重配成 circular 并绑定缓冲) */
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;
extern DMA_NodeTypeDef   Node_GPDMA1_Channel1;
extern DMA_QListTypeDef  List_GPDMA1_Channel1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef rx_dma_circular_start(void);
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
  /* USER CODE BEGIN 2 */

  /* Board 2: USART3_RX (circular DMA, HT/TC/IDLE) -> 32KB queue -> LPUART1_TX (poll).
     Arm the circular DMA ONCE; it then runs forever -- no re-arm, no HAL DMA call in any ISR. */
  (void)rx_dma_circular_start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    tx_kick();   /* RX is DMA + interrupt-driven; the loop only drains the queue to TX */
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

/* Configure USART3 RX as CIRCULAR DMA into dma_rx_buf and start it once. The fix vs the
   old crash: the linked-list node's DstAddress/DataSize are bound EXPLICITLY to dma_rx_buf,
   so on every wrap the GPDMA reloads the correct destination (the unbound node was what
   wild-wrote RAM before). ReceiveToIdle gives HT/TC/IDLE events via RxEventCallback. */
static HAL_StatusTypeDef rx_dma_circular_start(void)
{
  DMA_NodeConfTypeDef nc = {0};

  /* usart.c built this channel as one-shot NORMAL; tear it down and rebuild as circular LL. */
  (void)HAL_DMA_DeInit(&handle_GPDMA1_Channel1);

  nc.NodeType                         = DMA_GPDMA_LINEAR_NODE;
  nc.Init.Request                     = GPDMA1_REQUEST_USART3_RX;
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
  nc.SrcAddress                       = (uint32_t)&USART3->RDR;   /* BIND src = RDR        */
  nc.DstAddress                       = (uint32_t)dma_rx_buf;     /* BIND dst = buffer (FIX)*/
  nc.DataSize                         = DMA_RX_SIZE;              /* BIND length            */

  if (HAL_DMAEx_List_BuildNode(&nc, &Node_GPDMA1_Channel1) != HAL_OK)                       { return HAL_ERROR; }
  if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel1, NULL, &Node_GPDMA1_Channel1) != HAL_OK) { return HAL_ERROR; }
  if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel1) != HAL_OK)                      { return HAL_ERROR; }

  handle_GPDMA1_Channel1.Instance                         = GPDMA1_Channel1;
  handle_GPDMA1_Channel1.InitLinkedList.Priority          = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  handle_GPDMA1_Channel1.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
  handle_GPDMA1_Channel1.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  handle_GPDMA1_Channel1.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  handle_GPDMA1_Channel1.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel1) != HAL_OK)                               { return HAL_ERROR; }
  if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel1, &List_GPDMA1_Channel1) != HAL_OK)       { return HAL_ERROR; }

  __HAL_LINKDMA(&huart3, hdmarx, handle_GPDMA1_Channel1);
  (void)HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel1, DMA_CHANNEL_NPRIV);

  rx_last_pos = 0u;
  return HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dma_rx_buf, DMA_RX_SIZE);
}

/* Push a slice into the application ring queue (producer = RX event, ISR context). Queue
   full -> drop. Short critical section guards txq_count against the main-loop consumer. */
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

/* RX event from circular DMA: Size = current DMA write position in dma_rx_buf [0..DMA_RX_SIZE].
   Fires at half-transfer, transfer-complete and UART idle. Copy everything new since the last
   position (wrap-aware) into the queue. No HAL DMA call here -- the DMA just keeps running. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart3)
  {
    uint16_t pos  = (Size > DMA_RX_SIZE) ? DMA_RX_SIZE : Size;   /* clamp: never index OOB */
    uint16_t last = rx_last_pos;

    if (pos != last)
    {
      if (pos > last)
      {
        rx_bytes += (uint32_t)(pos - last);
        txq_push(&dma_rx_buf[last], (uint16_t)(pos - last));
      }
      else                                                       /* wrapped past the end */
      {
        rx_bytes += (uint32_t)(DMA_RX_SIZE - last) + pos;
        txq_push(&dma_rx_buf[last], (uint16_t)(DMA_RX_SIZE - last));
        if (pos > 0u)
        {
          txq_push(&dma_rx_buf[0], pos);
        }
      }
      rx_last_pos = (pos >= DMA_RX_SIZE) ? 0u : pos;
    }
  }
}

/* Circular DMA keeps running across UART errors; just count them. No HAL DMA call here. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    err_cnt++;
  }
}

/* Drain the queue to LPUART1 by writing TDR directly when TXE is set. Non-blocking and
   handle-free; the fast main loop keeps TDR fed for full line rate. */
static void tx_kick(void)
{
  uint32_t primask;

  if (((LPUART1->ISR & USART_ISR_TXE_TXFNF) == 0u) || (txq_count == 0u))
  {
    return;
  }

  LPUART1->TDR = txq[txq_tail];
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
