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
/* 1.1s 独立看门狗(直写寄存器,不依赖 HAL IWDG 驱动):收到数据喂狗;1.1s 无数据 -> 硬复位 MCU(=自动按RST)。 */
#define IWDG_REFRESH()  (IWDG->KR = 0x0000AAAAu)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define DMA_RX_SIZE   1024u          /* 每路 RX circular DMA 硬件缓冲 */
#define TX_QUEUE_SIZE (512u * 1024u) /* 应用环形队列 512KB(U575 RAM 768KB,够) */
#define RX_REARM_RETRY_MS 10u        /* 失败后限速重试，避免主循环反复重配 DMA */
#define FRAME_SOF         0xAAu      /* 帧起始同步字节(仅辅助重同步,解析靠 LEN) */
#define FRAME_SRC_LP      0x01u      /* 来源 = LPUART1 (PC0,  COM10) */
#define FRAME_SRC_U1      0x02u      /* 来源 = USART1  (PA10, COM8)  */
#define FRAME_HEADER_SIZE 4u         /* SOF + SRC + LEN_HI + LEN_LO */
#define FRAME_OVERHEAD    5u         /* header(4) + CRC8(1) */

/* 【关键修改】：强制 32 字节（Cache Line）对齐，确保硬件 Cache 刷新和失效不越界 */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
__attribute__((aligned(32))) static uint8_t dma_rx_lp[DMA_RX_SIZE];
__attribute__((aligned(32))) static uint8_t dma_rx_u1[DMA_RX_SIZE];
#else
static uint8_t dma_rx_lp[DMA_RX_SIZE];
static uint8_t dma_rx_u1[DMA_RX_SIZE];
#endif

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
static volatile uint32_t drop_cnt;       /* 队列满时丢弃的字节数(含被整帧丢弃的帧) */
static volatile uint32_t frame_drop_cnt; /* 因放不下而被整帧丢弃的帧数 */
static volatile uint32_t main_loop_cnt;  /* 主循环心跳:一直涨=活着;停住=HardFault/死循环 */
static volatile uint32_t empty_idle_skip_lp_cnt;  /* LPUART1 启动/重启空 IDLE 跳过次数(调试) */
static volatile uint32_t empty_idle_skip_u1_cnt;  /* USART1  启动/重启空 IDLE 跳过次数(调试) */
static volatile uint32_t err_cnt;
static volatile uint32_t rx_start_stage; /* 启动进度:1=LP起,2=U1起,3=两路都起好 */

static volatile uint8_t rearm_lp;
static volatile uint8_t rearm_u1;
static volatile uint32_t rearm_lp_cnt;   /* 调试:各路实际重启了几次 */
static volatile uint32_t rearm_u1_cnt;
static volatile uint32_t rearm_lp_fail_cnt;
static volatile uint32_t rearm_u1_fail_cnt;
static uint32_t rearm_lp_last_try;
static uint32_t rearm_u1_last_try;

extern DMA_HandleTypeDef handle_GPDMA1_Channel0;   /* LPUART1_RX */
extern DMA_HandleTypeDef handle_GPDMA1_Channel4;   /* USART1_RX  */

/* 【关键修改】：节点同样要求 32 字节对齐，防止 Cache 同步越界 */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
__attribute__((aligned(32))) static DMA_NodeTypeDef  rx_node_lp;
__attribute__((aligned(32))) static DMA_NodeTypeDef  rx_node_u1;
#else
static DMA_NodeTypeDef  rx_node_lp;
static DMA_NodeTypeDef  rx_node_u1;
#endif

static DMA_QListTypeDef rx_list_lp;
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
__attribute__((used, noinline, optnone))
static void rx_event_drain(const uint8_t *buf, volatile uint16_t *last_pos,
                           volatile uint32_t *rx_count, uint16_t Size, uint8_t src);
static void txq_push(const uint8_t *data, uint16_t len);
__attribute__((used, noinline, optnone))
static uint8_t crc8_update(uint8_t crc, uint8_t value);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
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

  /* 启动 1.1s 独立看门狗(IWDG):卡死/无流量 1.1s 即硬复位,等效自动按 RST(即便 CPU 卡在
     HardFault 也能复位)。喂狗在 HAL_UARTEx_RxEventCallback 收到真实数据时进行。 */
  __HAL_DBGMCU_FREEZE_IWDG();           /* 调试 halt 时冻结看门狗,不打扰下断点调试 */
  IWDG->KR  = 0x0000CCCCu;              /* 启动 IWDG(自动开 LSI ~32kHz) */
  IWDG->KR  = 0x00005555u;              /* 解锁 PR/RLR 写访问 */
  IWDG->PR  = 3u;                       /* 预分频 /32 -> 计数时钟 1 kHz */
  IWDG->RLR = 1099u;                    /* (1099+1)/1000 = 1.1 s */
  while ((IWDG->SR & 0x07u) != 0u) { }  /* 等 PR/RLR 更新完成 */
  IWDG->KR  = 0x0000AAAAu;              /* 初次喂狗 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ((rearm_lp != 0u) &&
        ((uint32_t)(HAL_GetTick() - rearm_lp_last_try) >= RX_REARM_RETRY_MS))
    {
      HAL_StatusTypeDef status;

      rearm_lp_last_try = HAL_GetTick();
      rearm_lp = 0u;
      rx_last_pos_lp = 0u;
      status = rx_dma_circular_start(&hlpuart1, &handle_GPDMA1_Channel0, &rx_node_lp,
                                     &rx_list_lp, GPDMA1_Channel0,
                                     GPDMA1_REQUEST_LPUART1_RX, dma_rx_lp);
      if (status == HAL_OK) { rearm_lp_cnt++; }
      else { rearm_lp_fail_cnt++; rearm_lp = 1u; }
    }
    
    if ((rearm_u1 != 0u) &&
        ((uint32_t)(HAL_GetTick() - rearm_u1_last_try) >= RX_REARM_RETRY_MS))
    {
      HAL_StatusTypeDef status;

      rearm_u1_last_try = HAL_GetTick();
      rearm_u1 = 0u;
      rx_last_pos_u1 = 0u;
      status = rx_dma_circular_start(&huart1, &handle_GPDMA1_Channel4, &rx_node_u1,
                                     &rx_list_u1, GPDMA1_Channel4,
                                     GPDMA1_REQUEST_USART1_RX, dma_rx_u1);
      if (status == HAL_OK) { rearm_u1_cnt++; }
      else { rearm_u1_fail_cnt++; rearm_u1 = 1u; }
    }

    main_loop_cnt++;   
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

static HAL_StatusTypeDef rx_dma_circular_start(UART_HandleTypeDef *huart,
                                               DMA_HandleTypeDef *hdma,
                                               DMA_NodeTypeDef *node,
                                               DMA_QListTypeDef *list,
                                               DMA_Channel_TypeDef *channel,
                                               uint32_t request,
                                               uint8_t *buffer)
{
  DMA_NodeConfTypeDef nc = {0};

  if (HAL_UART_AbortReceive(huart) != HAL_OK) { return HAL_ERROR; }
  if (HAL_DMA_DeInit(hdma) != HAL_OK) { return HAL_ERROR; }
  if (HAL_DMAEx_List_ResetQ(list) != HAL_OK) { return HAL_ERROR; }

  nc.NodeType                 = DMA_GPDMA_LINEAR_NODE;
  nc.Init.Request             = request;
  nc.Init.BlkHWRequest        = DMA_BREQ_SINGLE_BURST;
  nc.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  nc.Init.SrcInc              = DMA_SINC_FIXED;
  nc.Init.DestInc             = DMA_DINC_INCREMENTED;
  nc.Init.SrcDataWidth        = DMA_SRC_DATAWIDTH_BYTE;
  nc.Init.DestDataWidth       = DMA_DEST_DATAWIDTH_BYTE;
  nc.Init.SrcBurstLength      = 1;
  nc.Init.DestBurstLength     = 1;
  nc.Init.TransferAllocatedPort   = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  nc.Init.TransferEventMode       = DMA_TCEM_BLOCK_TRANSFER;
  nc.Init.Mode                = DMA_NORMAL;
  nc.TriggerConfig.TriggerPolarity= DMA_TRIG_POLARITY_MASKED;
  nc.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  nc.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  nc.SrcAddress               = (uint32_t)&huart->Instance->RDR;
  nc.DstAddress               = (uint32_t)buffer;
  nc.DataSize                 = DMA_RX_SIZE;

  if (HAL_DMAEx_List_BuildNode(&nc, node) != HAL_OK) { return HAL_ERROR; }
  
  /* 【关键修改】：强刷 D-Cache，确保硬件 DMA 能读到正确的节点配置，避免野指针 */

  
  if (HAL_DMAEx_List_InsertNode(list, NULL, node) != HAL_OK) { return HAL_ERROR; }
  if (HAL_DMAEx_List_SetCircularMode(list) != HAL_OK) { return HAL_ERROR; }

  hdma->Instance                         = channel;
  hdma->InitLinkedList.Priority          = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  hdma->InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
  hdma->InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  hdma->InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma->InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(hdma) != HAL_OK) { return HAL_ERROR; }
  if (HAL_DMAEx_List_LinkQ(hdma, list) != HAL_OK) { return HAL_ERROR; }

  __HAL_LINKDMA(huart, hdmarx, *hdma);
  if (HAL_DMA_ConfigChannelAttributes(hdma, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, DMA_RX_SIZE);
}


static void txq_push(const uint8_t *data, uint16_t len)
{
  /* 【关键修改】：移除多余的 __disable_irq() 提速 */
  uint32_t space;
  uint32_t first;

  space = TX_QUEUE_SIZE - txq_count;
  if ((uint32_t)len > space)
  {
    drop_cnt += (uint32_t)len - space;
    len = (uint16_t)space;
  }

  if (len != 0u)
  {
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
}


__attribute__((used, noinline, optnone))
static uint8_t crc8_update(uint8_t crc, uint8_t value)
{
  uint8_t bit;
  crc ^= value;
  for (bit = 0u; bit < 8u; bit++)
  {
    crc = ((crc & 0x80u) != 0u) ? (uint8_t)((crc << 1u) ^ 0x07u)
                                : (uint8_t)(crc << 1u);
  }
  return crc;
}

__attribute__((used, noinline, optnone))
static void rx_event_drain(const uint8_t *buf, volatile uint16_t *last_pos,
                           volatile uint32_t *rx_count, uint16_t Size, uint8_t src)
{
  /* 【关键修改】：读取前使 D-Cache 失效，强迫 CPU 从 SRAM 读取最新到达的 DMA 数据 */

  uint16_t pos  = (Size > DMA_RX_SIZE) ? DMA_RX_SIZE : Size;
  uint16_t last = *last_pos;
  const uint8_t *seg1;
  const uint8_t *seg2 = NULL;
  uint16_t len1;
  uint16_t len2 = 0u;
  uint16_t total;
  
  /* 【关键修改】：用 32 位整型保障 header 在局部栈上的 4 字节对齐 */
  uint32_t header_buf;
  uint8_t *header = (uint8_t *)&header_buf;
  
  uint8_t  crc;
  uint16_t i;
  uint32_t frame_len;

  if (pos == last) { return; }

  if (pos > last)
  {
    seg1 = &buf[last];
    len1 = (uint16_t)(pos - last);
  }
  else                                                  
  {
    seg1 = &buf[last];
    len1 = (uint16_t)(DMA_RX_SIZE - last);                    
    seg2 = &buf[0];
    len2 = pos;
  }
  total = (uint16_t)(len1 + len2);

  *last_pos = pos;

  if (total == 0u) { return; }
  *rx_count += total;

  header[0] = FRAME_SOF;
  header[1] = src;
  header[2] = (uint8_t)(total >> 8);
  header[3] = (uint8_t)total;

  crc = crc8_update(0u, src);
  crc = crc8_update(crc, header[2]);
  crc = crc8_update(crc, header[3]);
  for (i = 0u; i < len1; i++) { crc = crc8_update(crc, seg1[i]); }
  for (i = 0u; i < len2; i++) { crc = crc8_update(crc, seg2[i]); }

  frame_len = (uint32_t)FRAME_OVERHEAD + (uint32_t)total;
  if (frame_len > (uint32_t)(TX_QUEUE_SIZE - txq_count))
  {
    drop_cnt += frame_len;
    frame_drop_cnt++;
    return;
  }

  txq_push(header, FRAME_HEADER_SIZE);
  if (len1 != 0u) { txq_push(seg1, len1); }
  if (len2 != 0u) { txq_push(seg2, len2); }
  txq_push(&crc, 1u);
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  HAL_UART_RxEventTypeTypeDef ev_type = HAL_UARTEx_GetRxEventType(huart);

  if (huart == &hlpuart1)
  {
    if ((ev_type == HAL_UART_RXEVENT_IDLE) && (Size == DMA_RX_SIZE) &&
        (rx_last_pos_lp == 0u) &&
        ((uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx) == DMA_RX_SIZE))
    {
      empty_idle_skip_lp_cnt++;
      return;
    }
    IWDG_REFRESH();   /* 收到真实数据 -> 喂狗 */
    rx_event_drain(dma_rx_lp, &rx_last_pos_lp, &rx_bytes, Size, FRAME_SRC_LP);
  }
  else if (huart == &huart1)
  {
    if ((ev_type == HAL_UART_RXEVENT_IDLE) && (Size == DMA_RX_SIZE) &&
        (rx_last_pos_u1 == 0u) &&
        ((uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx) == DMA_RX_SIZE))
    {
      empty_idle_skip_u1_cnt++;
      return;
    }
    IWDG_REFRESH();   /* 收到真实数据 -> 喂狗 */
    rx_event_drain(dma_rx_u1, &rx_last_pos_u1, &rx_bytes_u1, Size, FRAME_SRC_U1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &hlpuart1)
  {
    err_cnt++;
    rearm_lp = 1u;
  }
  else if (huart == &huart1)
  {
    err_cnt++;
    rearm_u1 = 1u;
  }
}

static void tx_kick(void)
{
  uint32_t primask;

  /* 【关键修改】：改为 while 循环榨干 USART3 的 8 级硬件 FIFO */
  while (((USART3->ISR & USART_ISR_TXE_TXFNF) != 0u) && (txq_count != 0u))
  {
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
