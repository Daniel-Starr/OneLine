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

/* Board 2 bridge (协议网关 / CRC 过滤): USART3_RX --(circular DMA: HT/TC/IDLE)--> 协议解析 --> 512KB
   ring queue --> LPUART1_TX(poll) -> PC. 经典 "DMA(半/全传输中断)+串口空闲中断+环形队列":
     - RX: USART3 用 circular DMA 连续接进 dma_rx_buf;ReceiveToIdle_DMA 正常运行不重装,UART 错误后由主
       循环重建。HAL 在 半满/满/空闲 回调 RxEventCallback(Size=当前写位置),回调把新到字节(环绕处理)
       喂给协议解析器(跨回调保存状态,一个回调 != 一帧)。RX 全程不在 ISR 里调任何 HAL DMA 函数。
     - 解析: 找 SOF 0xAA -> SRC(0x01/0x02) -> LEN(大端,1..1024) -> 收齐 PAYLOAD+CRC8,用与 board1
       完全一致的 CRC-8/ATM 校验。CRC 对 -> 把"完整原始帧"整帧压队列;CRC/SRC/LEN 错 -> 丢弃并重找 0xAA。
       复位噪声/无效数据形不成合法帧,到不了电脑。一帧要么整帧入队、要么整帧丢(无半包)。
     - TX: 主循环轮询 LPUART1 TXE,从队列取字节写 TDR(直写寄存器,无句柄,最稳)。 */
#define DMA_RX_SIZE   1024u          /* USART3 RX circular DMA 硬件缓冲 */
#define TX_QUEUE_SIZE (512u * 1024u) /* 应用环形队列 512KB,用于吸收瞬时突发 */
#define RX_REARM_RETRY_MS 10u        /* 失败后限速重试,避免主循环反复重配 DMA */

/* board1 协议帧: [0xAA][SRC][LEN_H][LEN_L][PAYLOAD...][CRC8]
   SRC: 0x01=LPUART1, 0x02=USART1; LEN 大端 = PAYLOAD 字节数(1..1024);
   CRC-8/ATM(poly 0x07, init 0) 覆盖 SRC、LEN_H、LEN_L 和全部 PAYLOAD。 */
#define FRAME_SOF          0xAAu
#define FRAME_SRC_LP       0x01u
#define FRAME_SRC_U1       0x02u
#define FRAME_HEADER_SIZE  4u                                          /* SOF + SRC + LEN_H + LEN_L */
#define FRAME_MIN_PAYLOAD  1u
#define FRAME_MAX_PAYLOAD  DMA_RX_SIZE                                 /* 1024 */
#define FRAME_MAX_SIZE     (FRAME_HEADER_SIZE + FRAME_MAX_PAYLOAD + 1u)/* +CRC = 1029 */

/* USART3 RX 循环 DMA 缓冲:DMA 硬件写,RxEventCallback 读。32 字节对齐(无害;以后若开 Cache 才有意义)。 */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
__attribute__((aligned(32))) static uint8_t dma_rx_buf[DMA_RX_SIZE];
#else
static uint8_t dma_rx_buf[DMA_RX_SIZE];
#endif
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
static volatile uint8_t  rearm_rx;
static volatile uint32_t rearm_rx_cnt;
static volatile uint32_t rearm_rx_fail_cnt;
static uint32_t rearm_rx_last_try;

/* 协议解析器状态(跨 DMA 回调保存)。生产者=RxEventCallback(ISR),单路 USART3,无并发。 */
static uint8_t  frame_buf[FRAME_MAX_SIZE];   /* 正在收集的候选帧(SOF..CRC) */
static uint16_t frame_pos;                   /* 已收集字节数 */
static uint16_t frame_payload_len;           /* 头部解析出的 LEN */
static uint16_t frame_total;                 /* 整帧字节数 = 头(4)+LEN+CRC(1) */
static volatile uint32_t frame_ok_cnt;       /* CRC 通过、已转发的帧数 */
static volatile uint32_t frame_crc_err_cnt;  /* CRC 错丢弃 */
static volatile uint32_t frame_fmt_err_cnt;  /* SRC/LEN/格式错丢弃 */
static volatile uint32_t frame_drop_cnt;     /* 合法帧但队列放不下而整帧丢弃 */
static volatile uint32_t noise_drop_bytes;   /* 帧外噪声字节(复位 0x00 等) */
static volatile uint32_t empty_idle_skip_cnt;    /* 启动/重启空 IDLE 跳过次数(调试) */
static volatile uint32_t parser_rearm_reset_cnt; /* 因 UART 错误恢复而重置解析器的次数(调试) */

/* usart.c 里的 DMA 句柄/节点/队列(在 main 里重配成 circular 并绑定缓冲) */
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;
extern DMA_NodeTypeDef   Node_GPDMA1_Channel1;
extern DMA_QListTypeDef  List_GPDMA1_Channel1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef rx_dma_circular_start(void);
static uint8_t txq_push(const uint8_t *data, uint16_t len);
static void tx_kick(void);
static uint8_t crc8_update(uint8_t crc, uint8_t value);
static void parser_reset(void);
static void parser_reject(uint8_t value);
static uint8_t parser_handle_byte(uint8_t value);
static void parser_resync(void);
static void parser_feed(const uint8_t *data, uint16_t len);
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

  /* Board 2: USART3_RX (circular DMA, HT/TC/IDLE) -> 协议解析 -> 512KB queue -> LPUART1_TX (poll).
     Normal reception runs continuously; UART errors are recovered later from the main loop. */
  parser_reset();                         /* clean parser state before the first DMA start */
  if (rx_dma_circular_start() != HAL_OK)
  {
    Error_Handler();
  }

  /* 启动 1.1s 独立看门狗(IWDG):卡死/无流量 1.1s 即硬复位,等效自动按 RST(即便 CPU 卡死也能复位)。
     喂狗在 HAL_UARTEx_RxEventCallback 收到真实数据时进行。 */
  __HAL_DBGMCU_FREEZE_IWDG();           /* 调试 halt 时冻结看门狗 */
  IWDG->KR  = 0x0000CCCCu;              /* 启动 IWDG(自动开 LSI ~32kHz) */
  IWDG->KR  = 0x00005555u;              /* 解锁 PR/RLR */
  IWDG->PR  = 3u;                       /* /32 -> 1 kHz */
  IWDG->RLR = 1099u;                    /* (1099+1)/1000 = 1.1 s */
  while ((IWDG->SR & 0x07u) != 0u) { }  /* 等更新完成 */
  IWDG->KR  = 0x0000AAAAu;              /* 初次喂狗 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ((rearm_rx != 0u) &&
        ((uint32_t)(HAL_GetTick() - rearm_rx_last_try) >= RX_REARM_RETRY_MS))
    {
      HAL_StatusTypeDef status;

      rearm_rx_last_try = HAL_GetTick();
      rearm_rx = 0u;
      rx_last_pos = 0u;
      parser_reset();                    /* a UART error may have hit mid-frame: drop the partial */
      parser_rearm_reset_cnt++;
      status = rx_dma_circular_start();
      if (status == HAL_OK)
      {
        rearm_rx_cnt++;
      }
      else
      {
        rearm_rx_fail_cnt++;
        rearm_rx = 1u;
      }
    }

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

/* Configure USART3 RX as CIRCULAR DMA into dma_rx_buf and (re)start it. The linked-list node's
   DstAddress/DataSize are bound EXPLICITLY to dma_rx_buf so each wrap reloads the correct dest.
   Abort + DeInit + ResetQ first so this is safe both at boot and when re-arming after a UART error. */
static HAL_StatusTypeDef rx_dma_circular_start(void)
{
  DMA_NodeConfTypeDef nc = {0};

  if (HAL_UART_AbortReceive(&huart3) != HAL_OK) { return HAL_ERROR; }
  if (HAL_DMA_DeInit(&handle_GPDMA1_Channel1) != HAL_OK) { return HAL_ERROR; }
  if (HAL_DMAEx_List_ResetQ(&List_GPDMA1_Channel1) != HAL_OK) { return HAL_ERROR; }

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

  if (HAL_DMAEx_List_BuildNode(&nc, &Node_GPDMA1_Channel1) != HAL_OK)                          { return HAL_ERROR; }
  if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel1, NULL, &Node_GPDMA1_Channel1) != HAL_OK) { return HAL_ERROR; }
  if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel1) != HAL_OK)                         { return HAL_ERROR; }

  handle_GPDMA1_Channel1.Instance                         = GPDMA1_Channel1;
  handle_GPDMA1_Channel1.InitLinkedList.Priority          = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  handle_GPDMA1_Channel1.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
  handle_GPDMA1_Channel1.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  handle_GPDMA1_Channel1.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  handle_GPDMA1_Channel1.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel1) != HAL_OK)                                  { return HAL_ERROR; }
  if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel1, &List_GPDMA1_Channel1) != HAL_OK)          { return HAL_ERROR; }

  __HAL_LINKDMA(&huart3, hdmarx, handle_GPDMA1_Channel1);
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel1, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    return HAL_ERROR;
  }

  rx_last_pos = 0u;
  return HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dma_rx_buf, DMA_RX_SIZE);
}

/* Push a slice into the application ring queue. Whole-or-nothing: if it does not fit, drop ALL of
   it (return 0) so a frame is never half-written. Producer = parser (ISR); consumer = tx_kick. */
static uint8_t txq_push(const uint8_t *data, uint16_t len)
{
  uint32_t space;
  uint32_t first;

  space = TX_QUEUE_SIZE - txq_count;
  if ((uint32_t)len > space)
  {
    drop_cnt += len;
    return 0u;
  }

  if (len != 0u)
  {
    first = TX_QUEUE_SIZE - txq_head;
    if (first > (uint32_t)len)
    {
      first = len;
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
  return 1u;
}

/* CRC-8/ATM: poly 0x07, init 0x00, no reflection, no final XOR. MUST match board1 exactly --
   covers SRC, LEN_H, LEN_L and every PAYLOAD byte. */
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

static void parser_reset(void)
{
  frame_pos = 0u;
  frame_payload_len = 0u;
  frame_total = 0u;
}

/* Reject the current candidate. If the offending byte is itself a SOF, reuse it as the start of
   the next frame so data arriving right after garbage re-syncs immediately. */
static void parser_reject(uint8_t value)
{
  frame_fmt_err_cnt++;
  parser_reset();
  if (value == FRAME_SOF)
  {
    frame_buf[0] = value;
    frame_pos = 1u;
  }
}

/* Process ONE received byte through the frame state machine. Returns 1 iff a complete frame just
   FAILED CRC and the caller should resync; returns 0 otherwise. Never recurses. */
static uint8_t parser_handle_byte(uint8_t value)
{
  if (frame_pos == 0u)                           /* hunting for SOF */
  {
    if (value == FRAME_SOF)
    {
      frame_buf[0] = value;
      frame_pos = 1u;
    }
    else
    {
      noise_drop_bytes++;
    }
    return 0u;
  }

  if (frame_pos == 1u)                           /* SRC: only 0x01 / 0x02 */
  {
    if ((value == FRAME_SRC_LP) || (value == FRAME_SRC_U1))
    {
      frame_buf[1] = value;
      frame_pos = 2u;
    }
    else
    {
      parser_reject(value);                      /* SRC error -> fast single-byte resync */
    }
    return 0u;
  }

  if (frame_pos == 2u)                           /* LEN_H */
  {
    frame_buf[2] = value;
    frame_pos = 3u;
    return 0u;
  }

  if (frame_pos == 3u)                           /* LEN_L -> validate length, fix total */
  {
    frame_buf[3] = value;
    frame_payload_len = (uint16_t)(((uint16_t)frame_buf[2] << 8) | (uint16_t)value);
    if ((frame_payload_len < FRAME_MIN_PAYLOAD) || (frame_payload_len > FRAME_MAX_PAYLOAD))
    {
      parser_reject(value);                      /* LEN out of range -> fast resync */
      return 0u;
    }
    frame_total = (uint16_t)(FRAME_HEADER_SIZE + frame_payload_len + 1u);
    frame_pos = 4u;
    return 0u;
  }

  /* collecting PAYLOAD followed by the 1-byte CRC */
  frame_buf[frame_pos] = value;
  frame_pos++;
  if (frame_pos >= frame_total)                  /* frame complete -> verify CRC */
  {
    uint8_t crc = 0u;
    uint16_t i;

    crc = crc8_update(crc, frame_buf[1]);        /* SRC   */
    crc = crc8_update(crc, frame_buf[2]);        /* LEN_H */
    crc = crc8_update(crc, frame_buf[3]);        /* LEN_L */
    for (i = 0u; i < frame_payload_len; i++)
    {
      crc = crc8_update(crc, frame_buf[FRAME_HEADER_SIZE + i]);
    }

    if (crc == frame_buf[frame_total - 1u])
    {
      if (txq_push(frame_buf, frame_total) != 0u)   /* whole frame, or dropped whole */
      {
        frame_ok_cnt++;
      }
      else
      {
        frame_drop_cnt++;
      }
      parser_reset();
      return 0u;
    }

    /* CRC error: leave frame_buf intact (frame_pos == frame_total) so the caller can resync. */
    frame_crc_err_cnt++;
    return 1u;
  }
  return 0u;
}

/* CRC-error recovery: a bad CRC may have swallowed a real frame that started INSIDE the candidate.
   Find the next SOF after the false leading SOF and re-feed from there. Fully iterative + bounded
   (carry is strictly shorter), and a nested CRC failure during the re-feed just resets -- no recursion. */
static void parser_resync(void)
{
  static uint8_t carry[FRAME_MAX_SIZE];   /* ISR-only, single producer -> safe to reuse */
  uint16_t total = frame_total;           /* frame_pos == frame_total on a CRC failure */
  uint16_t k = 1u;
  uint16_t carry_len;
  uint16_t c;

  while ((k < total) && (frame_buf[k] != FRAME_SOF))
  {
    k++;
  }
  if (k >= total)                          /* no embedded SOF -> whole candidate is noise */
  {
    noise_drop_bytes += total;
    parser_reset();
    return;
  }

  carry_len = (uint16_t)(total - k);
  memcpy(carry, &frame_buf[k], carry_len);
  noise_drop_bytes += k;                    /* discarded prefix [0 .. k-1] */
  parser_reset();

  for (c = 0u; c < carry_len; c++)
  {
    if (parser_handle_byte(carry[c]) != 0u)  /* nested CRC failure -> drop & keep scanning */
    {
      noise_drop_bytes += frame_total;
      parser_reset();
    }
  }
}

/* Feed RX bytes (one wrap segment) through the parser. A CRC failure triggers an in-candidate
   resync; a complete CRC-valid frame is pushed whole (txq_push). Bad SRC/LEN/CRC frames and
   inter-frame noise (reset 0x00 bursts, glitches) never reach the PC. */
static void parser_feed(const uint8_t *data, uint16_t len)
{
  uint16_t n;

  for (n = 0u; n < len; n++)
  {
    if (parser_handle_byte(data[n]) != 0u)
    {
      parser_resync();
    }
  }
}

/* RX event from circular DMA. Fires at half-transfer, transfer-complete and UART idle. Feed the
   bytes received since the last position (wrap-aware) into the frame parser. Position uses HAL's
   Size directly (clamped). A full-buffer TC keeps last == DMA_RX_SIZE so the spurious follow-up
   IDLE that re-reports Size == DMA_RX_SIZE is de-duplicated by pos == last. The empty startup/restart
   IDLE (event==IDLE, Size==full, last==0, counter==full) is skipped so the zeroed buffer is not
   scanned/forwarded. No HAL DMA call here. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart3)
  {
    uint16_t pos;
    uint16_t last;

    if ((HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE) && (Size == DMA_RX_SIZE) &&
        (rx_last_pos == 0u) &&
        ((uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx) == DMA_RX_SIZE))
    {
      empty_idle_skip_cnt++;
      return;
    }

    IWDG_REFRESH();   /* 收到真实数据 -> 喂狗(1.1s 内没数据则硬复位) */

    pos  = (Size > DMA_RX_SIZE) ? DMA_RX_SIZE : Size;   /* clamp: never index OOB */
    last = rx_last_pos;

    if (pos != last)
    {
      if (pos > last)                            /* contiguous: buf[last .. pos-1] */
      {
        rx_bytes += (uint32_t)(pos - last);
        parser_feed(&dma_rx_buf[last], (uint16_t)(pos - last));
      }
      else                                       /* wrapped: buf[last..end] then buf[0..pos-1] */
      {
        uint16_t tail = (uint16_t)(DMA_RX_SIZE - last);   /* 0 when last == DMA_RX_SIZE */

        rx_bytes += (uint32_t)tail + (uint32_t)pos;
        if (tail != 0u)
        {
          parser_feed(&dma_rx_buf[last], tail);
        }
        if (pos != 0u)
        {
          parser_feed(&dma_rx_buf[0], pos);
        }
      }
      rx_last_pos = pos;   /* keep pos as-is; do NOT collapse DMA_RX_SIZE -> 0 */
    }
  }
}

/* HAL aborts DMA reception after a UART error. Flag the path for re-arm in the main loop so the
   callback stays short and never reconfigures DMA from interrupt context. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    err_cnt++;
    rearm_rx = 1u;
  }
}

/* Drain the queue to LPUART1 by writing TDR directly while TXE is set. Non-blocking, handle-free. */
static void tx_kick(void)
{
  uint32_t primask;

  while (((LPUART1->ISR & USART_ISR_TXE_TXFNF) != 0u) && (txq_count != 0u))
  {
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
