# Board 1 DMA/IDLE Receive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox ('- [ ]') syntax for tracking.

**Goal:** Build a stable two-input board-1 bridge using circular GPDMA, HT/TC/IDLE IRQs, and the existing shared TX ring.

**Architecture:** Channel 0 and 4 are initialized as circular linked-list DMA in the UART MSP functions. Main starts each already-linked channel exactly once, binds its node to a dedicated buffer, and enables DMAR/IDLEIE. Direct DMA and UART handlers compute new data and push it to the TX ring; they never invoke the HAL UART receive state machine.

**Tech Stack:** STM32U575, STM32U5 HAL linked-list initialization, CMSIS register access, Keil MDK, PowerShell.

---

## File map

| File | Change |
| --- | --- |
| 'tools/test-board1-rx-architecture.ps1' | Regression check for the direct RX architecture. |
| 'OneLine1/Core/Src/usart.c' | Generate Channel 0/4 as circular linked lists. |
| 'OneLine1/Core/Src/main.c' | Start, drain and direct IRQ wrappers. |
| 'OneLine1/Core/Inc/main.h' | Public IRQ wrappers. |
| 'OneLine1/Core/Src/stm32u5xx_it.c' | Direct RX vectors and fault capture. |

### Task 1: Write a failing architecture regression test

**Files:**
- Create: 'C:\Users\wang2\Desktop\test\OneLine\tools\test-board1-rx-architecture.ps1'
- Test: 'C:\Users\wang2\Desktop\test\OneLine\tools\test-board1-rx-architecture.ps1'

- [ ] **Step 1: Create the failing test**

~~~powershell
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$main = Get-Content -Raw (Join-Path $root 'OneLine1/Core/Src/main.c')
$irq = Get-Content -Raw (Join-Path $root 'OneLine1/Core/Src/stm32u5xx_it.c')
$header = Get-Content -Raw (Join-Path $root 'OneLine1/Core/Inc/main.h')
$usart = Get-Content -Raw (Join-Path $root 'OneLine1/Core/Src/usart.c')
$failures = [System.Collections.Generic.List[string]]::new()

foreach ($p in @('Board1_Lpuart1DmaIrq','Board1_Usart1DmaIrq','Board1_Lpuart1UartIrq','Board1_Usart1UartIrq','fault_cfsr','fault_pc')) {
  if ($main -notmatch $p -and $irq -notmatch $p -and $header -notmatch $p) { $failures.Add("missing: $p") }
}
foreach ($p in @('HAL_DMA_DeInit\(','HAL_UARTEx_ReceiveToIdle_DMA\(','HAL_UART_IRQHandler\(&hlpuart1\)','HAL_UART_IRQHandler\(&huart1\)')) {
  if ($main -match $p -or $irq -match $p) { $failures.Add("forbidden RX path: $p") }
}
foreach ($p in @(
  'HAL_DMAEx_List_BuildNode\(&NodeConfig, &Node_GPDMA1_Channel0\)',
  'HAL_DMAEx_List_SetCircularMode\(&List_GPDMA1_Channel0\)',
  'HAL_DMAEx_List_BuildNode\(&NodeConfig, &Node_GPDMA1_Channel4\)',
  'HAL_DMAEx_List_SetCircularMode\(&List_GPDMA1_Channel4\)')) {
  if ($usart -notmatch $p) { $failures.Add("missing circular config: $p") }
}
if ($failures.Count) { $failures | ForEach-Object { Write-Error $_ }; exit 1 }
Write-Host 'PASS: board1 RX uses direct HT/TC/IDLE IRQ architecture.'
~~~

- [ ] **Step 2: Verify RED**

Run:

~~~powershell
& 'C:\Users\wang2\Desktop\test\OneLine\tools\test-board1-rx-architecture.ps1'
~~~

Expected: exit code '1', citing the old 'HAL_DMA_DeInit' and 'HAL_UARTEx_ReceiveToIdle_DMA' paths.

- [ ] **Step 3: Commit**

~~~powershell
git add -- tools/test-board1-rx-architecture.ps1
git commit -m "test: define board1 direct RX architecture"
~~~

### Task 2: Configure Channel 0 and Channel 4 as circular DMA lists

**Files:**
- Modify: 'C:\Users\wang2\Desktop\test\OneLine\OneLine1\Core\Src\usart.c:254-280'
- Modify: 'C:\Users\wang2\Desktop\test\OneLine\OneLine1\Core\Src\usart.c:417-443'
- Test: 'C:\Users\wang2\Desktop\test\OneLine\tools\test-board1-rx-architecture.ps1'

- [ ] **Step 1: Replace the LPUART1 normal-DMA block**

Replace the Channel0 'HAL_DMA_Init' block in 'HAL_UART_MspInit' with the same linked-list structure currently used by Channel3, changing only the request and identifiers:

~~~c
NodeConfig.NodeType = DMA_GPDMA_LINEAR_NODE;
NodeConfig.Init.Request = GPDMA1_REQUEST_LPUART1_RX;
NodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
NodeConfig.Init.Direction = DMA_PERIPH_TO_MEMORY;
NodeConfig.Init.SrcInc = DMA_SINC_FIXED;
NodeConfig.Init.DestInc = DMA_DINC_INCREMENTED;
NodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
NodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
NodeConfig.Init.SrcBurstLength = 1;
NodeConfig.Init.DestBurstLength = 1;
NodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
NodeConfig.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
NodeConfig.Init.Mode = DMA_NORMAL;
NodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
NodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
NodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel0) != HAL_OK) { Error_Handler(); }
if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel0, NULL, &Node_GPDMA1_Channel0) != HAL_OK) { Error_Handler(); }
if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel0) != HAL_OK) { Error_Handler(); }
handle_GPDMA1_Channel0.Instance = GPDMA1_Channel0;
handle_GPDMA1_Channel0.InitLinkedList.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
handle_GPDMA1_Channel0.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
handle_GPDMA1_Channel0.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
handle_GPDMA1_Channel0.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
handle_GPDMA1_Channel0.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel0) != HAL_OK) { Error_Handler(); }
if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel0, &List_GPDMA1_Channel0) != HAL_OK) { Error_Handler(); }
__HAL_LINKDMA(uartHandle, hdmarx, handle_GPDMA1_Channel0);
if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel0, DMA_CHANNEL_NPRIV) != HAL_OK) { Error_Handler(); }
~~~

- [ ] **Step 2: Apply the identical linked-list pattern to USART1**

Paste the same code in the USART1 block, replacing every Channel0 identifier with Channel4 and 'GPDMA1_REQUEST_LPUART1_RX' with 'GPDMA1_REQUEST_USART1_RX'.

- [ ] **Step 3: Build**

Run:

~~~powershell
Set-Location 'C:\Users\wang2\Desktop\test\OneLine\OneLine1\MDK-ARM'
& 'C:\Users\wang2\AppData\Local\Keil_v5\UV4\UV4.exe' -b .\OneLine1.uvprojx -j0 -o .\board1_dma_idle_task2_build.log
~~~

Expected: '0 Error(s), 0 Warning(s).'

- [ ] **Step 4: Commit**

~~~powershell
git add -- OneLine1/Core/Src/usart.c
git commit -m "fix(board1): initialize RX DMA as circular lists"
~~~

### Task 3: Implement start-once, drain and direct IRQ wrappers

**Files:**
- Modify: 'C:\Users\wang2\Desktop\test\OneLine\OneLine1\Core\Src\main.c:48-367'
- Modify: 'C:\Users\wang2\Desktop\test\OneLine\OneLine1\Core\Inc\main.h:1-101'
- Test: 'C:\Users\wang2\Desktop\test\OneLine\tools\test-board1-rx-architecture.ps1'

- [ ] **Step 1: Add per-channel state**

Add this type and counters in 'main.c' user variables, then instantiate one context for Channel0/LPUART1/'dma_rx_lp' and one for Channel4/USART1/'dma_rx_u1':

~~~c
typedef struct {
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

static volatile uint32_t rx_ht_lp, rx_tc_lp, rx_idle_lp, rx_dma_err_lp;
static volatile uint32_t rx_ht_u1, rx_tc_u1, rx_idle_u1, rx_dma_err_u1;
static volatile uint32_t rx_start_stage, rx_start_error;
~~~

- [ ] **Step 2: Replace the old dynamic conversion with a start-once function**

Delete 'rx_dma_circular_start' and its two calls. Add this function and invoke it once for each context after all UART initialization. On any return other than 'HAL_OK', increment 'rx_start_error' and call 'Error_Handler()'.

~~~c
static HAL_StatusTypeDef board1_rx_start(Board1_RxPath *path)
{
  path->node->LinkRegisters[NODE_CBR1_DEFAULT_OFFSET] = DMA_RX_SIZE;
  path->node->LinkRegisters[NODE_CSAR_DEFAULT_OFFSET] = (uint32_t)&path->uart->RDR;
  path->node->LinkRegisters[NODE_CDAR_DEFAULT_OFFSET] = (uint32_t)path->buffer;
  path->channel->CFCR = DMA_CFCR_TCF | DMA_CFCR_HTF | DMA_CFCR_DTEF |
                        DMA_CFCR_ULEF | DMA_CFCR_USEF | DMA_CFCR_TOF;
  if (HAL_DMAEx_List_Start_IT(path->hdma) != HAL_OK) {
    rx_start_error++;
    return HAL_ERROR;
  }
  SET_BIT(path->channel->CCR, DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_DTEIE |
                              DMA_CCR_ULEIE | DMA_CCR_USEIE | DMA_CCR_TOIE);
  SET_BIT(path->uart->CR3, USART_CR3_DMAR | USART_CR3_EIE);
  SET_BIT(path->uart->CR1, USART_CR1_IDLEIE);
  return HAL_OK;
}
~~~

- [ ] **Step 3: Implement the shared ring advance**

Replace 'rx_event' with these helpers. HT calls 'board1_rx_drain(path, DMA_RX_SIZE / 2U)', TC calls it with 'DMA_RX_SIZE', and IDLE passes the result of 'board1_rx_write_pos'.

~~~c
static uint16_t board1_rx_write_pos(const Board1_RxPath *path)
{
  uint32_t remaining = path->channel->CBR1 & DMA_CBR1_BNDT;
  if (remaining > DMA_RX_SIZE) { remaining = DMA_RX_SIZE; }
  return (uint16_t)(DMA_RX_SIZE - remaining);
}

static void board1_rx_drain(Board1_RxPath *path, uint16_t pos)
{
  uint16_t last = *path->last_pos;
  if (pos == last) { return; }
  if (pos > last) {
    *path->rx_count += (uint32_t)(pos - last);
    txq_push(&path->buffer[last], (uint16_t)(pos - last));
  } else {
    *path->rx_count += (uint32_t)(DMA_RX_SIZE - last) + pos;
    txq_push(&path->buffer[last], (uint16_t)(DMA_RX_SIZE - last));
    if (pos != 0U) { txq_push(&path->buffer[0], pos); }
  }
  *path->last_pos = (pos == DMA_RX_SIZE) ? 0U : pos;
}
~~~

- [ ] **Step 4: Add public direct IRQ wrappers**

Add to 'main.h':

~~~c
void Board1_Lpuart1DmaIrq(void);
void Board1_Usart1DmaIrq(void);
void Board1_Lpuart1UartIrq(void);
void Board1_Usart1UartIrq(void);
~~~

Implement them in 'main.c'. The DMA wrappers read 'CSR', clear HT/TC/error flags in 'CFCR', increment their context counters, then call 'board1_rx_drain'. The UART wrappers clear 'USART_ICR_IDLECF' then drain at 'board1_rx_write_pos'; they clear PE/FE/NE/ORE with the matching ICR bits and increment 'err_cnt'. None of these four functions may call HAL.

- [ ] **Step 5: Verify GREEN and commit**

Run:

~~~powershell
& 'C:\Users\wang2\Desktop\test\OneLine\tools\test-board1-rx-architecture.ps1'
git add -- OneLine1/Core/Src/main.c OneLine1/Core/Inc/main.h
git commit -m "feat(board1): drain circular DMA from direct IRQs"
~~~

Expected: test prints its PASS line before the commit.

### Task 4: Route vectors directly and capture the next fault

**Files:**
- Modify: 'C:\Users\wang2\Desktop\test\OneLine\OneLine1\Core\Src\stm32u5xx_it.c:57-324'
- Test: 'C:\Users\wang2\Desktop\test\OneLine\tools\test-board1-rx-architecture.ps1'

- [ ] **Step 1: Replace only the four RX vector bodies**

~~~c
void GPDMA1_Channel0_IRQHandler(void) { Board1_Lpuart1DmaIrq(); }
void GPDMA1_Channel4_IRQHandler(void) { Board1_Usart1DmaIrq(); }
void USART1_IRQHandler(void) { Board1_Usart1UartIrq(); }
void LPUART1_IRQHandler(void) { Board1_Lpuart1UartIrq(); }
~~~

Keep Channel2, Channel3, USART3 and UART4 handlers unchanged.

- [ ] **Step 2: Store exception context in the HardFault handler**

Add these variables and a 'static void hardfault_capture(uint32_t *stacked)' function. It copies SCB fault registers plus stack indexes 0 through 7, disables interrupts, and loops.

~~~c
volatile uint32_t fault_cfsr, fault_hfsr, fault_mmfar, fault_bfar;
volatile uint32_t fault_r0, fault_r1, fault_r2, fault_r3;
volatile uint32_t fault_r12, fault_lr, fault_pc, fault_xpsr;

__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile (
    "tst lr, #4 \n"
    "ite eq     \n"
    "mrseq r0, msp \n"
    "mrsne r0, psp \n"
    "b hardfault_capture \n");
}
~~~

- [ ] **Step 3: Verify and commit**

Run:

~~~powershell
& 'C:\Users\wang2\Desktop\test\OneLine\tools\test-board1-rx-architecture.ps1'
Set-Location 'C:\Users\wang2\Desktop\test\OneLine\OneLine1\MDK-ARM'
& 'C:\Users\wang2\AppData\Local\Keil_v5\UV4\UV4.exe' -b .\OneLine1.uvprojx -j0 -o .\board1_dma_idle_verify_build.log
git add -- OneLine1/Core/Src/stm32u5xx_it.c
git commit -m "fix(board1): use direct RX IRQs and capture faults"
~~~

Expected: architecture test exit code '0' and the Keil log ends '0 Error(s), 0 Warning(s).'

### Task 5: Perform final audit and target acceptance

**Files:**
- Verify: 'C:\Users\wang2\Desktop\test\OneLine\OneLine1\MDK-ARM\OneLine1\OneLine1.map'
- Verify: 'C:\Users\wang2\Desktop\test\OneLine\tools\test-board1-rx-architecture.ps1'

- [ ] **Step 1: Verify build, SRAM placement and absence of old RX code**

~~~powershell
& 'C:\Users\wang2\Desktop\test\OneLine\tools\test-board1-rx-architecture.ps1'
$map = Get-Content -Raw 'C:\Users\wang2\Desktop\test\OneLine\OneLine1\MDK-ARM\OneLine1\OneLine1.map'
if ($map -notmatch 'txq\s+0x2000[0-9a-fA-F]+\s+Data\s+524288') { throw 'txq is not 512 KB in SRAM' }
rg -n 'HAL_DMA_DeInit|HAL_UARTEx_ReceiveToIdle_DMA|HAL_UART_IRQHandler\(&hlpuart1\)|HAL_UART_IRQHandler\(&huart1\)' 'C:\Users\wang2\Desktop\test\OneLine\OneLine1\Core\Src\main.c' 'C:\Users\wang2\Desktop\test\OneLine\OneLine1\Core\Src\stm32u5xx_it.c'
if ($LASTEXITCODE -eq 0) { throw 'old RX path remains' }
~~~

Expected: the architecture script passes; map contains a 524288-byte 'txq'; the final 'rg' has no matches.

- [ ] **Step 2: Verify on hardware**

Flash 'OneLine1.hex'. Send '1231323213232' on COM8, then COM10; COM7 must receive each sequence exactly. Send different continuous sequences on both input ports at 115200 and verify each source's byte order, 'drop_cnt == 0', and 'err_cnt == 0'. On a fault, record 'fault_cfsr', 'fault_hfsr', 'fault_bfar', 'fault_pc', and 'fault_lr'.

- [ ] **Step 3: Commit the verified result**

~~~powershell
git add -- OneLine1/Core/Src/main.c OneLine1/Core/Src/usart.c OneLine1/Core/Src/stm32u5xx_it.c OneLine1/Core/Inc/main.h tools/test-board1-rx-architecture.ps1
git commit -m "fix(board1): stabilize circular DMA idle reception"
~~~

