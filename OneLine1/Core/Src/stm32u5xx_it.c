/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32u5xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32u5xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FAULT_MAGIC           (0xFA017ECCuL)   
#define FAULT_VERSION         (2uL)            
#define FAULT_TYPE_HARDFAULT  (3uL)            
#define CFSR_MUNSTKERR        (1uL << 3)        
#define CFSR_MSTKERR          (1uL << 4)        
#define CFSR_UNSTKERR         (1uL << 11)       
#define CFSR_STKERR           (1uL << 12)       
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint32_t fault_cfsr;
volatile uint32_t fault_hfsr;
volatile uint32_t fault_mmfar;
volatile uint32_t fault_bfar;
volatile uint32_t fault_r0;
volatile uint32_t fault_r1;
volatile uint32_t fault_r2;
volatile uint32_t fault_r3;
volatile uint32_t fault_r12;
volatile uint32_t fault_lr;
volatile uint32_t fault_pc;
volatile uint32_t fault_xpsr;

typedef struct
{
  uint32_t magic;          
  uint32_t version;        
  uint32_t fault_type;       
  uint32_t capture_complete; 
  uint32_t frame_valid;    
  uint32_t fpu_extended;   
  uint32_t raw_sp;         
  uint32_t exc_return;     
  uint32_t msp;            
  uint32_t psp;            
  uint32_t control;        
  uint32_t primask;        
  uint32_t basepri;        
  uint32_t faultmask;      
  uint32_t cfsr;           
  uint32_t hfsr;           
  uint32_t shcsr;          
  uint32_t icsr;           
  uint32_t mmfar;          
  uint32_t bfar;           
  uint32_t afsr;           
  uint32_t r0;             
  uint32_t r1;             
  uint32_t r2;             
  uint32_t r3;             
  uint32_t r12;            
  uint32_t lr;             
  uint32_t pc;             
  uint32_t xpsr;           
  uint32_t dma_csr;        
  uint32_t dma_cbr1;       
  uint32_t dma_csar;       
  uint32_t dma_cdar;       
  uint32_t dma_cllr;       
  uint32_t dma_all_csar[16]; 
  uint32_t dma_all_cdar[16]; 
  uint32_t dma_all_cbr1[16]; 
  uint32_t raw_stack[32];  
} FaultRecord;

volatile FaultRecord g_fault;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
__attribute__((used, noinline, optnone))
void hardfault_capture(uint32_t *stacked, uint32_t exc_return)
{
  uint32_t primask_at_entry   = __get_PRIMASK();
  uint32_t basepri_at_entry   = __get_BASEPRI();
  uint32_t faultmask_at_entry = __get_FAULTMASK();
  uint32_t control_at_entry   = __get_CONTROL();
  uint32_t i;

  __disable_irq();

  g_fault.magic      = FAULT_MAGIC;
  g_fault.version    = FAULT_VERSION;
  g_fault.fault_type = FAULT_TYPE_HARDFAULT;
  g_fault.capture_complete = 0u;          
  g_fault.raw_sp     = (uint32_t)stacked;
  g_fault.exc_return = exc_return;

  g_fault.msp        = __get_MSP();
  g_fault.psp        = __get_PSP();
  g_fault.control    = control_at_entry;
  g_fault.primask    = primask_at_entry;
  g_fault.basepri    = basepri_at_entry;
  g_fault.faultmask  = faultmask_at_entry;

  g_fault.cfsr  = SCB->CFSR;
  g_fault.hfsr  = SCB->HFSR;
  g_fault.shcsr = SCB->SHCSR;
  g_fault.icsr  = SCB->ICSR;
  g_fault.mmfar = SCB->MMFAR;
  g_fault.bfar  = SCB->BFAR;
  g_fault.afsr  = SCB->AFSR;

  for (i = 0u; i < 32u; i++)
  {
    g_fault.raw_stack[i] = stacked[i];
  }

  g_fault.fpu_extended = ((exc_return & (1uL << 4)) == 0uL) ? 1u : 0u;
  g_fault.r0   = g_fault.raw_stack[0];
  g_fault.r1   = g_fault.raw_stack[1];
  g_fault.r2   = g_fault.raw_stack[2];
  g_fault.r3   = g_fault.raw_stack[3];
  g_fault.r12  = g_fault.raw_stack[4];
  g_fault.lr   = g_fault.raw_stack[5];
  g_fault.pc   = g_fault.raw_stack[6];
  g_fault.xpsr = g_fault.raw_stack[7];

  g_fault.frame_valid =
      ((g_fault.cfsr & (CFSR_MUNSTKERR | CFSR_MSTKERR | CFSR_UNSTKERR | CFSR_STKERR)) != 0uL)
      ? 0u : 1u;

  g_fault.dma_csr  = GPDMA1_Channel4->CSR;
  g_fault.dma_cbr1 = GPDMA1_Channel4->CBR1;
  g_fault.dma_csar = GPDMA1_Channel4->CSAR;
  g_fault.dma_cdar = GPDMA1_Channel4->CDAR;
  g_fault.dma_cllr = GPDMA1_Channel4->CLLR;

  for (i = 0u; i < 16u; i++)
  {
    DMA_Channel_TypeDef *ch = (DMA_Channel_TypeDef *)((uint32_t)GPDMA1_Channel0 + (i * 0x80u));
    g_fault.dma_all_csar[i] = ch->CSAR;
    g_fault.dma_all_cdar[i] = ch->CDAR;
    g_fault.dma_all_cbr1[i] = ch->CBR1;
  }

  fault_cfsr  = g_fault.cfsr;
  fault_hfsr  = g_fault.hfsr;
  fault_mmfar = g_fault.mmfar;
  fault_bfar  = g_fault.bfar;
  fault_r0    = g_fault.r0;
  fault_r1    = g_fault.r1;
  fault_r2    = g_fault.r2;
  fault_r3    = g_fault.r3;
  fault_r12   = g_fault.r12;
  fault_lr    = g_fault.lr;
  fault_pc    = g_fault.pc;
  fault_xpsr  = g_fault.xpsr;

  g_fault.capture_complete = 1u;

  __DSB();
  __ISB();

  for (;;)
  {
  }
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_NodeTypeDef Node_GPDMA1_Channel0;
extern DMA_QListTypeDef List_GPDMA1_Channel0;
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
extern DMA_NodeTypeDef Node_GPDMA1_Channel4;
extern DMA_QListTypeDef List_GPDMA1_Channel4;
extern DMA_HandleTypeDef handle_GPDMA1_Channel4;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
/* 【手改·CubeMX regen 会冲掉,需重贴】naked 蹦床:按 EXC_RETURN bit2 选 MSP/PSP 放 r0、
   EXC_RETURN 放 r1,跳到 hardfault_capture 抓现场。regen 会把它还原成空 while(1)。 */
__attribute__((naked)) void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  __asm volatile (
    "tst   lr, #4            \n"
    "ite   eq                \n"
    "mrseq r0, msp           \n"
    "mrsne r0, psp           \n"
    "mov   r1, lr            \n"
    "b     hardfault_capture \n");
  /* USER CODE END HardFault_IRQn 0 */
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32U5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32u5xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles GPDMA1 Channel 0 global interrupt.
  */
void GPDMA1_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 0 */

  /* USER CODE END GPDMA1_Channel0_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel0);
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 1 */

  /* USER CODE END GPDMA1_Channel0_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 2 global interrupt.
  */
void GPDMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 0 */

  /* USER CODE END GPDMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel2);
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 1 */

  /* USER CODE END GPDMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 3 global interrupt.
  */
void GPDMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 0 */
  /* UART4 已删,Ch3 不再使用,handle_GPDMA1_Channel3 已不存在 -> 空实现(并已在 gpdma.c 关掉其 NVIC)。
     根治:在 CubeMX 里删掉 Ch3 的 GPDMA 请求,否则 regen 会再次生成对未定义句柄的引用。 */
  /* USER CODE END GPDMA1_Channel3_IRQn 0 */
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 1 */

  /* USER CODE END GPDMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 4 global interrupt.
  */
void GPDMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 0 */

  /* USER CODE END GPDMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel4);
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 1 */

  /* USER CODE END GPDMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles LPUART1 global interrupt.
  */
void LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN LPUART1_IRQn 0 */

  /* USER CODE END LPUART1_IRQn 0 */
  HAL_UART_IRQHandler(&hlpuart1);
  /* USER CODE BEGIN LPUART1_IRQn 1 */

  /* USER CODE END LPUART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
