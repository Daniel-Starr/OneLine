$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$main = Get-Content -Raw (Join-Path $root 'OneLine1/Core/Src/main.c')
$irq = Get-Content -Raw (Join-Path $root 'OneLine1/Core/Src/stm32u5xx_it.c')
$header = Get-Content -Raw (Join-Path $root 'OneLine1/Core/Inc/main.h')
$usart = Get-Content -Raw (Join-Path $root 'OneLine1/Core/Src/usart.c')
$failures = [System.Collections.Generic.List[string]]::new()

foreach ($pattern in @(
  'Board1_Lpuart1DmaIrq',
  'Board1_Usart1DmaIrq',
  'Board1_Lpuart1UartIrq',
  'Board1_Usart1UartIrq',
  'fault_cfsr',
  'fault_pc'))
{
  if (($main -notmatch $pattern) -and ($irq -notmatch $pattern) -and ($header -notmatch $pattern))
  {
    $failures.Add("missing: $pattern")
  }
}

foreach ($pattern in @(
  'HAL_DMA_DeInit\(',
  'HAL_UARTEx_ReceiveToIdle_DMA\(',
  'HAL_UART_IRQHandler\(&hlpuart1\)',
  'HAL_UART_IRQHandler\(&huart1\)'))
{
  if (($main -match $pattern) -or ($irq -match $pattern))
  {
    $failures.Add("forbidden RX path: $pattern")
  }
}

foreach ($pattern in @(
  'HAL_DMAEx_List_BuildNode\(&NodeConfig, &Node_GPDMA1_Channel0\)',
  'HAL_DMAEx_List_SetCircularMode\(&List_GPDMA1_Channel0\)',
  'HAL_DMAEx_List_BuildNode\(&NodeConfig, &Node_GPDMA1_Channel4\)',
  'HAL_DMAEx_List_SetCircularMode\(&List_GPDMA1_Channel4\)'))
{
  if ($usart -notmatch $pattern)
  {
    $failures.Add("missing circular configuration: $pattern")
  }
}

if ($failures.Count -ne 0)
{
  $failures | ForEach-Object { Write-Error $_ }
  exit 1
}

Write-Host 'PASS: board1 RX uses direct HT/TC/IDLE IRQ architecture.'
