# OneLine

**OneLine 桥接 · 纯寄存器轮询方案（没有 DMA）**

一个透明、单向的双板 UART 桥：PC 的串口数据经过两块 STM32U575VGTx（Cortex‑M33）板子中转后**原样**回到 PC，全程**逐字节透传**——不加帧头、不加 CRC、不加端口号、不用 printf。

## 数据通路

```
PC ──▶ 板1 LPUART1_RX (PC0)  ┐
                             ├─ 合并 ─▶ 板1 USART3_TX (PA7)
PC ──▶ 板1 USART1_RX (PA10) ┘                  │
                                               ▼ (PA7 → PA5)
PC ◀── 板2 LPUART1_TX (PC1) ◀── 板2 USART3_RX (PA5)
```

- **板1（二合一）**：两路输入 `LPUART1_RX` + `USART1_RX` 合并 → 一路输出 `USART3_TX`
- **板2（中继）**：单路 `USART3_RX` → `LPUART1_TX` 吐回 PC
- 全链路 115200 / 8N1

## 方案：纯寄存器轮询

整条数据通路**不用 DMA、不用中断、不用 HAL 的 UART 运行时、不用句柄/回调**。只有一个软件环形 FIFO + 主循环轮询寄存器：

```c
while (1) {
    rx_poll();   // RXNE 置位?      -> 读 RDR -> 入 4096B FIFO
    tx_kick();   // TXE 置位且非空? -> 取一字节 -> 写 TDR
}
```

- `rx_poll()`：轮询每个 RX 串口的 `RXNE`，直接从 `RDR` 读字节塞进 FIFO（顺带清 `ORE` 防卡）
- `tx_kick()`：轮询 TX 串口的 `TXE`，从 FIFO 取一个字节写 `TDR`
- 收发都在主循环里跑，没有中断碰 FIFO → **无竞态、无临界区、无锁**

## 为什么是纯轮询（不用 DMA）

DMA 在这颗 U5 上反复把内存写坏、导致 **HardFault**，依次踩过三个坑：

| # | 故障 | 现象 |
|---|------|------|
| 1 | 环形链表 DMA 越界 | CubeMX 的 circular 节点没绑定目的地址，回绕时写花 RAM（`CFSR=0x8200`） |
| 2 | DMA‑TX 空句柄 | `HAL_UART_Transmit_DMA` 解引用被写坏的 `hdmatx`（`BFAR=0x30`） |
| 3 | DMA‑RX 坏返回地址 | `UART_Start_Receive_DMA` 路径里返回地址 Thumb 位被清、跳飞（`CFSR=0x00010000` UNDEFINSTR） |

纯轮询的数据通路里**没有任何可被写坏的指针**，所以这一整类故障在结构上消失了。160 MHz 的核在 115200 下每个字节有 87 µs，主循环每字节能轮询上千次 → 单路不丢字节。（板1 两路同时满速时输入 > 输出，FIFO 会溢出丢字节并计入 `drop_cnt`，属预期行为。）

## 工程结构

| 路径 | 说明 |
|------|------|
| `OneLine1/` | 板1（二合一）Keil / CubeMX 工程 |
| `OneLine2/` | 板2（中继）Keil / CubeMX 工程 |
| `Core/Src/main.c` | 应用逻辑（全部在各自的 USER CODE 区内） |

- 用 Keil μVision 编译，两个工程均 **0 Error / 0 Warning**
- DMA 通道虽仍由 CubeMX 配置但**未使用**（数据通路已全部走寄存器轮询）
