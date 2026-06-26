# 板 1 双路 UART 循环 DMA 接收设计

## 目标

将板 1 的 LPUART1_RX 与 USART1_RX 改为稳定的双路连续接收：GPDMA 循环链表的半传输/全传输中断与 UART 空闲中断共同发现数据，新增字节写入一个共享环形队列，再由主循环发送到 USART3_TX。

## 已知问题与约束

- 当前代码在 `main()` 中将已按普通 DMA 初始化的 Channel 0 和 Channel 4 先 `HAL_DMA_DeInit()`，再重建为循环链表 DMA；板 1 在这一启动路径进入 HardFault，且返回状态未被记录。
- 当前链接映像的 512 KB 队列、两个 1024-byte DMA 缓冲和 8 KB 栈均位于 768 KB SRAM 范围内且不重叠；内存容量不是本次启动期故障的直接证据。
- STM32U5 的 GPDMA 循环传输必须使用自环链表节点；每个节点必须固定写回本路 DMA 缓冲。
- 不允许在 RX 中断中 Abort、DeInit、重启 DMA，或调用 HAL UART 接收状态机。
- 保持现有端口、115200 波特率、透明字节转发和共享 512 KB 输出队列。

## 架构

每个输入 UART 独占一条 GPDMA 通道、一块 1024-byte DMA 环形缓冲和一个 36-byte自环节点：

| 输入 | GPDMA 通道 | 事件 | DMA 缓冲 |
| --- | --- | --- | --- |
| LPUART1_RX | GPDMA1 Channel 0 | HT、TC、IDLE | `dma_rx_lp` |
| USART1_RX | GPDMA1 Channel 4 | HT、TC、IDLE | `dma_rx_u1` |

初始化阶段直接配置每条通道为循环链表，并固定节点的源地址（UART RDR）、目的地址（对应 DMA 缓冲）和长度（1024）。随后直接使能 DMA HT/TC/错误中断、UART DMAR 与 UART IDLEIE；此后不再运行 HAL DMA 或 HAL UART 接收 API。

DMA IRQ 和 UART IRQ 都调用同一个按通道参数化的 `rx_drain()`：读取 DMA 当前剩余传输量取得写位置，比较本路上一次位置，处理环绕区间，将新字节压入共享队列。相同位置表示是重复事件，不复制数据。

主循环从共享队列读取字节，并在 USART3 TXE 为 1 时直接写入 TDR。队列满时仅累加丢弃计数，正常路径不得阻塞。

## 中断与并发规则

- 两路 DMA 与 UART IRQ 使用相同优先级，单核上不会并发执行；它们只修改各自的 DMA 读位置和共享队列生产端。
- 主循环是队列唯一消费者。读写共享的队列计数、头尾索引时使用极短临界区，避免 ISR 与主循环竞争。
- DMA HT、TC 和 UART IDLE 都只调用 `rx_drain()`；不在 IRQ 中发送数据、等待标志或重新配置硬件。
- UART ORE/FE/NE/PE 需要清除相应 ICR 标志并递增错误计数，DMA 保持运行。

## 可观测性与故障捕获

保留 `rx_bytes`、`rx_bytes_u1`、`tx_bytes`、`drop_cnt` 和 `err_cnt`，并新增：

- 每路 DMA 的 HT、TC、IDLE、DMA 错误计数。
- DMA 启动阶段与每步返回/寄存器快照。
- HardFault 的 CFSR、HFSR、BFAR、MMFAR、异常栈寄存器以及故障 PC/LR。

发生 HardFault 时停在处理器中，Watch 可直接读取以上变量，能确定精确故障指令和类型。

## 验证

1. Keil 构建板 1，要求 0 Error、0 Warning；检查 `.map` 中 SRAM 用量和栈不重叠。
2. 静态检查：板 1 RX 路径不得调用 `HAL_DMA_DeInit`、`HAL_DMA_Abort`、`HAL_UARTEx_ReceiveToIdle_DMA` 或 `HAL_UART_IRQHandler`。
3. 上电空闲时确认没有 HardFault，启动阶段码达到“两个通道已使能”。
4. 分别从 COM8 与 COM10 发送 1-byte、短报文和连续数据，确认 COM7 收到完整字节流，且 HT/TC/IDLE 计数符合数据形态。
5. 两路同时 115200 连续发送，分别以可区分序列验证每路内部顺序；正常测试中 `drop_cnt` 与错误计数应为 0。
6. 如再进入 HardFault，读取保存的 CFSR/HFSR/BFAR/PC 后再修改，不依据 Handler 停点猜测。
