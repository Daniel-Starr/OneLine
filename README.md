# OneLine 双板串口 DMA 直通桥

`master` 分支实现了一个基于 STM32U575 的双板、单向串口透明转发链路。它使用 DMA 接收和 DMA 发送，收到的数据块直接转发，不使用软件 FIFO。

当前代码基线：`ac2a4cd`（降低无 FIFO 转发首包延迟）。

## 数据流

```text
PC 输入
  │
  └─> 板 1：LPUART1 RX
          │  Receive-to-Idle + 循环 RX DMA（256 B）
          └─> USART3 TX
                   │  TX DMA（最多 256 B）
                   └─> 板 2：USART3 RX
                           │  Receive-to-Idle + 循环 RX DMA（256 B）
                           └─> LPUART1 TX
                                    │  TX DMA（最多 256 B）
                                    └─> PC 输出
```

板 1 和板 2 的转发逻辑相同，只是 RX/TX 串口角色相反：

| 板卡 | 接收端 | 发送端 |
| --- | --- | --- |
| 板 1 | LPUART1 | USART3 |
| 板 2 | USART3 | LPUART1 |

所有已使用串口均配置为 **115200 bit/s、8 数据位、1 停止位、无校验、无硬件流控**。

## 转发方式

1. `HAL_UARTEx_ReceiveToIdle_DMA()` 启动 256 字节循环 RX DMA。
2. HAL 在空闲、半传输完成或完整传输完成时调用 `HAL_UARTEx_RxEventCallback()`。
3. 回调记录循环 DMA 的当前写入位置；`bridge_poll_rx()` 找出尚未转发的连续数据块。
4. 数据先复制到独立的 `tx_dma_buf`，再通过 `HAL_UART_Transmit_DMA()` 发往下一跳。
5. TX DMA 完成回调释放 `tx_busy`，并立即继续检查是否已有新的 RX 数据。

该方案的特点是路径短、首包延迟低：数据无需先进入软件队列，CPU 只负责位置管理与把当前块复制到 TX DMA 缓冲区。

## DMA 配置

| 位置 | 方向 | 模式 | 缓冲区 |
| --- | --- | --- | --- |
| 板 1 LPUART1 | RX | GPDMA 链表循环 | 256 B |
| 板 1 USART3 | TX | Normal DMA | 最多 256 B/次 |
| 板 2 USART3 | RX | GPDMA 链表循环 | 256 B |
| 板 2 LPUART1 | TX | Normal DMA | 最多 256 B/次 |

STM32U575 使用 Cortex-M33；本工程的缓冲区可由 DMA 直接访问，当前实现不需要额外的 D-Cache 维护。

## 调试计数器

每块板的 `main.c` 都提供以下 `volatile` 计数器，适合在 Keil Watch 窗口观察：

- `rx_bytes`：从 RX DMA 取出的字节数。
- `tx_bytes`：TX DMA 已成功完成的字节数。
- `err_cnt`：接收或发送错误次数。
- `zero_burst_cnt` / `zero_drop_bytes`：被判定为全 `0x00` 的噪声块及其字节数。
- `rx_event_idle_cnt` / `rx_event_ht_cnt` / `rx_event_tc_cnt`：Receive-to-Idle 的事件分布。
- `tx_busy`：当前是否仍有一笔 TX DMA 在执行。

## 已知边界

- 本分支 **没有软件 FIFO**。`tx_busy` 为真时，转发函数会先返回；如果接收端在发送端追上前绕过 256 字节循环缓冲区，早期数据可能被覆盖。
- `drop_cnt` 虽已声明，但当前实现没有在上述无缓冲覆盖场景中递增；因此它不能用于证明链路无丢包。
- 接收 DMA 使用循环链表，错误回调会中止并重新启动接收 DMA。应通过实际硬件长时间运行验证异常恢复与循环回绕稳定性。
- 无协议帧、CRC、ACK/重传或 RTS/CTS 流控；它是透明字节转发，不是可靠传输协议。
- `DROP_ALL_ZERO_RX_CHUNKS` 默认开启，用于抑制 RX 悬空或被拉低时的 `0x00` 噪声；若业务允许真实的全零报文，应重新评估该开关。

## 建议验证

1. 使用短报文确认 PC 输入能够稳定到达板 2 的 PC 输出。
2. 测试 1、128、256 字节及跨 256 字节边界的报文，核对接收总字节数与 `tx_bytes`。
3. 进行持续发送和长时间运行，监控 `err_cnt`、事件计数及是否发生 HardFault。
4. 故意制造 RX 断开、悬空与过载，观察全零过滤、DMA 重启和数据完整性。

## 与 `NormalDMA` 分支的区别

| 分支 | 输入结构 | 接收 | 缓冲 | 发送 |
| --- | --- | --- | --- | --- |
| `master` | 单路输入 | 循环 DMA | 无软件 FIFO | TX DMA 直发 |
| `NormalDMA` | 板 1 双路输入汇聚 | 单次 Normal DMA | 4 KiB 软件 FIFO | 带超时的阻塞发送 |

`master` 适合作为低延迟的单路 DMA 直通基线；`NormalDMA` 则侧重多输入汇聚和更显式的背压/丢包观测。两者的可靠性和性能应分别在目标硬件上测试后再用于正式发布。
