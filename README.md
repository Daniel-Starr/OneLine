# NormalDMA 串口桥

> DMA 接收 + FIFO 缓冲 + 阻塞发送

`NormalDMA` 是 OneLine 双板串口桥的一个可验证基线版本。它将 DMA 用在接收端，将多个输入汇聚到软件 FIFO，再由主循环以带超时的阻塞方式发送，避免在中断上下文中启动或重启 DMA。

## 数据流

```text
PC 输入 A ──> 板 1 LPUART1 RX ─┐
                              ├─> 4 KiB 共享 FIFO ─> 板 1 USART3 TX ─> 板 2 USART3 RX
PC 输入 B ──> 板 1 USART1 RX ──┘                                           │
                                                                          FIFO
                                                                           │
                                                                           └─> 板 2 LPUART1 TX ─> PC 输出
```

## 关键实现

- 板 1 的 `LPUART1` 与 `USART1` 分别使用 `HAL_UARTEx_ReceiveToIdle_DMA()` 接收；板 2 的 `USART3` 使用相同方式接收。
- RX DMA 使用 **Normal（单次）模式**，每次接收最多 256 字节，并关闭 Half Transfer 中断。
- `HAL_UARTEx_RxEventCallback()` 与 `HAL_UART_ErrorCallback()` 仅复制数据、更新计数器并设置重启标志；不会在 ISR 中调用 HAL DMA。
- 主循环的 `bridge_rx_service()` 负责重启已完成或出错的 RX DMA。
- 主循环的 `tx_kick()` 每次从 FIFO 取出最多 256 字节，并调用 `HAL_UART_Transmit()` 发送；发送超时为 100 ms。
- FIFO 容量为 4096 字节。FIFO 满时新增字节会丢弃，并累加 `drop_cnt`；发送失败时不移动读指针，保留数据供下一轮重试。

## DMA 的使用范围

| 链路 | 接收 | 发送 |
| --- | --- | --- |
| 板 1：LPUART1 / USART1 | Receive-to-Idle + Normal DMA | 不适用 |
| 板 1：USART3 | 不适用 | 阻塞式 `HAL_UART_Transmit()` |
| 板 2：USART3 | Receive-to-Idle + Normal DMA | 不适用 |
| 板 2：LPUART1 | 不适用 | 阻塞式 `HAL_UART_Transmit()` |

因此，本版本不是“DMA 全双工转发”，而是 **DMA 接收 + FIFO 缓冲 + 阻塞发送**。

## 运行时计数器

板 1 可在调试器中观察：

- `rx_bytes`：LPUART1 接收字节数。
- `rx_bytes_u1`：USART1 接收字节数。
- `tx_bytes`：已从 FIFO 成功发送到板 2 的字节数。
- `drop_cnt`：FIFO 已满导致的丢弃字节数。
- `err_cnt`：RX DMA 启动或 RX 异常次数。
- `tx_err_cnt`：发送超时或发送失败次数。

板 2 也维护对应的接收、发送、丢弃与错误计数器。

## 已知限制

1. 正常 DMA 在一帧结束和主循环重启之间存在短暂接收窗口；极端连续流量下仍可能丢字节。
2. 两路输入若都以 115200 bit/s 持续发送，而板间 USART3 也为 115200 bit/s，输入总带宽高于输出带宽，FIFO 最终会满并增加 `drop_cnt`。本版本不承诺双路满速时无丢包。
3. 当前代码中实际初始化的 USART3 波特率为 115200，但 `.ioc` 文件记录为 921600；在重新生成 CubeMX 代码或发布前，必须先统一这两个配置。
4. `UART4` 已被初始化，但没有接入当前的转发数据流，不应将它视作第三路输入。
5. 启动阶段的首次 RX DMA 启动结果未触发自动重试；若启动失败，应通过 `err_cnt` 和调试器定位。

## 建议验证

在发布为稳定版前，至少完成以下测试并记录计数器：

1. 分别从两路输入发送短报文，确认都能到达板 2 的 PC 输出。
2. 两路交替发送与并发发送，记录 `drop_cnt` 是否增长。
3. 持续运行 30 分钟以上，确认没有进入 HardFault，且 `err_cnt`、`tx_err_cnt` 的变化可解释。
4. 人为制造 RX 断开、悬空与溢出，确认主循环能重新启动接收。

## 版本定位

建议版本标识：`v0.1.0-beta.1`
建议展示名称：**OneLine 双板双路串口汇聚桥**

该版本适合作为 DMA 接收、FIFO 汇聚与可靠性验证的基线，而不是双路满速无损传输的最终发布版本。
