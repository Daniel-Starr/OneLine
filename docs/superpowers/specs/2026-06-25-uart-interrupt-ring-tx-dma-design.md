# OneLine 双板无损串口桥设计

日期：2026-06-25

## 1. 目标

将现有双板串口桥改造成以下结构：

- 接收端使用持续开启的 UART RX 中断和软件环形缓冲。
- 发送端使用非阻塞 TX DMA。
- COM8 和 COM10 分别以 115200 bit/s 持续满速输入。
- 板间 USART3 和板2到 COM7 的 LPUART1 均使用 921600 bit/s。
- 正常连接、PC 正常读取的条件下，双路连续满速传输不丢字节、不重复、不乱序。
- 不再使用 RX DMA，不再反复启动、停止或 Abort RX DMA。
- CubeMX `.ioc`、生成代码和 Keil 工程配置保持一致。

输入工具是普通串口调试助手，不支持 RTS/CTS，也没有 ACK/重传。因此本设计不能承诺 COM7 断开、PC 长期不读取或硬件故障期间仍然无损；这些情况必须被计数和暴露，不能静默丢包。

## 2. 带宽预算

串口采用 8N1，每个有效字节占 10 bit：

- 单路 115200 输入：11520 byte/s。
- 双路合计输入：23040 byte/s。
- 921600 输出：92160 byte/s。

高速链路具有约 4 倍有效带宽余量。只要发送端和 PC 正常工作，FIFO 水位应在短暂波动后回落，而不是持续上升。

## 3. 总体架构

```text
COM8  --115200--> 板1 USART1 RX IRQ  --\
                                        +--> 公平合并 --> 板1 TX Ring
COM10 --115200--> 板1 LPUART1 RX IRQ --/                 |
                                                           v
                                              USART3 TX DMA 921600
                                                           |
                                                           v
                                              USART3 RX IRQ 921600
                                                           |
                                                           v
                                                  板2 Output Ring
                                                           |
                                                           v
                                              LPUART1 TX DMA 921600
                                                           |
                                                           v
                                                          COM7
```

所有 ISR 只做有界、短时工作：

- RX ISR 读取硬件 RDR/FIFO并写入软件环形缓冲。
- DMA 完成 ISR 只提交已发送长度、释放发送状态并设置继续发送标志。
- ISR 内不执行阻塞发送，不复制大块数据，不启动或停止 RX DMA。

## 4. 软件组件

### 4.1 软件环形缓冲

统一实现单生产者、单消费者的 `uart_ring_t`：

- 缓冲区大小使用 2 的幂，索引通过掩码回绕。
- `head` 只由生产者更新，`tail` 只由消费者更新。
- 满时不覆盖未消费数据；增加 `overflow_count` 并保持系统继续运行。
- 记录当前水位和历史最高水位 `high_watermark`。
- 批量读写在环回边界拆成最多两段。
- 不在逐字节处理期间长时间关闭全局中断。

建议容量：

| 缓冲 | 容量 | 生产者 | 消费者 |
|---|---:|---|---|
| 板1 USART1 RX Ring | 4096 byte | USART1 ISR | 公平合并器 |
| 板1 LPUART1 RX Ring | 4096 byte | LPUART1 ISR | 公平合并器 |
| 板1 USART3 TX Ring | 8192 byte | 公平合并器 | TX DMA 引擎 |
| 板2 Output Ring | 8192 byte | USART3 ISR | TX DMA 引擎 |

### 4.2 RX 中断接收器

输入 UART 不调用 `HAL_UART_Receive_IT()`，也不维护一次性接收请求。初始化后直接启用接收数据和错误中断：

- UART FIFO 开启，RX 阈值设为最低档；若目标外设不支持该模式，则使用 RXNE。
- ISR 在一次进入中排空当前可读的 RDR/FIFO。
- 每个字节立即压入对应软件环形缓冲。
- 对 ORE、FE、NE、PE 分别计数并按 STM32U5 HAL/CMSIS 定义清除标志。
- 清错后接收中断保持开启，不执行 DMA Abort 或重新接收。

自定义 RX IRQ 处理器独占板1 USART1、板1 LPUART1 和板2 USART3 的接收路径，避免与 HAL 的一次性接收状态机重复读取 RDR。发送 UART 的 DMA和 UART TC 处理继续使用 HAL。

### 4.3 板1公平合并器

板1保留两个独立输入 Ring，不直接让两个 ISR 写同一个共享 FIFO。

主循环公平合并规则：

1. 记住下一次优先检查的输入源。
2. 每轮从当前源最多取 64 byte，写入板1 USART3 TX Ring。
3. 随后切换到另一源。
4. 如果另一源为空，当前有数据的源可继续使用全部可用带宽。
5. 当 TX Ring 剩余空间不足时停止搬运，等待 DMA消费。

该设计保证每一路内部字节顺序不变，并限制一路持续满载时对另一条链路的阻塞长度。两路之间不存在可恢复的绝对时间顺序，COM7 看到的是最多 64 byte 为一个调度片的透明合并字节流。

### 4.4 TX DMA 引擎

板1 USART3 和板2 LPUART1 共用同一种发送状态机：

- 仅在 `tx_busy == false` 且 Ring 非空时启动 `HAL_UART_Transmit_DMA()`。
- 每次发送 Ring 尾部的一段连续内存，长度不跨越环回边界。
- 建议单次最大发送长度为 256 byte；板1公平性已经由上游 64 byte 调度片保证。
- 启动成功后记录 `inflight_length`，但暂不移动 Ring 的 `tail`。
- TX 完成回调中移动 `tail`、增加 `tx_bytes`、清除 `tx_busy` 并设置 `tx_kick_pending`。
- 主循环看到 `tx_kick_pending` 或空闲状态后启动下一段，回调内不递归启动下一次 DMA。
- DMA启动返回 `HAL_BUSY/HAL_ERROR` 时数据不出队，由主循环稍后重试。

若 DMA 运行途中报错，主循环负责恢复 HAL TX 状态并重试未提交的数据。由于普通 UART 没有端到端确认，异常发生在部分字节已经上线路时，重试可能造成重复；因此正常无损验收要求 `tx_dma_error_count == 0`。

## 5. 板卡配置

### 5.1 板1

| 外设 | 方向 | 波特率 | 接收/发送方式 |
|---|---|---:|---|
| USART1 | RX | 115200 | RX IRQ + 4096 byte Ring |
| LPUART1 | RX | 115200 | RX IRQ + 4096 byte Ring |
| USART3 | TX | 921600 | Normal TX DMA |

- 删除 USART1 和 LPUART1 的 RX DMA 请求、DMA句柄和 GPDMA IRQ。
- UART4 不属于当前数据通路，移除其主动初始化和 RX DMA 配置，避免无用 DMA状态影响调试。
- USART3 保留 TX DMA，使用现有 GPDMA1 Channel2 或由 CubeMX 重新分配后保持 `.ioc` 与源码一致。

### 5.2 板2

| 外设 | 方向 | 波特率 | 接收/发送方式 |
|---|---|---:|---|
| USART3 | RX | 921600 | RX IRQ + 8192 byte Output Ring |
| LPUART1 | TX | 921600 | Normal TX DMA |

- 删除 USART3 RX DMA 请求、DMA句柄和对应 GPDMA IRQ。
- LPUART1 保留 TX DMA，使用现有 GPDMA1 Channel3 或由 CubeMX 重新分配后保持一致。

### 5.3 CubeMX 一致性

`.ioc` 是外设和 DMA配置的来源，必须同步修改：

- RX UART 不配置 DMA。
- 高速链路明确配置为 921600。
- TX DMA配置为 Normal、Memory-to-Peripheral、源地址递增、目的地址固定、字节宽度。
- 中断优先级必须允许 RX 在 TX DMA期间及时运行。
- 重新生成代码后，不能恢复 Circular Linked-List RX DMA或 115200 的板间链路。

应用逻辑放在用户代码文件或 CubeMX `USER CODE` 区域，不修改 STM32 HAL 驱动源码。

## 6. 并发与中断规则

- RX ISR 是各自 RX Ring 的唯一生产者。
- 板1主循环是两个 RX Ring 的唯一消费者，也是板1 TX Ring 的唯一生产者。
- TX DMA完成回调是对应 TX Ring 的唯一提交者。
- 共享的 `tx_busy`、`inflight_length` 和 kick 标志只使用极短临界区或单次原子访问保护。
- RX 中断优先级高于或等于 DMA发送完成中断；ISR 中禁止等待另一个中断完成。
- 所有计数器仅用于观测，不参与正确性判断时可使用 `volatile uint32_t`。

## 7. 错误处理与可观测性

每块板至少提供以下观测量：

- 每个输入口的 `rx_bytes`。
- 每个 Ring 的当前水位和 `high_watermark`。
- 每个 Ring 的 `overflow_count`。
- `tx_started_bytes` 和 `tx_completed_bytes`。
- `tx_dma_start_error_count`、`tx_dma_error_count`。
- UART `ore_count`、`fe_count`、`ne_count`、`pe_count`。
- 当前 `tx_busy` 和 `inflight_length`。

处理策略：

- RX错误：分类计数、清标志、排空可读数据、继续接收。
- Ring 满：记录溢出，不覆盖旧数据；正常无损验收直接失败。
- TX DMA启动失败：保留数据并在主循环重试。
- TX DMA运行错误：退出 busy 状态，在主循环恢复并重试；记录错误供验收判定。
- 初始化失败：进入明确的错误状态，不假装继续运行。

## 8. 验证方案

### 8.1 构建和配置验证

- OneLine1、OneLine2 两套 Keil 工程均须 0 Error、0 Warning。
- 检查生成后的 C 配置与 `.ioc` 一致。
- 搜索确认应用代码不再调用 RX DMA启动、RX DMA Abort 或 Receive-to-Idle RX API。
- 确认 HAL 驱动目录没有项目专用补丁。

### 8.2 单路压力测试

- COM8 连续 115200 满速发送至少 10 分钟。
- COM10 连续 115200 满速发送至少 10 分钟。
- 分别核对发送文件、COM7接收文件和各级字节计数。
- 要求无缺字节、无重复、顺序一致，所有 overflow 和 UART错误计数为 0。

### 8.3 双路压力测试

- COM8 发送 `0x00..0x7F` 循环序列。
- COM10 发送 `0x80..0xFF` 循环序列。
- 两路同时以 115200 满速发送至少 30 分钟。
- COM7检查程序通过最高位区分来源，再分别检查低 7 bit 序列是否连续。
- 两路内部顺序必须保持，允许两路数据按最多 64 byte 调度片交织。
- 板1接收总数、板1发送完成数、板2接收数、板2发送完成数和 COM7接收总数必须一致。
- 所有 `overflow_count`、`tx_dma_error_count` 和 ORE计数必须为 0。
- Ring 的 `high_watermark` 必须稳定，不能随时间持续上升。

### 8.4 边界测试

- 1 byte 短发送和频繁启停。
- 长文件连续发送。
- 两路交替突发。
- 一路持续满载，另一路间歇插入。
- 接近环形缓冲回绕点的数据完整性。
- 暂停 COM7读取后恢复：系统应记录水位和溢出并继续运行，但不承诺暂停期间无损。

## 9. 验收条件

设计实现只有同时满足以下条件才算通过：

1. 两路 115200 连续满速 30 分钟无缺失、无重复、各自顺序正确。
2. COM7 总接收字节数与板1两路总输入字节数一致。
3. 所有软件 Ring 的 `overflow_count == 0`。
4. 正常压力测试期间 `tx_dma_error_count == 0`、ORE计数为 0。
5. 两套固件构建均为 0 Error、0 Warning。
6. `.ioc`、生成源码和实际波特率/DMA模式一致。
7. 不再复现 RX DMA重启、Abort 或 Circular Linked-List 野写相关 HardFault。

## 10. 非目标

- 不增加源标识、帧头、CRC或任何业务协议；桥接仍是透明字节流。
- 不保证 COM7断线、PC长期不读取或物理链路故障期间无损。
- 不恢复两路输入之间的绝对时间顺序。
- 不使用 RTS/CTS，也不开发 ACK/重传协议。
