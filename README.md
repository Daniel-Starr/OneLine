# OneLine 双板串口分帧桥

本分支面向 STM32U575VGTx，实现两路 PC 串口输入经两块开发板汇聚后输出到 PC。当前代码不是透明透传：板 1 为每个接收块添加来源、长度和 CRC，板 2 校验合法性后将完整二进制帧发送给 PC。

## 数据通路

```text
COM10 -> 板1 LPUART1 RX 115200 --\
                                   +-> 二进制分帧 -> USART3 TX 921600
COM8  -> 板1 USART1 RX  115200 --/                         |
                                                              v
                                      板2 USART3 RX 921600 -> CRC 校验
                                                              |
                                                              v
                                      板2 LPUART1 TX 921600 -> COM7
```

| 板卡 | 接收 | 处理 | 发送 |
| --- | --- | --- | --- |
| 板 1 | LPUART1、USART1，各自 1024 B 循环 DMA | 按 RX 事件取出新增数据，封装帧并写入 512 KiB 环形队列 | 主循环轮询 USART3 TXE/TXFNF，直接写 TDR |
| 板 2 | USART3，1024 B 循环 DMA | 跨 DMA 回调解析帧，校验 SRC、LEN 和 CRC，合法帧整体写入 512 KiB 环形队列 | 主循环轮询 LPUART1 TXE/TXFNF，直接写 TDR |

RX 使用 `HAL_UARTEx_ReceiveToIdle_DMA()`，通过半传输、全传输和空闲事件持续搬运数据。UART 出错后，回调只设置重启标志，主循环限速重建对应 DMA。

## 帧格式

```text
[0xAA][SRC][LEN_H][LEN_L][PAYLOAD...][CRC8]
```

- `SRC = 0x01`：板 1 LPUART1（COM10）。
- `SRC = 0x02`：板 1 USART1（COM8）。
- `LEN`：大端序，范围 1..1024，只表示 `PAYLOAD` 长度。
- `CRC8`：CRC-8/ATM，多项式 `0x07`，初值 `0x00`，覆盖 `SRC + LEN_H + LEN_L + PAYLOAD`。
- 板 2 校验通过后发送的是完整帧，COM7 端需要按上述格式解析；不会自动剥离帧头和 CRC。

## 工程结构

| 路径 | 说明 |
| --- | --- |
| `OneLine1/` | 板 1 的 STM32CubeMX / Keil 工程 |
| `OneLine2/` | 板 2 的 STM32CubeMX / Keil 工程 |
| `OneLine1/Core/Src/main.c` | 双路接收、分帧、发送队列和错误恢复 |
| `OneLine2/Core/Src/main.c` | 单路接收、帧解析/CRC 校验、发送队列和错误恢复 |
| `docs/superpowers/` | 历史设计与实施记录；部分文档描述旧方案，以当前源码为准 |
| `tools/test-board1-rx-architecture.ps1` | 旧架构静态检查脚本，目前尚未同步到最新实现 |

## 可观测变量

板 1 主要变量：

- `rx_bytes`、`rx_bytes_u1`、`tx_bytes`
- `drop_cnt`、`frame_drop_cnt`、`err_cnt`
- `rearm_lp_cnt`、`rearm_u1_cnt` 及对应失败计数
- `main_loop_cnt`、`rx_start_stage`

板 2 主要变量：

- `rx_bytes`、`tx_bytes`、`drop_cnt`、`err_cnt`
- `frame_ok_cnt`、`frame_crc_err_cnt`、`frame_fmt_err_cnt`、`frame_drop_cnt`
- `noise_drop_bytes`、`rearm_rx_cnt`、`rearm_rx_fail_cnt`

## 已知边界

- 512 KiB 队列用于吸收突发，不等于端到端可靠传输；队列满时会整帧或按代码路径丢弃并计数。
- UART 错误恢复会重置 DMA 消费位置；错误窗口内未处理的数据可能丢失。
- 独立看门狗只在收到真实 RX 事件时喂狗；任一板超过约 1.1 秒没有数据会主动复位。这是当前代码行为，部署前应确认是否符合产品需求。
- `.ioc` 仍保留部分未参与当前数据通路的 DMA/UART 配置；重新生成 CubeMX 代码前应核对源码与配置的一致性。
- 历史提交曾多次出现 DMA 回绕和重启相关 HardFault。软件构建成功不能替代真实硬件上的长时间双路压力、断线恢复和数据完整性验证。

## 构建与验证

分别使用 Keil µVision 打开并构建：

- `OneLine1/MDK-ARM/OneLine1.uvprojx`
- `OneLine2/MDK-ARM/OneLine2.uvprojx`

建议至少验证：

1. COM8、COM10 单路短报文与跨 1024 B 边界报文。
2. 双路同时持续发送，按 `SRC` 分流后检查各路顺序和 CRC。
3. 连续运行、插拔串口及错误恢复，确认无 HardFault 且所有丢弃/错误计数符合预期。
4. COM7 解析后的载荷总数与板 1 两路接收计数对应一致。
