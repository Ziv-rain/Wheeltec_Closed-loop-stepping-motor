# MSPM0G3507 闭环步进电机与平衡球控制

本工程用于天猛星 MSPM0G3507、MS42CG 编码器电机和 D36A
STEP/DIR 驱动器。当前 `DEMO_SELECT=5`，运行平衡球控制：

- TIMG0 每 5ms 更新编码器、任务状态、PID和位置闭环。
- UART2 接收视觉球位置。
- UART1 接收主控任务命令并上报状态。
- UART0 用于调试输出。
- 上电后使用编码器PWM绝对角度自动寻找水平位置。

## 关键接线

| 功能 | MSPM0引脚 |
| --- | --- |
| STEP | PA24 |
| DIR | PA13 |
| EN | PA12 |
| 编码器A/B | PA1 / PA0 |
| 编码器PWM | PB20 |
| 编码器Z | PA25 |
| 调试UART TX/RX | PA28 / PA31 |
| 主控UART TX/RX | PA17 / PA18 |
| 视觉UART TX/RX | PA21 / PA22 |

所有设备必须共地。编码器只能使用3.3V供电，禁止带电插拔电机功率线。
D36A拨码细分必须与 `D36A_MICROSTEP` 一致。

## 串口协议

帧格式：

```text
AA 55 TYPE LEN DATA... CRC_LO CRC_HI
```

CRC使用 CRC-16/IBM，初值 `0x0000`、多项式 `0xA001`，计算范围为
`TYPE + LEN + DATA`。

- `0x30`，视觉数据，长度8：球位置为前两个字节的小端有符号整数，
  单位0.01cm；随后为置信度和状态。
- `0x40`，任务命令，长度4：命令、任务号、两字节小端参数。
- `0x41`，状态上报，长度6：任务号、状态、球位置、电机角度。
- `0xFF`，心跳。运行状态下超过2.5秒未收到合法主控帧会停机并报故障。

任务命令：

| 命令 | 含义 |
| --- | --- |
| 1 | 切换任务 |
| 2 | 开始 |
| 3 | 停止 |
| 4 | 设置任务6目标位置，参数单位0.01cm |

目前任务3轨迹和任务6固定目标已经实现。任务4、5缺少赛题轨迹定义，
收到开始命令时会返回故障状态，避免误动作。

## 安全行为

- 自动回零具有等待超时、运动超时、PWM丢失、角度越界和无进展保护。
- 视觉数据超过200ms无效后停止位置闭环，不继续执行旧目标。
- 运行期间PWM绝对角度失效或越界会停止并进入故障状态。
- UART状态发送位于主循环，不会阻塞5ms控制中断和STEP计数中断。

机械角度、水平参考值和PID参数集中在
`Hardware/demo_config.h`。首次带机构调试应降低速度，并准备硬件急停。

## 构建

工程面向 Code Composer Studio、TI Arm Clang、MSPM0 SDK 2.11和
SysConfig。导入工程后应先重新生成 SysConfig 与 Debug 构建文件，
不要复用其他电脑生成的绝对路径构建文件。

## 板载 RAM 历史记录

`DEMO_SELECT=8` 下，赛题 4/5/6 运行期间以 20 Hz 把小球、控制器和底板车轮数据写入
RAM，不在行驶中打印大量串口日志，也不写 Flash。停车后保持两块板供电，
把 USB-TTL 连到上层板 UART0（115200-8-N-1），先在终端开启文本捕获，再发送一次 `H`。
导出的 `BALLLOG`/`CARLOG` 行可使用 `tools/analyze_encoder_history.py` 生成 CSV。

详细容量、协议、十组数据的操作步骤和风险说明见
[`docs/encoder-history-recorder.md`](docs/encoder-history-recorder.md)。
