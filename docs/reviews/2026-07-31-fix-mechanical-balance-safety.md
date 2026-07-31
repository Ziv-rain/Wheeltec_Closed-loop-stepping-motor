# 力学前馈控制安全修复审阅报告

## 基本信息

- 仓库：`Ziv-rain/Wheeltec_Closed-loop-stepping-motor`
- 用户要求：审阅力学仿真代码，修复发现的问题并上传 GitHub
- 源分支：`feature/mechanical-balance`
- 源提交：`e3be16a7db9505dffadd7eb8b0d472673b7b10d4`（`feat: D command manual tilt angle mode for open-loop ball rolling test`）
- 审阅分支：`review/fix-mechanical-balance-safety`
- 审阅日期：2026-07-31

## 审阅范围

检查了 DEMO7 的上电回零、力学前馈角计算、位置闭环接管、PWM绝对角度限位、
UART0调参与急停、D36A细分配置和基础构建路径。没有改变视觉协议或任务协议。

## 发现的问题及处理

### P1：回零故障后力学控制会重新启动电机

- 位置：`Control/app_demo.c`、`Control/ball_control.c`
- 影响：回零超时、PWM丢失、方向错误或越界停机后，同周期的力学控制仍会重新设置目标。
- 处理：增加回零就绪/故障状态接口；DEMO7只有回零成功后才运行力学控制，回零故障会锁存停止力学控制。

### P1：DEMO7缺少运行期PWM和机械硬限位保护

- 位置：`Control/mech_balance.c`
- 影响：回零后PWM信号丢失或机构达到机械限位时仍可能继续发送位置目标。
- 处理：每个控制周期校验PWM绝对角度；硬限位立即锁存故障，软限位区禁止继续向外运动。

### P1：DEMO7缺少可靠急停

- 位置：`Control/app_demo.c`
- 影响：调参或回零方向错误时无法通过调试串口立即阻止后续周期重新驱动。
- 处理：任意时刻收到UART0字符`X/x`立即清空命令缓冲、停止电机并锁存回零和力学控制故障；必须重新上电才能恢复。

### P1：调参解析和参数范围不安全

- 位置：`Control/app_demo.c`、`Control/mech_balance.c`
- 影响：旧解析器会接受尾随垃圾字符；负角度限幅或负变化率会形成颠倒边界；极端参数没有限制。
- 处理：改为严格整行解析，要求命令后只有一个合法数字；校验加速度、增益、微调、角度限幅和角速度范围。
- 后续源提交新增的`D`手动倾角命令已保留，并限制在当前安全角度范围内；发送合法`A`命令退出手动倾角模式。

### P1：正常球控和回零硬限位保护不一致

- 位置：`Control/ball_control.c`
- 影响：源分支改写球控后遗漏了原有硬限位故障判断，回零又允许越过声明的硬限位到传感器容差区。
- 处理：恢复正常球控硬限位判断，并让回零在真正的`PWM_LIMIT_LOW/HIGH`处停止。

### P1：D36A细分配置与说明不一致

- 位置：`Hardware/motor.h`、`empty.c`、`README.md`
- 影响：代码为32细分，旧说明仍要求16细分；硬件拨码不匹配会造成步数和角度比例错误。
- 处理：保留开发分支的32细分配置，统一说明为上电前按D36A手册确认32细分。未确认拨码前不得烧录运行。

### P2：同一5ms中断重复调用四次闭环处理

- 位置：`Control/app_demo.c`
- 影响：第一次启动步进脉冲后电机为busy，后三次调用直接返回，只增加中断负担。
- 处理：DEMO5和DEMO7都恢复为每周期调用一次`CL_Process()`。

### P3：注释混入开发者本机绝对路径

- 位置：`Control/task_ctrl.c`
- 处理：删除本机路径并恢复协议解析器注释。

## 实际修改文件

- `Control/app_demo.c`
- `Control/ball_control.c`
- `Control/ball_control.h`
- `Control/mech_balance.c`
- `Control/mech_balance.h`
- `Control/task_ctrl.c`
- `README.md`
- `empty.c`
- `docs/reviews/2026-07-31-fix-mechanical-balance-safety.md`

## 验证结果

- [x] `git diff --check`
- [x] TI Arm Clang逐文件语法编译
- [ ] 匹配版本CCS/SysConfig完整链接
- [ ] 硬件回零、急停和限位测试
- [ ] 与车辆主控或IMU联调

TI Arm Clang检查使用 Cortex-M0+、soft-float、C11、`-Wall -Wextra -Werror`
编译 `Control/mech_balance.c`、`Control/ball_control.c` 和
`Control/app_demo.c`，结果为零错误、零警告。

本机只有CCS 12.4，而工程元数据来自更高版本CCS；CCS 12.4返回
“project meta-data cannot be interpreted”，因此未把完整链接标记为通过。

## 剩余风险

- 模式7当前加速度只由UART0的`A<value>`手动注入，尚未接入真实车辆主控或IMU。
- 纯力学前馈只能抵消已知加速度扰动，不能把偏离的球重新拉回目标点；比赛控制应把它作为球位置PID的前馈项。
- Cortex-M0+没有硬件浮点，5ms中断内`atan2f()`的最坏执行时间尚未实测。
- 必须在低速、卸载或有机械急停条件下确认DIR正方向、`PWM_HORIZONTAL_REF`、上下限和D36A 32细分拨码。
- 源分支已提交的`Debug/Wheeltec_Closed-loop-stepping-motor.txt`是生成文件并含大量尾随空格，本次没有重新生成或提交该文件。

## 烧录说明

- 审阅分支：`review/fix-mechanical-balance-safety`
- 本仓库尚未确认`releaseBranch`，该审阅分支不是正式可烧录发布分支。
- 烧录前必须使用匹配版本CCS和MSPM0 SDK 2.11完成全量构建。
- 首次上电先断开负载或限制行程，确认32细分拨码；准备硬件急停并观察回零方向。
- 验证PWM丢失、触发上下限和UART0发送`X`后电机均保持停止，不会自动恢复。
