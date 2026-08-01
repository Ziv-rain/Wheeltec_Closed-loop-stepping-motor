# 编码器历史记录器审阅报告

## 范围

- 仓库：`Ziv-rain/Wheeltec_Closed-loop-stepping-motor`
- 源分支：`feat/accel-fusion`
- 源提交：`f1a33d3349ce3efc6005a92f45fb6dce05af00df`（开发期间新增的 `task=2` 兼容修复已同步保留）
- 审阅分支：`review/encoder-history-recorder`
- 底板源码基线：
  `C:\Users\lenovo\Desktop\电赛报告文件\2026-Basic Control _Finalall_Task456\2026-Basic Control _Finalall\2026-Basic Control`
- 底板新副本：`LowerBoard/2026-Basic-Control-Task456-EncoderLogger`

用户的原底板目录和原仓库脏工作树都没有覆盖或清理。

## 问题与风险级别

### P0

无新发现的 P0 代码问题。本版未经完整 CCS/SysConfig 链接和实车试验，因此不能标记为可直接烧录的发布版。

### P1

- 原方案若在行驶中通过 9600 baud 板间串口连续吐大日志，一个 14 B AA55 帧约占
  14.6 ms，会显著占用主循环并增加控制抖动风险。
- MSPM0G3507 SRAM 不适合同时保留十圈完整高维记录。已改为单圈固定容量、满后停止、停车后导出；
  新一圈开始前必须先保存上一圈。
- RAM 数据不持久，断电、复位或重新烧录都会丢失；现场导出期间必须保持两块板供电。

### P2

- 底板里程器只计旅程脉冲，不记方向；倒车段不能解释为负速度。
- 上层记录上限 38.4 s，底板为 76.8 s。超过上限时仍会安全停止记录，但较长行程会缺少后半段上层数据。

## 实际修改

- 底板增加 20 Hz RAM 记录器，记录时间戳、左/右轮脉冲增量、左/右 PWM、
  `base_speed` 和灰度位图。
- 上层增加 20 Hz RAM 记录器，记小球位置/速度、当前加速度、PID 输出、
  前馈角、目标/实际摆杆角和状态位。
- 新增停车后手动 `H` 导出。运行或手动视觉 PID 激活时拒绝导出。
- 新增 AA55 `0x40/0x05` 导出命令以及 `0x43`–`0x46` 历史帧；原有 `0x40`–`0x42`
  帧字段和 CRC 不变。
- 将原有 `0x42` 的编码器计数与底板时间戳做同时刻原子快照，供两层数据对齐。
- 增加离线工具，输出每样本 CSV 以及按 50 ms 相位聚合的 50/100 ms 前视加速度曲线。
- 将底板 `.cproject` 中其他电脑的 OLED 绝对路径改为 `${PROJECT_ROOT}/BSP/OLED`，不涉及运行参数。

## 控制与运动学不变性

- 未改 `Hardware/demo_config.h`、PID 增益、前馈增益、角度预算、斜率限制或内环频率。
- 未改底板循迹 PID、S 曲线启停、减速/刹车阈值、任务距离和 `base_speed`。
- 轮脉冲换算仍为 `0.0002618 m/count`，现有 16 位回绕处理不变。
- 运行时只在新的 20 Hz 车轮帧到达时写一次固定数组，没有串口打印和 Flash 写入。
- 历史加速度权重未接入实时控制；需等十组真车数据确认可重复性、符号和超前量后再开启。

## 内存评估

- TI Arm Clang 目标文件实测：上层记录模块 `.bss=15,371 B`，底板 `.bss=18,455 B`。
- 底板原始链接 map 显示 SRAM 为 32,768 B，原已用 2,367 B（含 512 B 栈）。
  按增量估算还余约 11.9 KB；仍需在 CCS 实际链接后复核最终 map。
- 上层旧 map 显示原 SRAM 占用低于 3 KB，加入 15.4 KB 后仍有明显余量；
  旧 map 不能替代当前分支的完整链接结果。

## 验证结果

- [x] `git diff --check`
- [x] Python `unittest`：日志容错、两层时间戳对齐、里程换算和 50/100 ms 前视字段，2 项通过
- [x] Python `py_compile`
- [x] TI Arm Clang Cortex-M0+ 语法检查：上层 `app_demo.c`/`task_ctrl.c`/`history_logger.c`
- [x] TI Arm Clang Cortex-M0+ 语法检查：底板 `empty.c`/`encoder_logger.c`/`aa55_proto.c`/`Drive.c`/`Odometry.c`
- [x] 底板副本与原目录做文件哈希核对；未列入修改范围的源文件保持一致
- [ ] 完整 CCS/SysConfig 编译、链接与最终 map 审查（本机缺少 MSPM0 SDK 2.11）
- [ ] 两层板实际 9600 baud 完整导出、断帧/队列溢出检查
- [ ] 实车运行对控制周期的无扰动确认

## 烧录/部署注意

仓库 `releaseBranch` 为空，本审阅分支不是指定发布分支。在下面条件完成前不建议直接上车：

1. 在 CCS 导入上层项目和底板副本，使用正确的 MSPM0 SDK/SysConfig 重新生成并全量构建；
2. 确认两个 map 中 SRAM 与栈余量；
3. 先架空车轮验证启动/停止不变，再跑 5–10 s 短记录并发 `H`；
4. 确认导出条数与 `BEGIN/END` 相等、`flags=0`、`dropped=0`后再进行整圈。
