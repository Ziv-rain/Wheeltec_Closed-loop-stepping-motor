# Task 5 编码器历史加速度融合审阅报告

## 范围与基线

- 仓库：`Ziv-rain/Wheeltec_Closed-loop-stepping-motor`
- 源分支：`review/encoder-history-recorder`
- 源提交：`4b35a2d081c82076e1614925d54599f3bcadadd1`
- 其中同步的 `feat/accel-fusion` 基线：`5d397cebae53023ba30c2472287ed0f49fd84e15`
- 审阅分支：`review/encoder-history-fusion`
- 数据来源：10 个完整导出中的 9 圈 Task 5（`2.txt` 到 `10.txt`）；`1.txt` 是 Task 4，不进入 Task 5 曲线。
- 本次只改变 Task 5 的编码器加速度输入融合，不改 PID、运行轨迹、循迹、前馈角度增益、电机参数、限幅或斜率限制。

## 落地方案

1. 将 9 圈 Task 5 同相位的“未来 50 ms 加速度”中位数固化为 507 点只读表：
   - 采样周期：50 ms；
   - 覆盖：0 到 25.30 s；
   - 存储：`int16_t`，单位为 `0.001 m/s²`；
   - Flash 数据量约 1014 B，不占用大块运行 RAM。
2. 只在 `Task 5 + RUNNING + 相位有效 + 历史未越界` 时融合：

   `送入原 MechBalance_SetAccel 的值 = 0.70 × 实时编码器加速度 + 0.30 × 历史未来 50 ms 加速度`

3. 融合结果继续走原有 `MechBalance_SetAccel()` 的 EMA、死区、500 ms 断流衰减及原前馈通道。
4. 第一个新车轮数据包作为本圈相位零点；使用底板 `source_tick_ms`，无符号减法兼容 32 位时间戳回绕。
5. Task 4、Task 6、手动视觉模式、停止状态、历史越界或遥测无效时，直接将原实时加速度送入原算法。
6. STOP 时清除相位和历史输出；若任务号在 RUNNING 中异常变化，也重新对相。
7. 新增原子加速度快照，将加速度、对应底板时间戳、有效帧号和原始遥测帧号一次性读取，防止 UART 中断恰好插入两个读取之间造成偶发错相；计算公式和原参数不变。
8. `S` 自动状态新增：
   - `hp`：上一帧查到的历史未来 50 ms 加速度；
   - `hused`：上一帧是否实际使用历史融合。

## 权重与回退

权重集中在 `Control/accel_history_profile.h`：

```c
#define ACCEL_HISTORY_REAL_PERCENT      70U
#define ACCEL_HISTORY_HISTORY_PERCENT   30U
```

代码在编译期要求两者之和为 100。若实车效果不如原版，只改成下面两项即可在编译期关闭历史融合并恢复原实时输入，不需要删逻辑：

```c
#define ACCEL_HISTORY_REAL_PERCENT      100U
#define ACCEL_HISTORY_HISTORY_PERCENT   0U
```

本次没有采用离线拟合得到的约 1.10–1.15 增益，也没有使用 100 ms 预见，避免第一次上车过于激进。

## 离线证据

- 9 圈 Task 5 中心里程均值约 6.3125 m，标准差约 0.0032 m，圈间轨迹重复性足以先做小权重试验。
- 留一圈交叉验证、50 ms 目标下：
  - 以因果加速度 EMA 为实时基线时，RMSE 约从 `0.08667 m/s²` 降至 `0.07946 m/s²`，改善约 8.3%；
  - 以当前加速度保持为基线时，RMSE 约从 `0.09341 m/s²` 降至 `0.08134 m/s²`，改善约 12.9%。
- 这些数字只能证明历史曲线对车辆加速度预见有统计价值；记录中没有同步 BALLLOG，不能据此宣称小球误差一定缩小。

## 风险等级

### P0

无已知新增 P0 代码问题。但尚未完成当前分支的完整 CCS/SysConfig 构建、链接、map 审查和实车试验，所以不能称为可直接烧录的发布版。

### P1

- 历史相位依赖 Task 5 轨迹与采集时一致；电池、电机、轮胎打滑、赛道摩擦或循迹节拍明显变化时，历史预见可能错相。
- 只有车辆编码器数据，没有同步小球位置闭环数据；最终收益必须用真车球误差对比确认。
- 仓库 `releaseBranch` 为空，本审阅分支不是已批准发布分支。

### P2

- 50 ms 表由量化编码器数据生成，局部呈 `0.052 m/s²` 量级台阶；30% 权重和原 EMA 会降低台阶冲击，但状态输出中仍可能看到离散值。
- 表只覆盖约 25.3 s；更长行程会自动回退为原实时加速度，不会继续外推。

## 验证结果

- [x] 固化表与分析源 CSV 逐点核对：507/507，四舍五入后完全一致；范围 `-0.367` 到 `+0.471 m/s²`。
- [x] Python `unittest`：权重归一化、表长度/范围/时长和原历史分析测试共 5 项通过。
- [x] TI Arm Clang Cortex-M0+：`Control/app_demo.c` 和 `Control/task_ctrl.c` 在 `-Wall -Wextra -Werror -fsyntax-only` 下通过。
- [x] 使用编译宏切换为 `100/0` 后再次执行相同严格语法检查：通过，历史控制分支在编译期关闭。
- [x] `git diff --check`。
- [x] 团队基础检查 `tools/team-check.ps1 -Repository Wheeltec_Closed-loop-stepping-motor`：通过；脚本同时提示烧录前仍需完整 CCS/SysConfig 构建。
- [ ] 完整 CCS/SysConfig 编译、链接与最终 map 检查。
- [ ] 真车 Task 5 A/B 对比。

## 实车验证建议

1. 先架空车轮或抬起摆杆验证：Task 4/6 的 `hused` 始终为 0；Task 5 RUNNING 后 `hused=1`；STOP 后恢复 0。
2. 第一圈只跑一次 70/30，记录小球峰值误差、正负 1 cm 占比、停车后最大偏移和稳定时间，同时保存历史日志。
3. 与 `100/0` 同电量、同赛道对照；若 70/30 在连续两圈都没有改善，立即回退 100/0，不继续加权。
4. 不要在第一次试验同时改 PID、前馈增益或车辆轨迹，否则无法判断历史融合本身是否有效。
