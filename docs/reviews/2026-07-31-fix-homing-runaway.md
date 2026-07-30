# 上电回零持续单向转动诊断与修复报告

## 基本信息

- 仓库：`Ziv-rain/Wheeltec_Closed-loop-stepping-motor`
- 现象：上电初始化进入自动回零后，电机持续朝一个方向转动
- 源分支：`main`
- 源提交：`6b89236b856e0bde461cd64d778272976e03c028`
- 审阅分支：`review/fix-homing-runaway`
- 审阅日期：2026-07-31

## 直接根因

### P0：空命令队列路径没有恢复全局中断

- 位置：`Control/task_ctrl.c` 的 `popq()`
- 原逻辑：进入临界区前保存 `PRIMASK`，调用 `__disable_irq()`；当队列为空时却使用 `if (pk) __enable_irq()`。
- 错误：`pk == 0` 才表示进入临界区前中断是开启的，因此空队列分支的条件写反。
- 运行结果：
  1. 第一次 5ms 中断进入 `TaskCtrl_Tick5ms()`。
  2. 命令队列为空，`popq()` 设置 `PRIMASK=1` 后不恢复。
  3. 同一周期的 `BallControl_Tick5ms()` 启动连续回零 PWM。
  4. ISR 返回后全局中断仍被屏蔽，后续 5ms 定时中断不再执行。
  5. 回零超时、PWM 丢失、机械限位和无进展停机逻辑全部失效；硬件 PWM 独立运行，表现为电机持续单向转动。
- 修复：空队列路径改为 `if (!pk) __enable_irq()`，与成功出队路径保持一致。

## 次要安全问题

### P1：回零最佳误差没有更新

- 位置：`Control/ball_control.c` 的回零进展判断。
- 原逻辑：只在开始回零时记录 `lherr`。只要当前误差曾经小于初始误差，无进展计时就可能持续清零，即使之后已经停止继续接近水平点。
- 修复：每次检测到有效进展时同步执行 `lherr = e`，无进展计时现在相对于最近最佳误差判断。

## 修改文件

- `Control/task_ctrl.c`
- `Control/ball_control.c`
- `docs/reviews/2026-07-31-fix-homing-runaway.md`

## 验证

- [x] 检查所有 `PRIMASK` 保存/恢复路径
- [x] TI ARM 编译器隔离语法检查
- [x] 静态回归：不存在错误的 `if(pk)__enable_irq()` 空队列路径
- [x] 静态回归：有效回零进展会更新 `lherr`
- [x] `git diff --check`
- [ ] 完整 CCS/SysConfig 构建
- [ ] 上板空载回零测试

## 上板建议

1. 第一次测试拆除机械负载或确保摆杆不会撞限位，并准备立即断电。
2. 上电后确认串口仍持续输出，证明 5ms 中断没有再次被永久屏蔽。
3. 若回零方向正确，应逐步接近 `PWM_HORIZONTAL_REF=143°` 并停止。
4. 若方向相反，应在约 `HOMING_NO_PROGRESS_MS=1500ms` 后进入故障并停止，而不是持续转动；随后校正 `AXIS_X_POSITIVE_DIR_LEVEL` 或电机 DIR 接线。
5. 核对水平位置的 PWM 实测值是否仍为 143°，机械安装变化后必须重新标定。
