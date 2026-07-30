# 电机 DEMO6 扫摆模式审阅报告

## 基本信息

- 仓库：`Ziv-rain/Wheeltec_Closed-loop-stepping-motor`
- 用户要求：检查并修改最新代码
- 源分支：`test/motor-bringup`
- 源提交：`c3036026af92f8c67390ac517f0a8f8465968a32`
- 审阅分支：`review/demo6-remote-review`
- 审阅日期：2026-07-31

## 审阅范围

检查 DEMO6 三角波目标生成、闭环目标接口、NVM 写入、配置合法性和基础编译。未完成指定 MSPM0 SDK 下的完整 CCS 链接和硬件运动测试。

## 发现的问题

### P0：扫摆模式高频擦写 MCU Flash

- 位置：`Control/closed_loop.c` 的 `CL_SetTargetAngle()`
- 影响：DEMO6 在主循环反复设置目标，原函数每次擦除并重写 1KB Flash 扇区，可能阻塞实时控制并快速耗尽 Flash 寿命。
- 处理结果：已移除设置普通目标时的 NVM 写入；NVM 仅保留给零点和方向等持久配置。

### P1：模式 5 被错误放行

- 位置：`Hardware/demo_config.h`
- 影响：模式范围扩展为 1..6 后，未实现且未正确初始化闭环的模式 5 也能通过编译。
- 处理结果：合法模式明确限制为 1、2、3、4、6。

### P1：扫摆目标按主循环速度重复更新

- 位置：`Control/app_demo.c`
- 影响：相同目标被无上限重复写入，时间行为依赖主循环负载。
- 处理结果：增加 20ms 固定更新周期，并在串口状态中输出闭环故障。

### P2：NVM 使用 NULL 依赖间接头文件

- 位置：`Hardware/nvm.c`
- 处理结果：增加 `<stddef.h>`。

## 实际修改

- `Control/app_demo.c`：固定周期更新扫摆目标，保留当前目标并输出故障状态。
- `Control/closed_loop.c`：普通目标更新不再写 Flash。
- `Hardware/demo_config.h`：增加更新周期和配置检查。
- `Hardware/nvm.c`：补充标准头。
- `docs/reviews/2026-07-31-demo6-review.md`：加入本审阅报告。

## 验证结果

- [x] `app_demo.c` TI ARM 编译器隔离语法检查通过
- [x] `closed_loop.c` TI ARM 编译器隔离语法检查通过
- [x] `nvm.c` 补齐 TI Flash 桩后语法检查通过
- [x] 三角波关键点：0ms=-20°、1500ms=0°、3000ms=20°、4500ms=0°、6000ms=-20°
- [x] `CL_SetTargetAngle()` 不再调用 `NVM_Save()`
- [x] `git diff --check` 通过
- [ ] 完整 CCS/SysConfig 构建
- [ ] 空载低速硬件测试

## 剩余风险

项目指定 SDK 和原生成路径在当前电脑不可用，尚未完成完整链接。机械方向、限位和编码器反馈必须上板验证。

## 烧录说明

- 建议审阅分支：`review/demo6-remote-review`
- 发布分支：尚未确认
- 首次烧录必须空载、低速，并准备急停
