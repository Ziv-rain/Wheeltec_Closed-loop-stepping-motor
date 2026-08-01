# H题 车载平衡滚球运动控制系统 — 开发总结

## 1. 最终实现了什么

### 硬件配置（SysConfig）
| 引脚 | 名称 | 方向 | 电阻 | 用途 |
|------|------|------|------|------|
| PB0 | GPIO_KEY | INPUT | PULL_UP | SW1 — 启动键 |
| PA2 | GPIO_KEY | INPUT | PULL_UP | SW2 — 切题键 |
| PB16 | GPIO_KEY | INPUT | PULL_UP | SW3 — 结束键 |
| PB21 | GPIO_BUTTON | INPUT | PULL_UP | 板载按键（备用） |
| PA29/PA30 | I2C_OLED | — | — | OLED SSD1306 128×64 |

### 软件架构
```
SysTick 1kHz ISR
  ├── g_sysTick++（系统时钟，每 1ms）
  └── 每 10ms 扫描三个按键（各自独立去抖状态机）
        └── 上升沿 → 写入 g_keyEvent

main() 主循环
  └── StateMachine_Run()
        ├── TASK_SELECT  选题页：SW1启动 / SW2切题(T2~T6循环) / SW3回T2
        ├── TASK_RUNNING  运行页：TIME秒级计时(5s自动完成) / SW3紧急停止 / SW1/SW2锁定
        └── TASK_DONE    结果页：显示耗时 / SW1重跑 / SW2切题 / SW3回T2(不自动跳转)
```

### OLED 显示内容
- **选题页**：`H-Task H-2` + SW1/SW2/SW3 功能提示
- **运行页**：`RUNNING H-X` + `Time: Xs`（每秒刷新，5s 停）+ `SW3: Stop`
- **结果页**：`H-X DONE` + `Time: Xs` + `SW1:Re SW3:Home`

### 移植的驱动
- OLED 驱动（`BSP/OLED/`）：从天猛星模块代码完整移植，SSD1306 I2C，支持中英文字符和图片

---

## 2. 遇到的问题及解决方案

### 问题 1：OLED 移植后按键完全不响应

**现象**：按下 PB2/PB3/PB5/PB7 没有任何反应，OLED 一直显示 Hello。

**根因**：
1. 最初误以为按键在 PB2~PB7，实际板载用户按键在 **PB21**
2. `SYSCFG_DL_init()` 中 `Board.configureUnused = true` 会将所有未配置引脚初始化为**数字输出**（`DL_GPIO_initDigitalOutput` + `DL_GPIO_enableOutput`），PB21 被设为输出 LOW
3. 后续调用 `DL_GPIO_initDigitalInputFeatures` 只改写 PINCM 寄存器（GPIO 模式 + 上拉），**不会自动清除 DOE 位**，引脚方向仍为输出，无法读取外部电平
4. 手动写 `IOMUX->SECCFG.PINCM[21]` 时用错了索引——PB21 实际对应 `IOMUX_PINCM49`，而非 `IOMUX_PINCM21`

**解决**：
1. 在 `empty.syscfg` 中添加 `GPIO_BUTTON`（PB21, INPUT + PULL_UP），让 **SysConfig 自动生成正确配置**，PINCM 索引、DOE 清除等全部由工具处理
2. 后续调整为 PB0/PA2/PB16 三按键方案时，同样通过 SysConfig 配置，不再手动写寄存器

### 问题 2：设计方案的两次变更

**方案一（单按键 + 长短按）**：
- 只有 PB21 一个按键，短按启动/确认，长按 ≥1s 切题
- 按下时开始计时，释放时判断长短
- **放弃原因**：无法支持紧急停止，长按 1s 响应慢，裁判测试体验差

**方案二（三按键独立功能）**：
- SW1(PB0) 启动、SW2(PA2) 切题、SW3(PB16) 停止
- 纯短按（30ms 去抖），上升沿触发，不区分长短按
- 三个独立去抖状态机在 SysTick ISR 中运行
- **最终采用**

### 问题 3：OLED 刷新闪烁

**现象**：状态切换时屏幕先全白再显示新内容。

**根因**：`OLED_Clear()` 调用 `OLED_Refresh()` 把空白画面推到 OLED，然后绘制函数再次 `OLED_Refresh()`，两次 I2C 传输造成"内容→白屏→内容"的闪烁。

**解决**：用 `memset(OLED_GRAM, 0, ...)` 替代 `OLED_Clear()`，只在显存中清零，最后一次性 `OLED_Refresh()`。

### 问题 4：OLED 时间小数无法连续刷新

**现象**：要求 TIME 两位小数连续计数（如 0.01→0.02→…）。

**分析**：OLED_Refresh 全屏传输 8×128=1024 字节 I2C，耗时约 20ms。每秒刷新 100 次需要独占 I2C 总线，且 CPU 大部分时间在等 I2C 传输。

**解决**：去掉小数部分，只显示整秒，每秒刷新一次 TIME 行。DONE 页同样只显示整秒。

### 问题 5：TASK_DONE 页面闪烁与自动跳转

- 去掉 PASS/FAIL 和 Dev 偏差行，简化为仅显示 DONE + Time + 按键提示
- 去掉 3 秒超时自动返回，改为等待用户按键操作

---

## 3. 方案选择记录

| 决策点 | 选项 A | 选项 B | 选择 | 理由 |
|--------|--------|--------|:--:|------|
| 按键数量 | 单按键（长短按区分） | 三按键（独立功能） | B | 支持紧急停止，操作直观，去抖逻辑简单 |
| GPIO 配置方式 | 手动写寄存器 | SysConfig 图形化 | SysConfig | 避免 PINCM 索引错误、DOE 遗漏等问题 |
| 按键扫描位置 | 主循环轮询 | SysTick ISR | ISR | 10ms 精确定时，不阻塞主循环 |
| 去抖方式 | 记录按下时刻算时长 | 固定 30ms（3次10ms） | 固定30ms | 不需要区分长短按，逻辑极简 |
| 状态切换动画 | OLED_Clear + 重绘 | memset(GRAM) + 一次刷新 | memset | 消除白屏闪烁 |
| 计时显示精度 | 两位小数（10ms） | 整秒（1s） | 整秒 | I2C 带宽不够 100Hz 刷新 |
| TASK_DONE 退出 | 3 秒超时自动返回 | 手动按键操作 | 手动按键 | 用户需要足够时间查看结果 |
