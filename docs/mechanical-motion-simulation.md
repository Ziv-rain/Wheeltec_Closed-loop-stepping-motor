# 钢球开环运动仿真

坐标约定：A'D' 中点为 `0 cm`，靠近 A' 为负，标定点 1 为 `-5 cm`；
靠近 D' 为正，标定点 2 为 `+5 cm`。默认任务是：

```text
0 cm -> +5 cm -> -5 cm
```

## 运行

在仓库根目录执行：

```powershell
python tools/mechanical_motion_sim.py
```

程序会自动拟合每一段的驱动角、制动角和持续时间，并生成：

- `simulation/mechanical_motion_profile.csv`：完整时间序列；
- `simulation/mechanical_motion_profile.svg`：位置及角度曲线；
- `simulation/mechanical_motion_profile.h`：供单片机程序参考的 C 轨迹表。

## 可调参数

```powershell
python tools/mechanical_motion_sim.py `
  --friction 0.012 `
  --damping 0.55 `
  --angle-ratio 1.0 `
  --motor-rate 100 `
  --motor-tau 0.06 `
  --min-angle -30 `
  --max-angle 45 `
  --targets-cm 5 -5 `
  --trials 2500
```

- `friction`：滚动阻力系数；
- `damping`：与速度成正比的阻尼，单位 `1/s`；
- `angle-ratio`：电机角到梁倾角的传动比；
- `motor-rate`：电机最大角速度，单位 `deg/s`；
- `motor-tau`：电机一阶响应时间常数，单位 `s`；
- `trials`：每一航段的拟合搜索量，越大通常越准、耗时越长。

模型把钢球视为无滑动滚动的实心球，沿梁方向的重力加速度系数为 `5/7`，
并加入库仑滚动阻力、粘性阻尼、电机速度限制和一阶响应。它用于给出首轮台架参数，
不等同于实物辨识结果。

## 上机注意

1. 先断开钢球或使用限位保护，确认正角确实使钢球向 D' 运动；方向相反时应改符号，
   不要直接运行整段序列。
2. 低速验证 `angle-ratio`、水平零点和电机角速度，随后用实际录像或视觉数据重新拟合
   `friction` 与 `damping`。
3. 开环轨迹对摩擦、梁变形和初始速度敏感。正式比赛应在每个标定点增加视觉反馈，
   至少对到点误差和残余速度进行闭环修正。
4. `mechanical_motion_profile.h` 是参考表，不会自动替换当前固件的四段状态机。
