# Balance Car — Chaseline: K210 Vision Line Tracking

STM32F103 两轮自平衡小车，K210 视觉模块赛道循迹。

## 技术栈
- **MCU**: STM32F103RC, HAL 库, ARMCC V5.06
- **视觉**: K210 (Kendryte), OV2640 摄像头
- **传感器**: MPU6050 陀螺仪 (卡尔曼滤波)
- **控制**: 直立环 PD + 速度环 PI + 转向环 PD
- **通信**: USART2 K210↔STM32 (115200, 协议 `$XXXYYY#`)

## 启动
1. 上电 → OLED 显示 "Chaseline Mode"
2. 按 KEY → 初始化 K210 串口
3. 再按 KEY → 电机启动，开始循迹

## 架构
```
┌──────────┐  $XXXYYY#   ┌───────────────────┐
│   K210   │────────────→│     STM32F103     │
│ main.py  │  USART2     │ app_line.c        │
│ 4线检测   │             │ app_k210_ai.c     │
│ 双黑块中点│             │ app_control.c     │
└──────────┘             └───────────────────┘
```

### K210 (`My_K210/main.py`)
- 4 条水平 ROI 检测线，由远及近优先级递减
- 双黑块中点算法：白色虚线可见时取左右黑块中点，不可见时取单块质心
- 远处 ROI 预判弯道 → 提前减速
- 全部丢线 → 向最后偏离方向自转找回

### STM32
- `app_line.c` — 转向 PD 控制器，速度模式切换
- `app_k210_ai.c` — 串口帧解析 (`$XXXYYY#`)
- `app_control.c` — 直立环 + 速度环 + 转向环，电机输出
- `pid_control.c` — 三层 PID 核心

## 目录
```
├── APP/APPK210/          # K210 通信 + 巡线PID
├── APP/mode/             # 模式选择 (ChaseLine_Mode)
├── APP/PID/              # 直立环/速度环/转向环
├── BSP/                  # 外设驱动
├── Core/                 # CubeMX 生成
├── MDK-ARM/              # Keil 工程
├── My_K210/main.py       # K210 视觉脚本
└── tools/flash_stm32.py  # 烧录脚本
```

## 编译与烧录

### STM32
Keil MDK: 工程 `MDK-ARM/Balance_Car_KEil_HAL.uvprojx`，目标 `Balance_Car_KEil_HAL`

```bash
python tools/flash_stm32.py
```
- 关闭所有串口监视器，拔插 USB 重新上电
- 57600 8E1

### K210
将 `My_K210/main.py` 放入 K210 SD 卡根目录。

## 版权

> Copyright (c) 2025 Unconscious. All rights reserved.

本项目为**闭源私有项目**。严禁未经授权的复制、分发、修改或用于竞赛/商业目的。
