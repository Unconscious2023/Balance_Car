# Balance Car — STM32 + K210 Vision Tracking

STM32F103 两轮自平衡小车，K210 AI 模块视觉循迹。

## 技术栈
- **MCU**: STM32F103RC, HAL 库, ARMCC V5.06
- **视觉**: K210 (Kendryte), OV2640 摄像头
- **传感器**: MPU6050 陀螺仪 (DMP), HC-SR04 超声波
- **控制**: 直立环 PD + 速度环 PI + 航向闭环 PID
- **通信**: USART2 K210↔STM32 (115200), UART5 蓝牙 BLE (9600)

## 架构
```
K210 (视觉决策) ←→ STM32 (运动执行)
    │                    │
  road_vision_        app_control.c
  sensor.py           app_vision_turn.c
                      app_user.c
```

## 版权声明

> Copyright (c) 2025 Unconscious. All rights reserved.

本项目为**闭源私有项目**，仅供团队成员内部使用。

**严禁**以下行为：
- 未经授权的复制、分发、修改
- 将本项目代码用于任何竞赛、课程作业、商业目的
- 将本项目代码上传至任何公开平台

违反上述条款将追究法律责任。
