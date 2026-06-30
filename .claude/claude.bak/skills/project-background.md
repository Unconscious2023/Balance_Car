# 项目背景 — STM32F103 两轮自平衡小车 (my_balance)

## 项目概述

这是基于 STM32F103RC 的两轮自平衡小车工程，使用 HAL 库开发，MDK-ARM (Keil) 编译。CubeMX 工程名为 `my_balance`。

## 小车硬件配置

### 保留的外设和传感器
| 外设 | 通信方式 | 用途 |
|------|----------|------|
| **MPU6050** | 软件 I2C (PB10=SDA, PB11=SCL) | 6轴姿态传感器，提供倾角和角速度 |
| **K210 AI 模块** | USART2 (PA2=TX, PA3=RX, 115200) | 视觉识别：巡线、颜色跟随、二维码、自学习 |
| **超声波 HC-SR04** | GPIO (PA0=TRIG, PA1=ECHO → TIM2) | 超声波测距、避障、跟随 |
| **蓝牙 BLE (从机)** | UART5 (PC12=TX, PD2=RX, 9600) | 手机蓝牙遥控、PID 参数调节 |
| **OLED SSD1306** | 软件 I2C (与 MPU6050 共用), **128×32 像素, 仅3行** | 模式显示、状态信息 |

### 板载基础外设（已保留）
| 外设 | 引脚 | 用途 |
|------|------|------|
| 直流电机 ×2 | TIM8 CH1-4 (PC6-PC9) | 平衡车驱动，4路PWM |
| 编码器 ×2 | TIM3 (PA6/PA7), TIM4 (PB6/PB7) | 电机转速测量 |
| LED | PB3 | 状态指示 |
| 蜂鸣器 | PA11 | 声音反馈 |
| 按键 | PA8 | 模式选择/确认 |
| 电池检测 | ADC1_IN5 (PA5) | 电量监控 |

### 已移除的外设（小车实物没有）
PS2 手柄、LIDAR 雷达、CCD 摄像头、红外循迹 (4路)、电磁循迹

## 项目结构

```
06291110/
├── Core/               ← CubeMX 生成的 HAL 初始化代码
│   ├── Inc/            main.h, gpio.h, tim.h, usart.h, adc.h, stm32f1xx_hal_conf.h
│   └── Src/            main.c, gpio.c, tim.c, usart.c, adc.c
├── BSP/                ← 板级支持包（外设驱动层）
│   ├── MPU6050/        陀螺仪驱动 + DMP 运动库
│   │   └── DMP/        inv_mpu.c + inv_mpu_dmp_motion_driver.c（InvenSense 官方库）
│   ├── OLED/           OLED SSD1306 I2C 驱动 + 7×10 字体
│   ├── Ultrasonic/     超声波传感器测距 + 避障
│   ├── Bluetooth/      蓝牙 BLE 驱动 + 新旧两版协议处理
│   ├── k210/           K210 USART2 接口
│   ├── Motor/          电机 PWM 驱动 (TIM8)
│   ├── Enconder/       编码器读取 (TIM3, TIM4)
│   ├── Battery/        电池电压检测 (ADC)
│   ├── Key/            按键驱动 (PA8)
│   ├── LED/            LED 驱动 (PB3)
│   ├── Beep/           蜂鸣器驱动 (PA11)
│   ├── Timer/          TIM6 1ms 系统时钟
│   ├── IIC_Software/   软件模拟 I2C (PB10, PB11)
│   ├── Usart1/         调试串口 (printf 重定向)
│   ├── INT_Sever/      MPU6050 外部中断服务
│   ├── delay/          DWT 微秒/毫秒延时
│   ├── bsp.c/.h        BSP 初始化入口
│   ├── AllHeader.h     全局头文件汇总
│   └── myenum.h        枚举定义（模式、状态、位带宏）
├── APP/                ← 应用层
│   ├── app.c/.h        主循环 dispatch
│   ├── app_control.c   平衡控制核心（MPU6050 中断回调 → 姿态解算 → PID 控制）
│   ├── app_motor.c/.h  电机 PWM 计算 + 编码器→速度转换
│   ├── app_user.c/.h   模式选择 + 新版汽车协议 + K210帧 + 航向闭环 + 超声波限速
│   ├── filter/         互补滤波
│   ├── KF/             卡尔曼滤波
│   ├── PID/            PID 控制器（直立环、速度环、转向环）
│   ├── mode/           模式选择与参数配置
│   ├── OLED_Show/      OLED 显示函数
│   └── APPK210/        K210 视觉应用（二维码、巡线、跟随、自学习、MNIST）
├── k210/               ← K210 端 Python 代码
│   └── line_follow_planner.py  巡线规划 + A5 协议帧发送
├── tools/              ← 工具脚本
│   └── flash_stm32.py  STM32 串口烧录脚本
├── disabled/           ← 已禁用的模块 (备份)
├── Drivers/            ← HAL 库 + CMSIS (禁止修改)
├── MDK-ARM/            ← Keil 工程文件
│   └── my_balance.uvprojx  工程文件
├── my_balance.ioc      ← CubeMX 配置文件
└── CLAUDE.md           ← 项目指令
```

## 控制架构

### 核心控制循环（5ms MPU6050 EXTI 中断）

```
MPU6050 INT (5ms) → app_control.c: HAL_GPIO_EXTI_Callback()
  ├─ Get_Angle()           ← 姿态解算 (DMP/卡尔曼/互补滤波 三选一)
  ├─ 读取编码器             ← TIM3(左轮) TIM4(右轮)
  ├─ Get_Velocity_Form_Encoder() ← 脉冲→mm/s
  ├─ Balance_PD()          ← 直立环 PD (倾角 + 角速度)
  ├─ Velocity_PI()         ← 速度环 PI (编码器偏差积分)
  ├─ Turn_PD()             ← 转向环 PD (Z轴角速度)
  ├─ PWM_Ignore()          ← 死区滤波 ±1300
  ├─ PWM_Limit()           ← 限幅 ±2600
  ├─ [BT/CL模式] Car_Diff_Turn() ← 陀螺航向闭环 PI
  ├─ Pick_Up/Put_Down      ← 拿起放下检测
  └─ Set_Pwm()             ← 写入 TIM8 CCRx → 电机
```

### 模式

当前活跃模式（Mode_select_v2）:
- **Bluetooth_Mode**: 新版汽车协议遥控 + 航向闭环 + PID 在线调参
- **ChaseLine_Mode**: K210 巡线控制 + 航向闭环

其他可实现模式（需修改 Mode_select_v2 启用）:
- Normal, U_Follow, U_Avoid, Weight_M, K210_QR, K210_Line, K210_Follow, K210_SelfLearn, K210_mnist

### 通信协议

**新版蓝牙汽车协议**（A5...5A 二进制帧）:
- 控制帧 4字节: `A5 <flags> <chk> 5A`
- PID调参帧 9字节: `A5 <7params> <chk> 5A`
- 遥测帧 7字节: `A5 <speed> <hdg> <dir> <tgt> <chk> 5A` (每100ms)

**K210 巡线帧** (USART2, 6字节): `A5 <speed> <turn> <dir> <chk> 5A`

**旧版蓝牙协议** (text-based): 仅 Normal/Weight_M 模式使用

## 代码注意事项

### 严禁修改的文件
- `Core/Inc/usart.h` ← CubeMX 自动生成
- 所有 `stm32f1xx_*` 开头的文件 ← STM32 HAL 系统库
- `STM32F1xx_HAL_Driver/` 目录下所有文件
- `CMSIS/` 目录下所有文件

### 需要谨慎修改的文件
- `Core/Src/main.c` ← CubeMX 生成，只改 USER CODE 区域
- `Core/Inc/main.h` ← CubeMX 生成，GPIO 引脚宏定义
- `MDK-ARM/my_balance.uvprojx` ← Keil 工程文件（增删源文件需同步）

### 编码规范
- 使用 `u8`/`u16`/`u32` 类型别名（定义在 `AllHeader.h` 中）
- 注释风格：中英双语
- 角度单位：度（°）
- 角度极性：小车前倾为正，后倾为负
- PID 参数：放大 100 倍存储（便于蓝牙调参）
- 编码器极性：左轮前进为正，右轮前进为负（`-Read_Encoder(MOTOR_ID_MR)`）
- 模式选择：用手拧轮子切换模式

### 关键全局变量
| 变量 | 含义 | 类型 |
|------|------|------|
| `Angle_Balance` | 平衡倾角 (°) | float |
| `Gyro_Balance` | 平衡角速度 (°/s) | float |
| `Gyro_Turn` | 转向角速度（Z轴） | float |
| `Motor_Left/Motor_Right` | 左右电机 PWM 输出 | int |
| `mode` | 当前模式 (Car_Mode 枚举) | Car_Mode |
| `Mid_Angle` | 机械中值 | int |
| `Stop_Flag` | 停止标志（1=停止, 0=运行） | u8 |
| `battery` | 电池电压 (V) | float |
| `g_distance` | 超声波测距值 (mm) | u32 |
| `Car_Target_Velocity` | 目标速度 | float |
| `Car_Turn_Amplitude_speed` | 转向幅度 | float |
| `GET_Angle_Way` | 姿态算法选择 (2=卡尔曼) | u8 |

### 修改代码时的检查清单
1. 新增模式需要在 `myenum.h` 的 `Car_Mode` 枚举中 `Mode_Max` 之前添加
2. 新增模式需要在以下文件中添加对应分支：
   - `BSP/bsp.c` → `bsp_mode_init()`
   - `APP/app.c` → `app_user()`
   - `APP/mode/app_mode.c` → `Set_Mid_Angle()` / `Set_angle()` / `Set_control_speed()` / `Set_PID()`
   - `APP/app_control.c` → `HAL_GPIO_EXTI_Callback()`（如需要）
   - `APP/OLED_Show/oled_show.c` → `show_mode_oled()`
3. 新增外设驱动放在 `BSP/` 下，应用层放在 `APP/` 下
4. 新增头文件需要在 `BSP/AllHeader.h` 中添加 `#include`

## 开发环境

### 编译
- **IDE**: Keil MDK-ARM v5 (UV4)
- **编译器**: ARMCC V5.06 update 7 (build 960)
- **UV4.exe**: `D:\Keil_v5_ARM\UV4\UV4.exe`
- **工程**: `MDK-ARM\my_balance.uvprojx`
- **目标**: `my_balance`
- **VS Code**: 使用 Keil Assistant 扩展自动调 UV4 编译

### 烧录/下载
- **USB 串口下载**: CH340K USB 自动下载电路（上电自动进入 bootloader）
- **SWD 调试**: ST-Link 通过 SWD 接口 (PA13=SWDIO, PA14=SWCLK)
- **烧录脚本**: `python tools/flash_stm32.py`
- **参数**: 57600 8E1

### 相关文件
- **CubeMX 工程**: `my_balance.ioc`
- **K210 模型**: `../K210涉及模型文件/`（相对于本工程目录）
