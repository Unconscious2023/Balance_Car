# STM32F103 两轮自平衡小车

> 详细背景、硬件配置、代码架构 → 见 `.claude/skills/project-background.md`

## 最高指示
- **详细回答用户的每一个问题。** 每一个都要说清楚：现象、原因、方案、为什么。不要只改代码不解释。不要跳过。

## 严禁修改
- `Core/Inc/usart.h` — CubeMX 生成
- `stm32f1xx_*` 所有文件 — STM32 HAL 系统库
- `STM32F1xx_HAL_Driver/`、`CMSIS/` — 驱动库

## 谨慎修改
- `Core/Src/main.c` — CubeMX 生成，只改 USER CODE 区域
- `Core/Inc/main.h` — CubeMX 生成，GPIO 宏定义
- `MDK-ARM/my_balance.uvprojx` — Keil 工程文件（增删源文件需同步）

## Git 规范
- **只在你主动要求时才提交**，平时不改动不主动 commit
- **编译通过后才允许 commit**，编译报错时拒绝提交
- 提交格式: `<type>: <描述> (YYYY-MM-DD)`
- type: feat / refactor / fix / docs / chore
- 不 amend，不 force-push，每次提交保留历史
- **本地仓库，无云端 remote**，不需要 push

## 权限原则
- ✅ 自动: 项目目录下所有增删改查 + 整台电脑所有只读查询
- ⚠️ 确认: 提交推送、项目目录之外的修改/删除
- ❌ 禁止: 联网

## 踩坑记录
每次解决报错/问题后，**必须总结并写入** `.claude/skills/troubleshooting.md`。格式：问题现象 → 原因 → 解决方案 → 教训。
**遇到 bug 时优先查 troubleshooting.md**，看是否之前遇到过同类问题。

## 编译
Keil Assistant (VS Code 扩展) → 自动调用 `D:\Keil_v5_ARM\UV4\UV4.exe`，目标 `Balance_Car_KEil_HAL`，工程 `MDK-ARM\Balance_Car_KEil_HAL.uvprojx`

## 烧录
```bash
python tools/flash_stm32.py
```
- 自动识别 CH340K 串口，调用 `D:\tools\stm32flash.exe` 烧录
- **烧录前必须关掉所有串口监视器**（Serial Monitor、FlyMcu 等）
- **拔插 USB 重新上电**以触发 CH340K 自动进入 bootloader
- 128000 8E1，不需要 `-i` GPIO 序列（硬件自动处理 BOOT0/RESET）

## 拿不准先问
**只要有不明确、不确定的地方，一定要先问我，不要擅自做决策。**
包括但不限于：删哪些文件、改哪些配置、新增外设/模式是否硬件支持、CubeMX 生成的文件能不能动。
