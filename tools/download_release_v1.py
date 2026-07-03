#!/usr/bin/env python3
"""
STM32F103 烧录工具 — 自动识别 CH340 串口并烧录
用法: python tools/flash_stm32.py
"""

import subprocess
import sys
from pathlib import Path

# --- 配置 ---
PROJECT_ROOT = Path(__file__).resolve().parent.parent
STM32FLASH = r"D:\tools\stm32flash.exe"
HEX_FILE = PROJECT_ROOT / "release" / "release_v1" / "Balance_Car_KEil_HAL.hex"
# FLASH_BAUD = 115200         # bootloader 通信波特率
FLASH_BAUD = 128000         # bootloader 通信波特率
CH340_VID = 0x1A86          # 沁恒电子


def find_ch340_port():
    """扫描 COM 端口，返回第一个 CH340 端口号"""
    try:
        import serial.tools.list_ports
    except ImportError:
        print("[ERROR] pyserial 未安装，请运行: pip install pyserial")
        return None

    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid == CH340_VID:
            pid_hex = f"{p.pid:04X}" if p.pid else "????"
            print(f"[INFO] 检测到 CH340: {p.device} (PID:{pid_hex}) - {p.description}")
            return p.device

    print("[WARN] 未检测到 CH340 串口，当前可用端口:")
    for p in ports:
        vid_pid = f"VID:{p.vid:04X} PID:{p.pid:04X}" if p.vid else "无 VID/PID"
        print(f"  {p.device} - {vid_pid}")
    return None


def main():
    # 1. 检查 stm32flash.exe
    if not Path(STM32FLASH).exists():
        print(f"[ERROR] stm32flash.exe 不存在: {STM32FLASH}")
        sys.exit(1)

    # 2. 检查 hex 文件
    if not HEX_FILE.exists():
        print(f"[ERROR] hex 文件不存在: {HEX_FILE}")
        print("       请先编译工程")
        sys.exit(1)

    print(f"[INFO] HEX: {HEX_FILE.name} ({HEX_FILE.stat().st_size} bytes)")

    # 3. 找到 CH340 端口
    port = find_ch340_port()
    if not port:
        sys.exit(1)

    # 4. 烧录
    cmd = [
        STM32FLASH,
        "-w", str(HEX_FILE),
        "-v",              # 写入后校验
        "-g", "0x0",       # 烧完自动复位运行
        "-b", str(FLASH_BAUD),
        "-m", "8e1",       # 8 数据位, 偶校验, 1 停止位
        port,
    ]

    print(f"\n[CMD] {' '.join(cmd)}")
    print("[INFO] 开始烧录...\n")

    result = subprocess.run(cmd, capture_output=False, text=True)

    if result.returncode == 0:
        print(f"\n[OK] 烧录成功! 已通过 {port} 下载到 STM32")
    else:
        print(f"\n[FAIL] 烧录失败 (exit={result.returncode})，常见原因:")
        print("  1. 串口被占用 — 关掉 Serial Monitor 后重试")
        print("  2. 未进入 bootloader — 拔插 USB 重新上电")
        sys.exit(result.returncode)


if __name__ == "__main__":
    main()
