# Gimbal Master

任务二使用 STM32F103C8T6、MPU6050 和两只 SG90，制作 yaw–pitch 二轴自稳云台。上位机显示 roll、pitch、yaw 三轴姿态，Windows 或安卓通过 HC-05 经典蓝牙串口给定目标。yaw 以本次静止校准的方向为零。

## 开发基线

驱动来自 [任务一 zwj_Frameworks 的固定提交](https://github.com/Yhorm685/zwj_Frameworks/tree/84fd7fd32720641116e82bf68de881d31b826de0)。任务二在独立仓库内维护所需的 Common、BSP、Module 和 F103 CubeMX 工程，保留 HAL、CMSIS、FreeRTOS 原始许可证。构建只需本仓库。

## 构建与启动

从仓库根目录打开 PowerShell。固件需要 Arm GNU Toolchain、CMake 和 Ninja。

```powershell
cd boards/f103
cmake --preset Debug
cmake --build --preset Debug
cd ../..
```

使用 Ozone 打开 [F103 调试工程](boards/f103/f103.jdebug)，它会加载 `boards/f103/build/Debug/f103.elf`。Release 构建将两条命令的预设都改为 `Release`。首次烧录前先断开舵机电源，保留 IMU、调试器和 USB–TTL，便于单独验证采样与校准。

电脑端使用带 Tk 的 Python 3.10 或更高版本。安装 pySerial 后运行上位机。

```powershell
python -m pip install pyserial
python host/monitor.py
```

带 HC-05 的蓝牙串口负责目标指向，有线串口提供三轴波形、在线调参和 CSV 记录。接线、命令、测试和记录入口如下。

- [板级接线图和实机验收顺序](docs/bringup.md)
- [姿态解算、控制和命令协议](docs/control.md)
- [上位机操作与带宽、阶跃、重复精度分析](host/README.md)
- [构建与测试记录](docs/validation.md) · [AI 对话节选](docs/ai-session.md)

## 实施与验收

| 阶段 | 交付和验证 |
| --- | --- |
| 1. 建仓 | 确定框架版本、板卡、蓝牙方案和任务二验收目标 |
| 2. 固件 | 定时器触发控制、静止校准、姿态解算、双轴控制、失联和故障保护；主机测试和交叉构建 |
| 3. 通信和上位机 | 有线遥测与在线调参、蓝牙手动指向、CSV 记录、波形和指标分析 |
| 4. 上板 | 核对接线与供电，单轴空载检查，再联调机械、自稳、蓝牙及上位机 |
| 5. 性能交付 | 保存原始时间序列、独立角度参考和工况，录制静止自稳、蓝牙指向、在线调参视频 |

原题建议控制频率至少 50 Hz。合格线为带宽至少 2 Hz，小阶跃（不超过 10°）调节时间不超过 0.5 s 且稳态误差不超过幅值的 5%，约 30° 阶跃在 1 s 内进入 ±2°，重复指向误差在 ±2° 内，双轴合成绝对误差不超过 5°，静态抖动峰峰值不超过 2°。两轴应分别测量，执行器需独立供电，结构需刚性连接。
