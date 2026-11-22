# 工程验证记录

## 2026-11-17 至 18 日 · 控制与通信

任务二固定复用任务一 `84fd7fd` 的 F103 框架。控制和协议使用可在电脑上编译的 C++17 实现。测试输入包括静止、已知旋转、失联、限位、故障复位及非法帧。串口权限、上下电状态、权重回读和指令边界均通过主机断言。构建与运行命令如下。

```powershell
cmake -S tests -B build/native-check -G Ninja
cmake --build build/native-check
ctest --test-dir build/native-check --output-on-failure
python -m unittest discover -s host/tests -v
```

跨语言联测将合成 IMU 输入送入实际 C++ 控制器，用固件协议格式化应答与姿态，再由 Python 上位机解码并写入临时 CSV。CSV 依次记录 CAL、READY、ACTIVE、FAULT，目标为 yaw +3°、pitch −2°，Kp 读回 2.500。该记录用于验证接口，传感器来自软件输入。

上位机完成三轴绘图、数值回读、捕获和单频采集操作的离线检查。pySerial 3.5 的串口回环通过。电脑端 14 项测试覆盖协议和性能分析。两只真实舵机的动作、姿态漂移、蓝牙失联后的实际断电时间以及台架性能均待测。

## 2026-11-19 · F103 集成版本

在交付目录从固件提交 `7235f03` 使用 Arm GNU 14.3.1、CMake 和 Ninja 分别配置并构建 Debug、Release。两者均生成可供 Ozone 加载的 ELF 和裸 BIN。链接报告 RAM 18,880 / 20,480 B，Flash 45,460 / 65,536 B。此目录两份 BIN 的 SHA-256 相同，为 `9a8a72bd01372f4d77ca4fd0f22640b184e5b7d3413b1c011e60494e3937b37f`。RAM 数字包含静态预留区，实际任务栈与堆余量仍待板上检查。

CubeMX 再次生成后，139 个板级文件比对一致，检查排除了构建目录和本机生成的 .mxproject。主机 CTest 三组通过，覆盖控制、串口协议与双 UART；其中双 UART 使用 mock HAL 完成 122 次检查。

控制周期为 TIM2 标称 10 ms，SG90 PWM 为 50 Hz。PB12 控制外部舵机电源开关使能。独立看门狗 IWDG 只在控制任务处理完输出后刷新，按 40 kHz LSI 估计超时约 0.5 s，实物关断时延待测。

## 第一组实测记录

本次枚举未见可连接的 COM 端口。首轮保持舵机支路断电，通过 USB–TTL 接收有线遥测。连接后先开始 CSV 记录，再从有线端发送 `RESET` 重新校准，保持 IMU 水平静止 5 秒，然后缓慢倾斜并放平。保存从 `CAL` 到 `READY` 的 CSV。若进入 `FAULT`，保留首个故障前后的原始行。此轮无需蓝牙或舵机解锁。结构标定、动力保护、性能测量及三段演示视频待实物联调后补入。
