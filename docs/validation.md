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
