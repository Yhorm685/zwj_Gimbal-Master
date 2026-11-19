# 云台上位机与台架数据分析

以下 PowerShell 命令从仓库根目录执行。Windows 版 Python 3.10 或以上需包含 Tk。串口界面使用 pySerial，离线分析与测试使用标准库。

```powershell
python -m pip install pyserial
python host/monitor.py
python -m unittest discover -s host/tests -v
```

## 接线、监视与调参

1. 将板上有线遥测 UART 通过 3.3 V USB–TTL 接至 Windows，选择相应 COM 并连接。默认 115200、8N1。界面显示设备时间、状态、漏周期、执行耗时和最近 10 s 的 roll、pitch、yaw。
2. 在 Windows 中配对 HC-05，若需要控制则选择它的出站 SPP 虚拟串口，与 USB–TTL 使用不同的 COM。HC-05 模块侧 UART 默认 9600、8N1，串口框允许修改波特率。连接蓝牙会发送 STOP。
3. 有线连接后点击“读取”获得设备当前 Kp，也可输入 0 至 10 的有限小数并点击“写入”，默认 Kp 为 1.5。“设备回读”来自固件的 PARAM 行。发送后显示等待回读，超过 0.75 s 会提示超时。遥测中的 Kp 列可用于比较实际运行数据，日志显示命令成功或拒绝。
4. 上电静止采集 200 个有效稳定样本后进入 READY。完成实物标定、回中和固件使能条件检查后点击“手动使能”。确认遥测为 ACTIVE，再发送已标定范围内的 yaw、pitch 角度，固件每轴支持 −35° 至 +35°。心跳在手动使能后约每 100 ms 发送。点击“停止”向已连接的两个端口发 STOP。“清故障”仅发 RESET，故障恢复后需要再次手动使能。
5. 点击“开始 CSV”选择**新文件名**。界面只记录有线端接收的有效遥测帧。“结束 CSV”显示文件位置和帧数。设备时间回退或有线端断开会结束该份记录。接收损坏、主机队列溢出或磁盘错误会在界面提示。遇到采集缺口请保留原文件并单独复测。

蓝牙会话连续 0.75 s 未收到有效 PING/ARM 应答后，上位机停止心跳并尝试 STOP。固件连续 500 ms 收不到有效心跳时锁定故障。物理链路断开时 STOP 可能无法送达，实际断电依赖板端的心跳超时和执行器电源保护。软件不会重连后自动解锁。

每行 CSV 含主机接收 UTC、文件内顺序号 sample 和固件字段 `ms,roll,pitch,yaw,target_yaw,target_pitch,state,gain,missed,exec_us`。角度单位为度，ms 是启动后的设备毫秒数。数值保留线路上的十进制文本。sample 只给本机收到的有效帧编号，missed 是控制任务漏周期计数。建议每份实测原始文件配记录，写明固件和主机版本、板号、轴、负载、供电、安装方向、Kp 和外部角度参考分辨率。

## 台架试验

下列文件名和时间戳为用法示意。按设备时间选择同一次上电的连续 ACTIVE 区间，并将真实原始文件另存，新 JSON 不覆盖已有结果。yaw 角差采用最短有符号角差。阶跃与静态抖动分别对两轴计算。

```powershell
python host/analyze.py --output small-yaw.json step yaw-step.csv --axis yaw --kind small --start-ms 12000 --end-ms 16000
python host/analyze.py step pitch-step.csv --axis pitch --kind large --start-ms 21000 --end-ms 24500
python host/analyze.py jitter still.csv --axis yaw --start-ms 12000 --end-ms 73000
```

- **阶跃**。固定底座，先记录稳定初始目标，再手动发送一次新目标并保持。时间段应包含跳变前的稳定帧。小阶跃按幅值的 ±5% 误差带，大阶跃在实测行程允许时选约 30°，按 ±2° 计算。调节时间从有线遥测首次显示新目标起算，取最后一次越界后进入误差带并保持至记录结束的时间。默认要求最后至少连续保持 0.5 s，可用 `--hold-s` 调整。稳态误差百分比采用最后保持窗口内的平均有符号误差绝对值除以阶跃幅值，同时给出窗口内最大绝对误差。结果为 null 表示在观察窗口内未证实调节完成。对照指标分别为小阶跃 ≤0.5 s 且稳态误差 ≤5%，大阶跃 ≤1.0 s。
- **静态抖动**。底座固定、目标恒定，每轴至少保留 60 s，从未经主机平滑的姿态数据计算峰峰值，对照 ≤2°。报告同时给出中位遥测速率。yaw 的漂移也会进入测得的峰峰值。
- **带宽**。先实测所选轴围绕 0° 的安全行程，保持另一轴为 0°。每次重新以一个频点启动上位机，例如下方命令启用 yaw 轴 ±2°、2 Hz、持续 8 s 的**手动试验按钮**。同一进程独占两路 COM。依次连接、开始有线 CSV、手动使能、将两轴目标置零并等待稳定。界面显示配置后点击“开始已标定频点”。每个 TARGET 需要应答，命令更新约 25 Hz；命令间隔异常、心跳或遥测丢失、设备退出 ACTIVE、操作员停止以及试验结束均尝试 STOP。机械卡滞、触及限位或饱和时应停止该点并记录。退出参数模式前保留本次 CSV 和日志里的设备起止时间。

```powershell
python host/monitor.py --tone yaw 2 2 8
python host/analyze.py --output yaw-bandwidth.json bandwidth --axis yaw --point 0.5 low.csv 14000 25000 --point 2 mid.csv 18000 23500 --point 3 high.csv 16000 20000
```

带宽分析应保留低频基线、约 2 Hz 和更高的频点；每个点的幅值、载荷、控制参数及安装条件保持一致。单频命令模式支持最高 3.125 Hz，每周期至少 8 次计划命令更新。先去除切入瞬态，再选择至少三个完整周期，每周期至少四个遥测采样位置。启动试验时会依据近期遥测间隔检查频率；离线计算再次按实际时间戳检查。工具按每帧的实际时间戳，对各点的目标与测得角度分别进行最小二乘正弦、余弦及直流偏置拟合，输出幅值比、相位差及拟合残差。这样可以处理与采样窗口不整周期对齐的已知频率。以最低实测频点的增益为参考，定位首次向下穿越 −3 dB 的区间，并在对数频率和 dB 间插值。对照带宽 ≥2 Hz；若测试频段未括住交点，或受命令更新率、遥测率或机械行程限制，则保留区间及限制，带宽值为 null。相位失真、输出残差过大或低频点尚未处于平坦段时，需结合原始曲线复核。阶跃的时间分辨率受遥测间隔限制，报告的调节时间应与该间隔一同阅读。

## 外部参考

重复精度和绝对误差都要填写独立角度盘或已标定相机的**真实读数**。每个 trial 记一次完成后的独立读数；全部角度采用与固件一致的坐标零点和方向。另存读数时间、标定经过时间、参考器具精度及原始照片或视频。每份重复试验文件选择单一轴并固定双轴目标，direction 填该轴到达目标时的方向 increasing 或 decreasing。两方向各至少 10 次，比较误差极值及每次误差与 ±2°。双轴绝对误差逐次计算 `hypot(yaw误差,pitch误差)`，对照 ≤5°。

CSV 表头如下。absolute 模式允许省略 direction。

```csv
trial,direction,target_yaw,target_pitch,ref_yaw,ref_pitch
```

```powershell
python host/analyze.py --output repeat-yaw.json repeat yaw-reference.csv --axis yaw
python host/analyze.py --output absolute.json absolute independent-reference.csv
```

本目录的单元测试使用程序构造的临时数据检验解析、状态处理及数学计算。正式达标结论须引用原始实机记录。
