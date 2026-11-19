"""Windows wired telemetry monitor and manual Bluetooth orientation control."""

from collections import deque
from queue import Empty, Full, Queue
from threading import Event, Thread
import argparse
import math
import time
import tkinter as tk
from tkinter import filedialog, ttk

from protocol import Capture, Lines, finite, telemetry


class Link:
    def __init__(self, port):
        self.port = port
        self.inbox = Queue(maxsize=2048)
        self.stop = Event()
        self.error = ""
        self.reader = Thread(target=self.read, daemon=True)
        self.reader.start()

    def read(self):
        lines = Lines()
        try:
            while not self.stop.is_set():
                data = self.port.read(min(1024, max(1, self.port.in_waiting)))
                for line in lines.feed(data):
                    self.inbox.put_nowait((time.time(), line))
        except Full:
            self.error = "接收队列已满，记录中断"
        except (OSError, ValueError) as exc:
            if not self.stop.is_set():
                self.error = str(exc)

    def send(self, line):
        data = (line + "\n").encode("ascii")
        if self.port.write(data) != len(data):
            raise OSError("串口写入不完整")

    def close(self):
        self.stop.set()
        self.port.close()
        self.reader.join(timeout=0.3)


class Heartbeat:
    """Only an explicit ARM action starts a heartbeat session."""

    def __init__(self):
        self.end()

    def start(self, now):
        self.enabled = True
        self.last_ack = now
        self.last_ping = now

    def end(self):
        self.enabled = False
        self.last_ack = 0.0
        self.last_ping = 0.0

    def reply(self, line, now):
        if line in {"ERR ARM", "OK STOP", "OK RESET"}:
            self.end()
        elif self.enabled and line in {"OK PING", "OK ARM"}:
            self.last_ack = now

    def tick(self, now):
        if self.enabled and now - self.last_ack >= 0.75:
            self.end()
            return "STOP"
        if self.enabled and now - self.last_ping >= 0.1:
            self.last_ping = now
            return "PING"
        return None


class Tone:
    """One finite, zero-centred test run, on one manually selected bench axis."""

    def __init__(self, axis, amplitude, hz, seconds):
        amplitude, hz, seconds = (finite(value) for value in (amplitude, hz, seconds))
        if axis not in ("yaw", "pitch") or min(amplitude, hz, seconds) <= 0:
            raise ValueError("须选择 yaw 或 pitch，并指定正幅值、频率和时长")
        if amplitude > 35:
            raise ValueError("幅值超出固件 ±35° 目标范围")
        if hz > 25 / 8 or seconds * hz < 4:
            raise ValueError("25 Hz 命令更新须覆盖每周期至少 8 点，记录须至少 4 个周期")
        self.axis, self.amplitude, self.hz, self.seconds = axis, amplitude, hz, seconds
        self.running = False

    def start(self, now):
        self.running = True
        self.start_time = self.next_at = now
        self.last_tx = now
        self.pending = False

    def tick(self, now):
        elapsed = now - self.start_time
        if elapsed >= self.seconds:
            return "STOP"
        if now - self.last_tx > min(0.3, 0.25 / self.hz):
            raise ValueError("串口应答或命令更新间隔过长")
        if self.pending or now < self.next_at:
            return None
        angle = self.amplitude * math.sin(2 * math.pi * self.hz * elapsed)
        self.pending = True
        self.last_tx = now
        self.next_at += 0.04
        # Skip overdue slots instead of replaying them in a burst.
        if self.next_at <= now:
            self.next_at = now + 0.04
        yaw, pitch = (angle, 0) if self.axis == "yaw" else (0, angle)
        return f"TARGET {yaw:.3f} {pitch:.3f}"


class Monitor:
    def __init__(self, root, serial_module, enumerate_ports, tone=None):
        self.root = root
        self.serial = serial_module
        self.enumerate_ports = enumerate_ports
        self.links = {}
        self.selectors = {}
        self.buttons = {}
        self.baud = {}
        self.capture = None
        self.closing = False
        self.history = deque(maxlen=2000)
        self.heartbeat = Heartbeat()
        self.tone = tone
        self.last_row = None
        self.last_received = 0
        self.telemetry_stale = False
        self.parameter_deadline = None
        self.latest = tk.StringVar(value="等待有线遥测")
        self.gain_report = tk.StringVar(value="—")
        self.record_status = tk.StringVar(value="")
        self.next_plot = 0.0
        root.title("两轴云台 · 姿态监视器")
        root.minsize(740, 550)
        top = ttk.Frame(root, padding=8)
        top.pack(fill="x")
        for index, (name, caption, default) in enumerate(
                (("usb", "有线 USB–TTL", 115200), ("bt", "蓝牙 SPP", 9600))):
            ttk.Label(top, text=caption).grid(row=index, column=0, sticky="w", padx=4)
            selector = ttk.Combobox(top, width=19)
            selector.grid(row=index, column=1, padx=4, pady=2)
            rate = ttk.Combobox(top, values=("9600", "38400", "57600", "115200"), width=9)
            rate.set(str(default))
            rate.grid(row=index, column=2)
            button = ttk.Button(top, text="连接", command=lambda key=name: self.toggle(key))
            button.grid(row=index, column=3, padx=4)
            self.selectors[name], self.baud[name], self.buttons[name] = selector, rate, button
        ttk.Button(top, text="刷新端口", command=self.refresh).grid(row=0, column=4, padx=4)
        ttk.Button(top, text="开始 CSV", command=self.record).grid(row=0, column=5, padx=4)
        ttk.Button(top, text="结束 CSV", command=self.end_record).grid(row=1, column=5, padx=4)
        ttk.Label(root, textvariable=self.record_status).pack(anchor="w", padx=12)

        tune = ttk.Frame(root, padding=(8, 2))
        tune.pack(fill="x")
        ttk.Label(tune, text="滤波 Kp").pack(side="left")
        self.gain = self.entry(tune, "1.5")
        ttk.Button(tune, text="写入", command=self.set_gain).pack(side="left")
        ttk.Button(tune, text="读取", command=self.get_gain).pack(side="left")
        ttk.Label(tune, text="设备回读").pack(side="left", padx=5)
        ttk.Label(tune, textvariable=self.gain_report).pack(side="left")
        control = ttk.Frame(root, padding=8)
        control.pack(fill="x")
        ttk.Label(control, text="yaw °").pack(side="left")
        self.yaw = self.entry(control, "0")
        ttk.Label(control, text="pitch °").pack(side="left")
        self.pitch = self.entry(control, "0")
        for label, action in (("发送目标", self.target), ("手动使能", self.arm),
                              ("停止", self.stop_motion), ("清故障", self.reset)):
            ttk.Button(control, text=label, command=action).pack(side="left", padx=2)
        if self.tone:
            ttk.Button(control, text="开始已标定频点", command=self.start_tone).pack(side="left", padx=2)
            ttk.Label(root, text=f"单频试验：{tone.axis} 轴 · 0° ± {tone.amplitude:g}° · "
                      f"{tone.hz:g} Hz · {tone.seconds:g} s。请确认行程已实测、两轴已回中。"
                      ).pack(anchor="w", padx=12)
        ttk.Label(root, textvariable=self.latest).pack(anchor="w", padx=12)
        self.canvas = tk.Canvas(root, bg="white", height=250, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=8, pady=4)
        self.log_box = tk.Text(root, height=6, state="disabled")
        self.log_box.pack(fill="x", padx=8, pady=4)
        self.refresh()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(20, self.poll)

    @staticmethod
    def entry(parent, initial):
        widget = ttk.Entry(parent, width=7)
        widget.insert(0, initial)
        widget.pack(side="left", padx=4)
        return widget

    def log(self, message):
        self.log_box.configure(state="normal")
        self.log_box.insert("end", time.strftime("%H:%M:%S ") + message + "\n")
        if int(self.log_box.index("end-1c").split(".")[0]) > 120:
            self.log_box.delete("1.0", "21.0")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def refresh(self):
        try:
            ports = [p.device for p in self.enumerate_ports()]
            for selector in self.selectors.values():
                selector.configure(values=ports)
        except OSError as exc:
            self.log(str(exc))

    def toggle(self, name):
        if name in self.links:
            if name == "bt":
                self.stop_motion()
            self.disconnect(name)
            return
        device = self.selectors[name].get().strip()
        if not device or any(link.port.port.upper() == device.upper() for link in self.links.values()):
            self.log("请选择独立的 COM 端口")
            return
        try:
            port = self.serial.Serial(device, int(self.baud[name].get()), timeout=0.025, write_timeout=0.1)
        except (ValueError, OSError) as exc:
            self.log(str(exc))
            return
        self.links[name] = Link(port)
        self.buttons[name].configure(text="断开")
        self.log(f"{device} 已连接")
        if name == "bt":
            self.heartbeat.end()
            self.send("bt", "STOP")
        else:
            self.history.clear()
            self.gain_report.set("—")
            self.get_gain()

    def disconnect(self, name):
        link = self.links.pop(name, None)
        if link:
            try:
                link.close()
            except OSError as exc:
                self.log(str(exc))
            self.buttons[name].configure(text="连接")
            self.log(f"{name} 已断开")
        if name == "bt":
            self.heartbeat.end()
        elif name == "usb":
            self.end_record()
            self.latest.set("有线连接已断开")
            self.gain_report.set("—")
            self.parameter_deadline = None
            self.last_row = None
        if self.tone and self.tone.running:
            self.stop_motion("连接中断")

    def send(self, name, line):
        if name not in self.links:
            self.log(f"{name} 未连接")
            return False
        try:
            self.links[name].send(line)
        except (ValueError, OSError) as exc:
            self.log(str(exc))
            self.disconnect(name)
            return False
        if line != "PING" and not (self.tone and self.tone.running and line.startswith("TARGET ")):
            self.log(f"{name} > {line}")
        return True

    def arm(self):
        if self.tone and self.tone.running:
            return
        if self.send("bt", "PING") and self.send("bt", "ARM"):
            self.heartbeat.start(time.monotonic())

    def stop_motion(self, reason="手动停止"):
        if self.tone and self.tone.running:
            self.tone.running = False
            self.log(f"频点结束 · {reason} · 最近设备 ms={self.last_row['ms']}")
        self.heartbeat.end()
        for name in list(self.links):
            self.send(name, "STOP")

    def reset(self):
        if self.tone and self.tone.running:
            self.stop_motion()
        self.heartbeat.end()
        self.send("usb" if "usb" in self.links else "bt", "RESET")

    def numeric_command(self, name, command, entries, lower, upper):
        try:
            numbers = [finite(widget.get()) for widget in entries]
            if any(number < lower or number > upper for number in numbers):
                raise ValueError(f"{command} 支持范围为 {lower:g} 至 {upper:g}")
            values = [f"{number:.4f}" for number in numbers]
        except ValueError as exc:
            self.log(str(exc))
            return False
        return self.send(name, " ".join([command, *values]))

    def target(self):
        if self.tone and self.tone.running:
            self.log("请先停止本次频点试验")
            return
        self.numeric_command("bt", "TARGET", (self.yaw, self.pitch), -35, 35)

    def get_gain(self):
        if self.send("usb", "GET"):
            self.gain_report.set("等待回读")
            self.parameter_deadline = time.monotonic() + 0.75

    def set_gain(self):
        if self.numeric_command("usb", "GAIN", (self.gain,), 0, 10):
            self.gain_report.set("等待回读")
            self.parameter_deadline = time.monotonic() + 0.75

    def record(self):
        if self.capture or "usb" not in self.links:
            self.log("请连接有线串口，并先结束上一份记录")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if path:
            try:
                self.capture = Capture(path)
                self.record_status.set(f"正在记录 {path}")
            except OSError as exc:
                self.log(str(exc))

    def end_record(self):
        if self.tone and self.tone.running:
            self.stop_motion("记录结束")
        if self.capture:
            capture, self.capture = self.capture, None
            try:
                capture.close()
                self.record_status.set(f"已保存 {capture.count} 帧 · {capture.file.name}")
            except OSError as exc:
                self.record_status.set(f"写入失败 · {exc}")

    def receive(self, name, received, line):
        if line is None:
            self.log(f"{name} 收到损坏或过长的行")
            return
        if name == "usb" and line.startswith("T,"):
            try:
                row = telemetry(line)
            except ValueError as exc:
                self.log(f"跳过损坏遥测 · {exc}")
                return
            point = (int(row["ms"]), *(float(row[key]) for key in ("roll", "pitch", "yaw")))
            if self.history and point[0] < self.history[-1][0]:
                self.history.clear()
                self.end_record()
                self.log("设备时间回退，请为新时间段重新开始记录")
            self.history.append(point)
            self.last_row, self.last_received = row, received
            self.telemetry_stale = False
            self.latest.set("{ms} ms    {state}    roll {roll}°    pitch {pitch}°    yaw {yaw}°    "
                            "Kp {gain}    漏周期 {missed}    执行 {exec_us} µs".format(**row))
            if row["state"] == "FAULT":
                self.heartbeat.end()
            if self.tone and self.tone.running and row["state"] != "ACTIVE":
                self.stop_motion("设备退出 ACTIVE")
            if self.capture:
                try:
                    self.capture.append(received, row)
                except OSError as exc:
                    self.end_record()
                    self.record_status.set(f"写入失败，记录已停止 · {exc}")
            return
        if name == "bt":
            self.heartbeat.reply(line, time.monotonic())
            if self.tone and self.tone.running:
                if line == "OK TARGET":
                    self.tone.pending = False
                if line.startswith("ERR"):
                    self.stop_motion(line)
        if line.startswith("PARAM "):
            try:
                if not 0 <= finite(line[6:]) <= 10:
                    raise ValueError("回读超出范围")
                self.gain_report.set(line[6:])
                self.parameter_deadline = None
            except ValueError:
                self.log(f"参数回读无效 · {line}")
        if name == "usb" and line in ("ERR GET", "ERR GAIN"):
            self.parameter_deadline = None
            self.gain_report.set("指令被拒绝")
        if line and line != "OK PING" and not (self.tone and self.tone.running and line == "OK TARGET"):
            self.log(f"{name} < {line}")

    def poll(self):
        for name, link in list(self.links.items()):
            for _ in range(160):
                try:
                    stamp, line = link.inbox.get_nowait()
                except Empty:
                    break
                self.receive(name, stamp, line)
            if link.error:
                self.log(link.error)
                self.disconnect(name)
        now = time.monotonic()
        if self.last_row and time.time() - self.last_received > 0.35 and not self.telemetry_stale:
            self.telemetry_stale = True
            self.latest.set("有线遥测已超时，最近设备时间 " + self.last_row["ms"] + " ms")
        if self.parameter_deadline is not None and now >= self.parameter_deadline:
            self.parameter_deadline = None
            self.gain_report.set("回读超时")
        action = self.heartbeat.tick(now)
        if action:
            if action == "STOP":
                self.log("蓝牙应答超时，手动使能会话已结束")
            self.send("bt", action)
        if self.tone and self.tone.running:
            if not self.heartbeat.enabled or time.time() - self.last_received > 0.35:
                self.stop_motion("心跳或有线遥测中断")
            else:
                try:
                    command = self.tone.tick(now)
                    if command == "STOP":
                        self.stop_motion("到达指定时长")
                    elif command:
                        self.send("bt", command)
                except ValueError as exc:
                    self.stop_motion(str(exc))
        if now >= self.next_plot:
            self.plot()
            self.next_plot = now + 0.1
        self.root.after(20, self.poll)

    def start_tone(self):
        if self.tone.running:
            return
        if (not self.capture or not self.heartbeat.enabled or not self.last_row or
                self.last_row["state"] != "ACTIVE" or time.time() - self.last_received > 0.35):
            self.log("请先开始有线记录、手动使能并确认 ACTIVE")
            return
        if any(abs(float(self.last_row[k])) > 0.01 for k in ("target_yaw", "target_pitch")):
            self.log("请先手动将两轴目标设为 0° 并确认已稳定")
            return
        recent = [p[0] for p in self.history if p[0] >= self.history[-1][0] - 1500]
        if len(recent) < 4 or max(b-a for a,b in zip(recent, recent[1:])) * self.tone.hz > 250:
            self.log("有线遥测节拍不足以分辨本次频率")
            return
        self.log(f"开始频点 · 设备 ms={self.last_row['ms']}")
        self.tone.start(time.monotonic())

    def plot(self):
        c = self.canvas
        c.delete("all")
        width, height = c.winfo_width(), c.winfo_height()
        if width < 100 or height < 50:
            return
        lo_x, hi_x = 50, width - 12
        lo_y, hi_y = 25, height - 22
        end = self.history[-1][0] if self.history else 0
        start = end - 10000
        points = [p for p in self.history if p[0] >= start]
        limit = max((abs(angle) for p in points for angle in p[1:]), default=0)
        scale = max(5.0, limit * 1.05)
        for value in (-scale, 0, scale):
            y = (hi_y + lo_y) / 2 - value / scale * (hi_y - lo_y) / 2
            c.create_line(lo_x, y, hi_x, y, fill="#ddd")
            c.create_text(lo_x - 4, y, text=f"{value:.1f}°", anchor="e")
        for index, (axis, color) in enumerate((("roll", "#08779a"), ("pitch", "#bd4f15"), ("yaw", "#337e35")), 1):
            c.create_text(lo_x + (index - 1) * 65, 10, text=axis, fill=color, anchor="w")
            coords = []
            for p in points:
                coords.extend((lo_x + (p[0] - start) / 10000 * (hi_x - lo_x),
                               (hi_y + lo_y) / 2 - p[index] / scale * (hi_y - lo_y) / 2))
            if len(coords) >= 4:
                c.create_line(coords, fill=color, width=2)
        c.create_text(lo_x, height - 8, text="−10 s", anchor="w")
        c.create_text(hi_x, height - 8, text="0 s", anchor="e")

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.stop_motion()
        for name in list(self.links):
            self.disconnect(name)
        self.end_record()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tone", nargs=4, metavar=("AXIS", "AMPLITUDE_DEG", "HZ", "SECONDS"),
                        help="启用单次标定台架频点按钮，执行前须手动使能并记录 CSV")
    args = parser.parse_args()
    try:
        tone = Tone(args.tone[0], *(finite(v) for v in args.tone[1:])) if args.tone else None
    except ValueError as exc:
        parser.error(str(exc))
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        raise SystemExit("请先运行 python -m pip install pyserial")
    root = tk.Tk()
    monitor = Monitor(root, serial, list_ports.comports, tone)
    try:
        root.mainloop()
    finally:
        monitor.close()


if __name__ == "__main__":
    main()
