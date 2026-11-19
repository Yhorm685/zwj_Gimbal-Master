"""Line framing and lossless telemetry capture for the desktop monitor."""

import csv
from datetime import datetime, timezone
import math
from pathlib import Path


FIELDS = ("ms", "roll", "pitch", "yaw", "target_yaw", "target_pitch",
          "state", "gain", "missed", "exec_us")
STATES = {"CAL", "READY", "ACTIVE", "FAULT"}


def finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("数值必须有限")
    return number


def natural(value):
    if not value.isascii() or not value.isdecimal():
        raise ValueError("计数字段不是无符号十进制整数")
    return int(value)


def telemetry(line):
    fields = line.split(",")
    if len(fields) != len(FIELDS) + 1 or fields[0] != "T":
        raise ValueError("遥测字段数或标记错误")
    row = dict(zip(FIELDS, fields[1:]))
    if row["state"] not in STATES:
        raise ValueError("未知状态")
    for name, value in row.items():
        if name in ("ms", "missed", "exec_us"):
            natural(value)
        elif name != "state":
            finite(value)
    return row  # retain the firmware's exact decimal spelling in the CSV


class Lines:
    """Discard a whole oversize/corrupt line and resynchronize at its LF."""

    def __init__(self):
        self.pending = bytearray()
        self.overflow = False

    def feed(self, data):
        for byte in data:
            if byte == 10:
                try:
                    yield None if self.overflow else self.pending.rstrip(b"\r").decode("ascii")
                except UnicodeDecodeError:
                    yield None
                self.pending.clear()
                self.overflow = False
            elif len(self.pending) < 256:
                self.pending.append(byte)
            else:
                self.overflow = True


class Capture:
    def __init__(self, path):
        self.file = Path(path).open("x", encoding="utf-8", newline="")
        self.count = 0
        self.writer = csv.DictWriter(self.file, ("received_utc", "sample", *FIELDS))
        try:
            self.writer.writeheader()
            self.file.flush()
        except OSError:
            self.file.close()
            raise

    def append(self, received, row):
        self.count += 1
        timestamp = datetime.fromtimestamp(received, timezone.utc).isoformat(timespec="milliseconds")
        self.writer.writerow({"received_utc": timestamp, "sample": self.count, **row})
        if self.count % 20 == 0:
            self.file.flush()

    def close(self):
        self.file.close()
