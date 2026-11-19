"""Offline analysis of bench data. Outputs JSON, leaving input CSVs intact."""

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean, median

from protocol import finite, natural


def read_csv(path, needed):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = set(needed) - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"{path} 缺少列 {sorted(missing)}")
        if len(reader.fieldnames) != len(set(reader.fieldnames)):
            raise ValueError(f"{path} 存在重复列名")
        rows = list(reader)
    if not rows:
        raise ValueError(f"{path} 没有样本")
    if any(None in row or any(row.get(name) is None for name in needed) for row in rows):
        raise ValueError(f"{path} 存在字段数量不完整的行")
    return rows


def delta(value, reference, axis):
    difference = value - reference
    return (difference + 180.0) % 360.0 - 180.0 if axis == "yaw" else difference


def continuous(values, axis):
    series = [values[0]]
    for prev, value in zip(values, values[1:]):
        series.append(series[-1] + delta(value, prev, axis))
    return series


def segment(path, axis, start=None, end=None):
    target = "target_" + axis
    rows = read_csv(path, ("ms", axis, target, "state"))
    times = [natural(r["ms"]) for r in rows]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("设备时间未严格递增，请分开处理复位或回绕两侧的记录")
    selected = [(t, r) for t, r in zip(times, rows)
                if (start is None or t >= start) and (end is None or t <= end)]
    if len(selected) < 3:
        raise ValueError("选定时间段至少需要 3 帧")
    if any(r["state"] != "ACTIVE" for _, r in selected):
        raise ValueError("选定区间包含非 ACTIVE 帧，请用 --start-ms / --end-ms 选择有效区间")
    times = [(t - selected[0][0]) / 1000 for t, _ in selected]
    gaps = [b - a for a, b in zip(times, times[1:])]
    if max(gaps) > 3 * median(gaps) + 1e-6:
        raise ValueError("时间段包含超过中位采样间隔三倍的缺口")
    measured = continuous([finite(r[axis]) for _, r in selected], axis)
    commands = continuous([finite(r[target]) for _, r in selected], axis)
    return times, commands, measured


def step(times, commands, measured, axis, kind, hold):
    if hold <= 0:
        raise ValueError("保持时间必须为正")
    transitions = [i for i in range(1, len(commands)) if abs(commands[i] - commands[i - 1]) > 1e-6]
    if len(transitions) != 1:
        raise ValueError("区间须包含一次目标角跳变及跳变前后的恒定目标")
    index, target = transitions[0], commands[-1]
    amplitude = abs(target - commands[0])
    if kind == "small" and amplitude > 10.000001:
        raise ValueError("小角度阶跃幅值须 ≤10°")
    if kind == "large" and amplitude <= 10.000001:
        raise ValueError("大角度工况应在安全行程内选约 30° 阶跃")
    band = 0.05 * amplitude if kind == "small" else 2.0
    t0 = times[index]
    if times[-1] - t0 + 1e-9 < hold:
        raise ValueError("阶跃后的数据不足一个完整保持窗口")
    errors = [delta(x, target, axis) for x in measured]
    outside = [i for i in range(index, len(times)) if abs(errors[i]) > band + 1e-8]
    first = outside[-1] + 1 if outside else index
    held = first < len(times) and times[-1] - times[first] + 1e-9 >= hold
    tail = [e for t, e in zip(times, errors) if t + 1e-9 >= times[-1] - hold]
    return {"axis": axis, "step_deg": target - commands[0], "band_deg": band,
            "sample_interval_s": median(b-a for a,b in zip(times, times[1:])),
            "hold_s": hold, "observed_after_step_s": times[-1] - t0,
            "settling_s": times[first] - t0 if held else None,
            "steady_signed_error_deg": fmean(tail),
            "steady_error_percent": 100 * abs(fmean(tail)) / amplitude,
            "steady_peak_abs_error_deg": max(map(abs, tail))}


def jitter(times, commands, measured, axis):
    if times[-1] - times[0] < 60:
        raise ValueError("静态抖动要求至少 60 s 数据")
    if max(commands) - min(commands) > 1e-6:
        raise ValueError("抖动测试区间内的目标须恒定")
    return {"axis": axis, "samples": len(times), "duration_s": times[-1] - times[0],
            "median_sample_hz": 1 / median(b-a for a,b in zip(times, times[1:])),
            "peak_to_peak_deg": max(measured) - min(measured)}


def harmonic(times, values, frequency):
    # Least-squares fit of offset + a*sin(wt) + b*cos(wt) at actual sample times.
    terms = [(math.sin(2*math.pi*frequency*t), math.cos(2*math.pi*frequency*t)) for t in times]
    sin_mean = fmean(s for s, _ in terms)
    cos_mean = fmean(c for _, c in terms)
    value_mean = fmean(values)
    ss = cc = sc = sy = cy = 0.0
    for (sine, cosine), value in zip(terms, values):
        s, c, y = sine - sin_mean, cosine - cos_mean, value - value_mean
        ss += s*s
        cc += c*c
        sc += s*c
        sy += s*y
        cy += c*y
    det = ss*cc - sc*sc
    if det <= 1e-12 * max(1, ss*cc):
        raise ValueError("采样相位不足以拟合正弦波")
    a, b = (sy*cc - cy*sc) / det, (cy*ss - sy*sc) / det
    offset = value_mean - a*sin_mean - b*cos_mean
    rms = math.sqrt(fmean((value - offset - a*s - b*c)**2 for (s, c), value in zip(terms, values)))
    return math.hypot(a, b), math.atan2(b, a), rms


def frequency_point(path, frequency, axis, start=None, end=None):
    times, commands, measured = segment(path, axis, start, end)
    if frequency <= 0 or (times[-1] - times[0]) * frequency + 1e-8 < 3:
        raise ValueError(f"{path} 至少需要 3 个稳态周期，频率须为正")
    if max(b-a for a,b in zip(times, times[1:])) * frequency > 0.25 + 1e-8:
        raise ValueError(f"{path} 遥测间隔过大，每个周期至少需要 4 个采样位置")
    amp_in, phase_in, residual_in = harmonic(times, commands, frequency)
    amp_out, phase_out, residual_out = harmonic(times, measured, frequency)
    if amp_in <= 1e-6 or residual_in > 0.25 * amp_in:
        raise ValueError(f"{path} 输入须为指定频率的稳态正弦，检查时间段及拟合残差")
    return {"file": str(path), "frequency_hz": frequency, "input_amplitude_deg": amp_in,
            "output_amplitude_deg": amp_out, "gain": amp_out / amp_in,
            "phase_deg": delta(math.degrees(phase_out), math.degrees(phase_in), "yaw") if amp_out > 1e-6 else None,
            "input_fit_rms_deg": residual_in, "output_fit_rms_deg": residual_out}


def bandwidth(points):
    points = sorted(points, key=lambda p: p["frequency_hz"])
    if len(points) < 2 or len({p["frequency_hz"] for p in points}) != len(points):
        raise ValueError("请提供至少两个不同的频点")
    reference = points[0]["gain"]
    if reference <= 1e-9:
        raise ValueError("低频参考增益过小")
    threshold = 10**(-3/20)
    crossing, bracket = None, None
    for p in points:
        ratio = p["gain"] / reference
        p["gain_db_relative"] = 20 * math.log10(ratio) if ratio > 1e-12 else None
    for prev, curr in zip(points, points[1:]):
        if prev["gain"] > reference * threshold >= curr["gain"]:
            bracket = [prev["frequency_hz"], curr["frequency_hz"]]
            if curr["gain_db_relative"] is not None:
                fraction = (-3-prev["gain_db_relative"]) / (curr["gain_db_relative"]-prev["gain_db_relative"])
                crossing = bracket[0] * (bracket[1]/bracket[0])**fraction
            break
    return {"reference_hz": points[0]["frequency_hz"], "crossing_bracket_hz": bracket,
            "range_note": ("已括住首次下降穿越 -3 dB 的频点" if bracket else
                           "所测频点未括住下降穿越 -3 dB 的位置，须扩展频段；带宽未确定"),
            "interpolated_minus3db_hz": crossing, "points": points}


def external(path, axis=None):
    required = ["trial", "target_yaw", "target_pitch", "ref_yaw", "ref_pitch"]
    rows = read_csv(path, [*required, "direction"] if axis else required)
    ids = [r["trial"].strip() for r in rows]
    if "" in ids or len(set(ids)) != len(ids):
        raise ValueError("每次试验需要独立且非空的 trial 编号")
    results = []
    for trial, row in zip(ids, rows):
        errs = {a: delta(finite(row["ref_" + a]), finite(row["target_" + a]), a)
                for a in ("yaw", "pitch")}
        if axis:
            results.append({"trial": trial, "direction": row["direction"].strip(),
                            "error_deg": errs[axis]})
        else:
            results.append({"trial": trial, "yaw_error_deg": errs["yaw"], "pitch_error_deg": errs["pitch"],
                            "combined_deg": math.hypot(errs["yaw"], errs["pitch"])})
    if not axis:
        return {"max_combined_deg": max(x["combined_deg"] for x in results), "trials": results}
    for name in ("target_yaw", "target_pitch"):
        if len({finite(row[name]) for row in rows}) != 1:
            raise ValueError("重复试验须使用同一个双轴目标")
    if any(x["direction"] not in ("increasing", "decreasing") for x in results):
        raise ValueError("direction 仅支持 increasing 或 decreasing")
    groups = {}
    for direction in ("increasing", "decreasing"):
        errors = [x["error_deg"] for x in results if x["direction"] == direction]
        if len(errors) < 10:
            raise ValueError(f"{direction} 方向至少需要 10 次独立试验")
        groups[direction] = {"count": len(errors), "min_error_deg": min(errors),
                             "max_error_deg": max(errors), "mean_error_deg": fmean(errors)}
    return {"axis": axis, "max_abs_error_deg": max(abs(x["error_deg"]) for x in results),
            "directions": groups, "trials": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="另存新的 JSON，默认输出到终端")
    modes = parser.add_subparsers(dest="mode", required=True)
    for mode in ("step", "jitter", "bandwidth", "repeat", "absolute"):
        sub = modes.add_parser(mode)
        if mode != "absolute":
            sub.add_argument("--axis", choices=("yaw", "pitch"), required=True)
        if mode != "bandwidth":
            sub.add_argument("file", type=Path)
        if mode in ("step", "jitter", "bandwidth"):
            sub.add_argument("--start-ms", type=int)
            sub.add_argument("--end-ms", type=int)
        if mode == "step":
            sub.add_argument("--kind", choices=("small", "large"), required=True)
            sub.add_argument("--hold-s", type=finite, default=0.5)
        if mode == "bandwidth":
            sub.add_argument("--point", nargs="+", action="append", required=True, metavar="VALUE",
                             help="HZ CSV 或 HZ CSV START_MS END_MS，可重复指定")
    args = parser.parse_args()
    try:
        if args.mode in ("step", "jitter"):
            args_in = segment(args.file, args.axis, args.start_ms, args.end_ms)
            if args.mode == "step":
                result = step(*args_in, args.axis, args.kind, args.hold_s)
            else:
                result = jitter(*args_in, args.axis)
        elif args.mode == "bandwidth":
            points = []
            for spec in args.point:
                if len(spec) not in (2, 4):
                    raise ValueError("--point 格式为 HZ CSV 或 HZ CSV START_MS END_MS")
                start, end = map(int, spec[2:]) if len(spec) == 4 else (args.start_ms, args.end_ms)
                points.append(frequency_point(spec[1], finite(spec[0]), args.axis, start, end))
            result = bandwidth(points)
        else:
            result = external(args.file, args.axis if args.mode == "repeat" else None)
        report = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)
        if args.output:
            with args.output.open("x", encoding="utf-8") as stream:
                print(report, file=stream)
        else:
            print(report)
    except (OSError, ValueError, TypeError, OverflowError, csv.Error) as exc:
        parser.exit(2, f"数据分析失败 · {exc}\n")


if __name__ == "__main__":
    main()
