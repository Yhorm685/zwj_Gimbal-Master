"""Synthetic checks of analysis, serial framing and manual control lifecycle."""

import csv
import io
import math
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest

HOST = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST))
import analyze
from monitor import Heartbeat, Link, Monitor, Tone
from protocol import Capture, FIELDS, Lines, telemetry


class ProtocolTests(unittest.TestCase):
    def test_firmware_capture_preserves_decimals_and_order(self):
        lines = Lines()
        self.assertEqual(list(lines.feed(b"T,230,1.250,-2.000,")), [])
        [frame, ack] = lines.feed(b"3.100,0,0,READY,1.500,0,401\r\nOK GET\n")
        self.assertEqual(ack, "OK GET")
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "raw.csv"
            capture = Capture(path)
            capture.append(1.125, telemetry(frame))
            capture.close()
            with path.open(encoding="utf-8", newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(tuple(row), ("received_utc", "sample", *FIELDS))
            self.assertEqual([row[k] for k in FIELDS], frame.split(",")[1:])
            self.assertEqual(row["sample"], "1")
            self.assertEqual(row["received_utc"], "1970-01-01T00:00:01.125+00:00")
            with self.assertRaises(FileExistsError):
                Capture(path)

    def test_framing_recovers_after_overlong_and_nonascii(self):
        frames = list(Lines().feed(b"x" * 257 + b"\n\xff\nOK ARM\n"))
        self.assertEqual(frames, [None, None, "OK ARM"])

    def test_reject_invalid_telemetry(self):
        base = "T,10,0,0,0,0,0,ACTIVE,1,0,12"
        for bad in ("T,0", base.replace("ACTIVE", "BAD"), base.replace(",1,", ",nan,"),
                    base.replace("12", "-1")):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                telemetry(bad)

    def test_link_writes_ascii_lf_and_reports_partial_transfer(self):
        link = Link.__new__(Link)
        link.port = io.BytesIO()
        link.send("GAIN 1.2500")
        self.assertEqual(link.port.getvalue(), b"GAIN 1.2500\n")
        class ShortPort:
            def write(self, data):
                return len(data) - 1
        link.port = ShortPort()
        with self.assertRaises(OSError):
            link.send("STOP")

    def test_fault_reset_and_reconnect_require_manual_arm(self):
        hb = Heartbeat()
        self.assertIsNone(hb.tick(0.1))
        hb.start(0)
        self.assertEqual(hb.tick(0.11), "PING")
        hb.reply("OK PING", 0.17)
        self.assertEqual(hb.tick(1.0), "STOP")
        hb.reply("OK ARM", 1.1)
        self.assertIsNone(hb.tick(1.3))
        for response in ("ERR ARM", "OK RESET", "OK STOP"):
            hb.start(2)
            hb.reply(response, 2.02)
            self.assertIsNone(hb.tick(2.2))

    def test_tone_requires_manual_start_and_limits_real_cadence(self):
        tone = Tone("pitch", 2, 2, 5)
        self.assertFalse(tone.running)
        tone.start(5)
        self.assertEqual(tone.tick(5), "TARGET 0.000 0.000")
        self.assertIsNone(tone.tick(5.05))  # previous command not acknowledged
        tone.pending = False
        self.assertEqual(tone.tick(5.0625), "TARGET 0.000 1.414")
        with self.assertRaises(ValueError):
            tone.tick(5.3)
        self.assertEqual(tone.tick(10), "STOP")
        for args in [("yaw", 2, 10, 1), ("yaw", 2, 2, 0.7), ("yaw", 0, 2, 10),
                     ("yaw", 36, 1, 10), ("pitch", float("nan"), 1, 10)]:
            with self.assertRaises(ValueError):
                Tone(*args)

    def test_start_tone_requires_recording_active_and_manual_session(self):
        ui = Monitor.__new__(Monitor)
        ui.tone = Tone("yaw", 2, 1, 5)
        ui.log = lambda _: None
        ui.capture = None
        ui.heartbeat = Heartbeat()
        for capture in (None, True):
            ui.capture = capture
            ui.start_tone()
            self.assertFalse(ui.tone.running)

    def test_numeric_commands_enforce_protocol_ranges_before_transmission(self):
        class Entry:
            def __init__(self, text):
                self.text = text
            def get(self):
                return self.text
        sent = []
        ui = Monitor.__new__(Monitor)
        ui.send = lambda name, line: sent.append((name, line)) or True
        ui.log = lambda _: None
        self.assertTrue(ui.numeric_command("usb", "GAIN", [Entry("1.5")], 0, 10))
        self.assertEqual(sent, [("usb", "GAIN 1.5000")])
        for invalid in ("NaN", "-1", "10.01", "abc"):
            self.assertFalse(ui.numeric_command("usb", "GAIN", [Entry(invalid)], 0, 10))
        self.assertFalse(ui.numeric_command("bt", "TARGET", [Entry("35.001"), Entry("0")], -35, 35))
        self.assertEqual(len(sent), 1)


class AnalysisTests(unittest.TestCase):
    def test_step_passing_through_band_does_not_count_as_settled(self):
        t = [i / 10 for i in range(16)]
        commands = [0, 0] + [10] * 14
        measured = [0, 0, 3, 9.7, 12] + [10] * 11
        output = analyze.step(t, commands, measured, "pitch", "small", 0.5)
        self.assertAlmostEqual(output["settling_s"], 0.3)
        self.assertAlmostEqual(output["steady_error_percent"], 0)
        measured[-2] = 11
        self.assertIsNone(analyze.step(t, commands, measured, "pitch", "small", 0.5)["settling_s"])
        with self.assertRaises(ValueError):
            analyze.step(t[:5], commands[:5], measured[:5], "pitch", "small", 0.5)

    def test_static_jitter_unwraps_yaw_and_needs_sixty_seconds(self):
        t = [i/10 for i in range(601)]
        measured = analyze.continuous([179.8 if i % 2 else -179.8 for i in range(601)], "yaw")
        result = analyze.jitter(t, [180] * len(t), measured, "yaw")
        self.assertAlmostEqual(result["peak_to_peak_deg"], 0.4)
        with self.assertRaises(ValueError):
            analyze.jitter(t[:-1], [180]*600, measured[:-1], "yaw")

    def test_tone_fit_gain_phase_and_bandwidth_bracketing(self):
        with tempfile.TemporaryDirectory() as folder:
            points = []
            for f, ratio in ((0.5, 0.8), (2, 0.8*10**(-3/20)), (3, 0.35)):
                path = Path(folder) / f"{f}.csv"
                with path.open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(("ms", "target_pitch", "pitch", "state"))
                    for i in range(401):
                        t = i * 0.02
                        phase = 2*math.pi*f*t
                        writer.writerow((i*20, 1 + 2*math.sin(phase),
                                         -1 + 2*ratio*math.sin(phase - math.pi/6), "ACTIVE"))
                point = analyze.frequency_point(path, f, "pitch")
                points.append(point)
                self.assertAlmostEqual(point["gain"], ratio)
                self.assertAlmostEqual(point["phase_deg"], -30)
                with self.assertRaises(ValueError):
                    analyze.frequency_point(path, 20, "pitch")  # undersampling
            result = analyze.bandwidth(points)
            self.assertAlmostEqual(result["interpolated_minus3db_hz"], 2)
            self.assertIsNone(analyze.bandwidth([points[0], {**points[1], "gain": 0.79}])["crossing_bracket_hz"])

    def test_external_reference_and_repeat_trials(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "reference.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(("trial", "direction", "target_yaw", "target_pitch", "ref_yaw", "ref_pitch"))
                for i in range(20):
                    writer.writerow((i+1, "increasing" if i < 10 else "decreasing", 179, 10, -178, 14))
            self.assertAlmostEqual(analyze.external(path)["max_combined_deg"], 5)
            self.assertAlmostEqual(analyze.external(path, "yaw")["max_abs_error_deg"], 3)
            lines = path.read_text().splitlines()
            path.write_text("\n".join(lines[:-1]))
            with self.assertRaises(ValueError):
                analyze.external(path, "yaw")

    def test_segment_rejects_fault_gaps_and_reboot(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "input.csv"
            for raw in ("0,0,0,FAULT\n10,0,0,ACTIVE\n20,1,0,ACTIVE",
                        "0,0,0,ACTIVE\n10,0,0,ACTIVE\n20,1,0,ACTIVE\n500,1,0,ACTIVE",
                        "20,0,0,ACTIVE\n30,0,0,ACTIVE\n10,1,0,ACTIVE"):
                path.write_text("ms,yaw,target_yaw,state\n" + raw)
                with self.assertRaises(ValueError):
                    analyze.segment(path, "yaw")

    def test_command_line_writes_new_result_and_rejects_bad_csv(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "reference.csv"
            output = Path(folder) / "result.json"
            source.write_text("trial,target_yaw,target_pitch,ref_yaw,ref_pitch\n1,0,0,3,4\n")
            command = [sys.executable, str(HOST / "analyze.py"), "--output", str(output),
                       "absolute", str(source)]
            run = subprocess.run(command, capture_output=True)
            self.assertEqual(run.returncode, 0, run.stderr)
            self.assertIn('"max_combined_deg": 5.0', output.read_text())
            self.assertEqual(subprocess.run(command, capture_output=True).returncode, 2)
            source.write_text("trial,target_yaw,target_pitch,ref_yaw,ref_pitch\n1,0,0\n")
            run = subprocess.run([sys.executable, str(HOST / "analyze.py"), "absolute", str(source)],
                                 capture_output=True)
            self.assertEqual(run.returncode, 2)
            self.assertNotIn(b"Traceback", run.stderr)


if __name__ == "__main__":
    unittest.main()
