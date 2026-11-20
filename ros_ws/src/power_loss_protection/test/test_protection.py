"""Offline tests: no UPS access, ROS motion, service mutations or shutdown."""
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import modbus
import power_loss_protection as guard


def sample(backup=False):
    return dict(main=not backup, backup=backup, status=2 if backup else 1)


class FakeSocket:
    def __init__(self, response):
        self.response, self.sent = response, b""

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def settimeout(self, timeout):
        pass

    def sendall(self, data):
        self.sent += data

    def recv(self, size):
        # Fragment every response to exercise TCP stream reassembly.
        result, self.response = self.response[:1], self.response[1:]
        return result


class ProtocolTests(unittest.TestCase):
    def test_crc(self):
        self.assertEqual(struct.pack("<H", modbus.crc16(bytes.fromhex("01 03 00 00 00 01"))),
                         bytes.fromhex("84 0a"))

    def read(self, response, transport):
        sock = FakeSocket(response)
        with patch.object(modbus.socket, "create_connection", return_value=sock):
            value = modbus.read_registers("127.0.0.1", 502, 1, transport, .5, count=2)
        return value, sock.sent

    def test_rtu_stream_and_request(self):
        frame = bytes.fromhex("01 03 04 5d c0 55 f0")
        values, sent = self.read(frame+struct.pack("<H", modbus.crc16(frame)), "rtu_over_tcp")
        self.assertEqual(values, [24000, 22000])
        self.assertEqual(sent[:6], bytes.fromhex("01 03 00 00 00 02"))

    def test_tcp_stream_and_request(self):
        values, sent = self.read(bytes.fromhex("00 01 00 00 00 07 01 03 04 5d c0 55 f0"), "modbus_tcp")
        self.assertEqual(values, [24000, 22000])
        self.assertEqual(sent, bytes.fromhex("00 01 00 00 00 06 01 03 00 00 00 02"))

    def test_bad_crc(self):
        with self.assertRaises(modbus.ProtocolError):
            self.read(bytes.fromhex("01 03 04 5d c0 55 f0 00 00"), "rtu_over_tcp")

    def test_wrong_transaction(self):
        with self.assertRaises(modbus.ProtocolError):
            self.read(bytes.fromhex("00 02 00 00 00 07 01 03 04 5d c0 55 f0"), "modbus_tcp")

    def test_wrong_unit(self):
        with self.assertRaises(modbus.ProtocolError):
            self.read(bytes.fromhex("02 03 04"), "rtu_over_tcp")

    def test_truncated(self):
        with self.assertRaises(ConnectionError):
            self.read(bytes.fromhex("01 03 04 5d"), "rtu_over_tcp")

    def test_tcp_exception(self):
        with self.assertRaisesRegex(modbus.ProtocolError, "exception 2"):
            self.read(bytes.fromhex("00 01 00 00 00 03 01 83 02"), "modbus_tcp")

    def test_slow_stream_has_one_deadline(self):
        sock = FakeSocket(b"abc")
        with patch.object(modbus.time, "monotonic", side_effect=[.1, .6]):
            with self.assertRaises(TimeoutError):
                modbus.recv_exact(sock, 3, .5)

    def test_rtu_exception_crc(self):
        frame = bytes.fromhex("01 83 02")
        with self.assertRaisesRegex(modbus.ProtocolError, "exception 2"):
            self.read(frame+struct.pack("<H", modbus.crc16(frame)), "rtu_over_tcp")

    def test_tcp_wrong_length(self):
        with self.assertRaises(modbus.ProtocolError):
            self.read(bytes.fromhex("00 01 00 00 ff ff 01"), "modbus_tcp")

    def test_ambiguous_state_rejected(self):
        with self.assertRaises(modbus.ProtocolError):
            modbus.telemetry([24000, 24000, 24000, 200, 1000, 100, 3, 0, 0x344])

    def test_firmware_and_backup(self):
        result = modbus.telemetry([0, 24000, 24000, 200, 1000, 100, 2, 0, 0x344])
        self.assertTrue(result["backup"])
        self.assertEqual(result["firmware"], "0x0344")

    def test_display_volts_and_amps_preserves_raw_sample(self):
        raw = modbus.telemetry([24112, 23155, 23983, 200, 1500, 95, 1, 1, 0x344])
        displayed = modbus.display_telemetry(raw)
        self.assertEqual(displayed["vmain_v"], 24.112)
        self.assertEqual(displayed["vcap_v"], 23.155)
        self.assertEqual(displayed["vout_v"], 23.983)
        self.assertEqual(displayed["iout_a"], 1.5)
        self.assertNotIn("iout_ma", displayed)
        self.assertEqual(raw["iout_ma"], 1500)
        self.assertEqual(displayed["status"], raw["status"])


class DecisionTests(unittest.TestCase):
    def test_backup_confirm_and_latch(self):
        d = guard.Decision(0, 3, 3)
        self.assertIsNone(d.update(.2, sample(True)))
        self.assertIsNone(d.update(.4, sample(True)))
        self.assertEqual(d.update(.6, sample(True)), "UPS BACKUP confirmed")
        self.assertEqual(d.update(.8, sample()), "UPS BACKUP confirmed")

    def test_transient_recovery(self):
        d = guard.Decision(0, 3, 3)
        d.update(.2, sample(True))
        d.update(.4, sample())
        d.update(.6, sample(True))
        self.assertIsNone(d.update(.8, sample(True)))

    def test_error_breaks_confirmation(self):
        d = guard.Decision(0, 3, 3)
        d.update(.2, sample(True))
        d.update(.4, sample(True))
        d.update(.6)
        self.assertIsNone(d.update(.8, sample(True)))

    def test_no_data_from_startup(self):
        d = guard.Decision(0, 3, 3)
        self.assertIsNone(d.update(2.9))
        self.assertEqual(d.update(3.), "UPS communication unavailable")

    def test_late_good_frame_does_not_hide_gap(self):
        d = guard.Decision(0, 3, 3)
        self.assertEqual(d.update(3.1, sample()), "UPS communication unavailable")

    def test_configured_loss_threshold_is_one_and_half_seconds(self):
        for filename in ("config.json", "config.example.json"):
            config = guard.load_config(ROOT/filename)
            self.assertEqual(config["communication_loss_seconds"], 1.5)
            d = guard.Decision(0, config["confirm_count"], config["communication_loss_seconds"])
            d.update(.5, sample())
            self.assertIsNone(d.update(1.))
            self.assertIsNone(d.update(1.5))
            self.assertEqual(d.update(2.), "UPS communication unavailable")

    def test_standby_immediate(self):
        d = guard.Decision(0, 3, 3)
        self.assertEqual(d.update(.1, dict(main=False, backup=False, status=4)), "UPS output state unsafe")


class ShutdownTests(unittest.TestCase):
    def setUp(self):
        self.c = guard.load_config(ROOT/"config.example.json")

    def test_observe_has_no_side_effects(self):
        with patch.object(guard, "run_bounded") as run, patch.object(guard, "atomic_json") as write:
            guard.shutdown(self.c, "test")
            run.assert_not_called()
            write.assert_not_called()

    def test_formal_entry_uses_armed_production_config(self):
        with patch.object(sys, "argv", ["guard", "run"]), \
                patch.object(guard, "monitor") as monitor:
            guard.main()
        config = monitor.call_args.args[0]
        self.assertEqual(config["mode"], "armed")
        self.assertEqual(config["computer_power_source"], "external")
        self.assertEqual(config["latch_file"], "/var/lib/power-loss-protection/shutdown.json")

    def test_armed_monitor_requires_root(self):
        self.c["mode"] = "armed"
        with patch.object(guard.os, "geteuid", return_value=1000):
            with self.assertRaises(PermissionError):
                guard.monitor(self.c)

    def test_formal_run_rejects_observe_config(self):
        with patch.object(sys, "argv", ["guard", "run", "--config", str(ROOT/"config.example.json")]), \
                patch.object(guard, "monitor") as monitor:
            with self.assertRaisesRegex(ValueError, "mode=armed"):
                guard.main()
            monitor.assert_not_called()

    def test_armed_requires_measured_budget(self):
        self.c["mode"] = "armed"
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"config.json"
            path.write_text(json.dumps(self.c))
            with self.assertRaisesRegex(ValueError, "Measure backup"):
                guard.load_config(path)

    def test_shutdown_after_stop_failure_and_lock_created_first(self):
        self.c["mode"] = "armed"
        with tempfile.TemporaryDirectory() as tmp:
            self.c["latch_file"] = str(Path(tmp)/"latch.json")
            self.c["ready_file"] = str(Path(tmp)/"ready.json")
            calls = []
            def run(argv, seconds):
                self.assertTrue(Path(self.c["latch_file"]).exists())
                calls.append(argv)
                if "poweroff" in argv:
                    return 0
                raise TimeoutError("Simulated stop hang")
            with patch.object(guard, "run_bounded", side_effect=run):
                with self.assertLogs(guard.LOG, level="ERROR"):
                    guard.shutdown(self.c, "test")
            self.assertIn("poweroff", calls[-1])

    def test_poweroff_retry_is_bounded(self):
        self.c["mode"] = "armed"
        with tempfile.TemporaryDirectory() as tmp:
            self.c["latch_file"] = str(Path(tmp)/"latch.json")
            self.c["ready_file"] = str(Path(tmp)/"ready.json")
            calls = []
            def run(argv, seconds):
                calls.append(argv)
                return 1
            with patch.object(guard, "run_bounded", side_effect=run):
                with self.assertLogs(guard.LOG, level="ERROR"):
                    with self.assertRaisesRegex(RuntimeError, "not accepted"):
                        guard.shutdown(self.c, "test")
            self.assertEqual(sum("poweroff" in argv for argv in calls), 3)

    def test_readiness_requires_fresh_data_and_no_latch(self):
        self.c["mode"] = "armed"
        with tempfile.TemporaryDirectory() as tmp:
            self.c["latch_file"] = str(Path(tmp)/"latch.json")
            self.c["ready_file"] = str(Path(tmp)/"ready.json")
            guard.atomic_json(self.c["ready_file"], dict(main=True, monotonic=10))
            with patch.object(guard.time, "monotonic", return_value=11):
                self.assertTrue(guard.can_start(self.c))
                Path(self.c["latch_file"]).write_text("corrupt latch")
                self.assertFalse(guard.can_start(self.c))
            Path(self.c["latch_file"]).unlink()
            with patch.object(guard.time, "monotonic", return_value=14):
                self.assertFalse(guard.can_start(self.c))


if __name__ == "__main__":
    unittest.main()
