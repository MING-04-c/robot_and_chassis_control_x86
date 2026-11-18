"""验证单位、符号和无效BMS反馈不会产生假功率。"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from collections import deque
from unittest.mock import Mock
import unittest

from masc_chassis_msgs.msg import BmsStatus

spec = importlib.util.spec_from_file_location(
    'power_monitor', Path(__file__).resolve().parents[1] / 'src/power_monitor.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PowerTest(unittest.TestCase):
    def test_si_units_and_signed_current(self):
        self.assertAlmostEqual(module.calculate_power(48.9, -1.5), 73.35)
        self.assertAlmostEqual(module.calculate_power(50.0, 2.0), 100.0)
        self.assertEqual(module.calculate_power(50.0, 0.0), 0.0)
        for voltage, current in ((0, 1), (-1, 1), (float('nan'), 1), (50, float('inf'))):
            with self.assertRaises(ValueError):
                module.calculate_power(voltage, current)

    def test_timeout_and_stale_stamp_do_not_publish(self):
        clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
        node = SimpleNamespace(
            get_clock=lambda: clock, timeout=2.0, window=120.0,
            samples=deque(), started=module.monotonic(), received=None,
            valid=False, latest=None, publisher=Mock())
        node.invalidate = lambda: module.PowerMonitor.invalidate(node)
        message = BmsStatus(bat_voltage=48.9, bat_current=-1.5)
        message.header.stamp.sec = 10
        module.PowerMonitor.on_bms(node, message)
        self.assertAlmostEqual(node.publisher.publish.call_args.args[0].data, 73.35)
        message.bms_health = BmsStatus.BMS_HEALTH_TIMEOUT
        module.PowerMonitor.on_bms(node, message)
        self.assertFalse(node.valid)
        self.assertEqual(node.publisher.publish.call_count, 1)
        message.bms_health = BmsStatus.BMS_HEALTH_GOOD
        message.header.stamp.sec = 7
        module.PowerMonitor.on_bms(node, message)
        self.assertEqual(node.publisher.publish.call_count, 1)


if __name__ == '__main__':
    unittest.main()
