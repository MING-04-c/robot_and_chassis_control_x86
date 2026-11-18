#!/usr/bin/env python3
"""BMS 电压(V)乘电流(A)取绝对值，发布功率大小(W)并实时绘图。"""
from collections import deque
import math
import signal
from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from masc_chassis_msgs.msg import BmsStatus
from std_msgs.msg import Float64


def calculate_power(voltage, current):
    """输入已换算的 V/A，不再使用 CAN 原始数据的 0.1 比例。"""
    if not math.isfinite(voltage) or voltage <= 0 or not math.isfinite(current):
        raise ValueError('电压必须为正，电压和电流必须是有限数值')
    power = abs(voltage * current)
    if not math.isfinite(power):
        raise ValueError('功率超出有效范围')
    return power


class PowerMonitor(Node):
    def __init__(self):
        super().__init__('power_monitor')
        topic = self.declare_parameter('bms_topic', '/masc_chassis/bms_status').value
        output = self.declare_parameter('power_topic', '/power/watts').value
        self.window = float(self.declare_parameter('window_seconds', 120.0).value)
        self.timeout = float(self.declare_parameter('stale_timeout', 2.0).value)
        self.show_plot = self.declare_parameter('show_plot', True).value
        if not all(math.isfinite(x) and x > 0 for x in (self.window, self.timeout)):
            raise ValueError('window_seconds 和 stale_timeout 必须为正数')
        self.samples = deque(maxlen=10000)
        self.started = monotonic()
        self.received = None
        self.latest = None
        self.valid = False
        self.publisher = self.create_publisher(Float64, output, 10)
        self.subscription = self.create_subscription(
            BmsStatus, topic, self.on_bms, qos_profile_sensor_data)
        self.figure = None
        if self.show_plot:
            import matplotlib.pyplot as plt
            self.plt = plt
            self.figure, self.axes = plt.subplots(figsize=(11, 5))
            if not getattr(self.figure.canvas, 'required_interactive_framework', None):
                plt.close(self.figure)
                raise RuntimeError('需要图形桌面显示折线图；无桌面请用 show_plot:=false')
            self.figure.canvas.manager.set_window_title('Battery Power Monitor')
            self.line, = self.axes.plot([], [], color='#2479d8', linewidth=1.8)
            self.axes.set_xlabel('Elapsed time (s)')
            self.axes.set_ylabel('Power (W), P = |U x I|')
            self.axes.axhline(0, color='gray', linewidth=0.8)
            self.axes.grid(True, alpha=0.3)
            self.title = self.axes.set_title('Waiting for BMS data')
            self.figure.text(0.5, 0.01, 'Power magnitude |U x I|; current retains its original sign', ha='center')
            self.figure.tight_layout(rect=[0, 0.04, 1, 1])
            plt.show(block=False)
        self.get_logger().info('功率监控：%s -> %s；P=|U(V)×I(A)|，单位W，电流显示保留原始符号' % (topic, output))

    def on_bms(self, message):
        stamp = message.header.stamp
        age = (self.get_clock().now().nanoseconds - (stamp.sec * 10**9 + stamp.nanosec)) / 1e9
        if message.bms_health == BmsStatus.BMS_HEALTH_TIMEOUT or not -0.5 <= age <= self.timeout:
            self.invalidate()
            return
        try:
            power = calculate_power(message.bat_voltage, message.bat_current)
        except ValueError:
            self.invalidate()
            return
        now = monotonic()
        if self.received is not None and now - self.received > self.timeout:
            self.samples.append((now - self.started, math.nan))
        self.received, self.valid = now, True
        self.latest = (message.bat_voltage, message.bat_current, power)
        self.samples.append((now - self.started, power))
        while self.samples and self.samples[0][0] < now - self.started - self.window:
            self.samples.popleft()
        self.publisher.publish(Float64(data=power))

    def invalidate(self):
        if self.valid:
            self.samples.append((monotonic() - self.started, math.nan))
        self.valid = False

    def update_plot(self):
        if self.received is not None and monotonic() - self.received > self.timeout:
            self.invalidate()
        if self.figure is None:
            return
        elapsed = monotonic() - self.started
        while self.samples and self.samples[0][0] < elapsed - self.window:
            self.samples.popleft()
        values = list(self.samples)
        self.line.set_data([x[0] for x in values], [x[1] for x in values])
        self.axes.relim()
        self.axes.autoscale_view(scalex=False)
        self.axes.set_xlim(max(0, elapsed - self.window), max(self.window, elapsed))
        if self.valid and self.latest is not None:
            voltage, current, power = self.latest
            self.title.set_text(f'Live | U={voltage:.1f} V | I={current:+.1f} A | Power={power:.2f} W')
            self.title.set_color('black')
        else:
            self.title.set_text('BMS data unavailable / stale' if self.received is not None else 'Waiting for valid BMS data')
            self.title.set_color('#ba3030')
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = [False]
    def stop(_signum, _frame):
        stopping[0] = True
    previous = {s: signal.signal(s, stop) for s in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = PowerMonitor()
        last_draw = 0.0
        while rclpy.ok() and not stopping[0]:
            if node.figure is not None and not node.plt.fignum_exists(node.figure.number):
                break
            rclpy.spin_once(node, timeout_sec=0.02)
            if monotonic() - last_draw >= 0.1:
                node.update_plot()
                last_draw = monotonic()
            if node.figure is not None:
                node.plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.figure is not None:
                node.plt.close(node.figure)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    main()
