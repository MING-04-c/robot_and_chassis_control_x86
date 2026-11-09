"""双端心跳计时；使用单调时钟，不依赖两台电脑校时。"""
import math
import time


class HeartbeatWatch:
    def __init__(self, timeout=10., clock=time.monotonic, armed=False):
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('heartbeat_timeout 必须为有限正数')
        self.timeout = float(timeout)
        self.clock = clock
        self.started = clock()
        self.received = None
        self.armed = armed

    def observe(self):
        self.received = self.clock()
        self.armed = True

    def snapshot(self):
        age = None if self.received is None else max(0., self.clock()-self.received)
        elapsed = max(0., self.clock()-self.started) if age is None else age
        expired = self.armed and elapsed >= self.timeout
        state = 'timed_out' if expired else ('connected' if age is not None else 'waiting')
        return dict(state=state, connected=state == 'connected', monitored=self.armed,
                    age_seconds=age, timeout_seconds=self.timeout)
