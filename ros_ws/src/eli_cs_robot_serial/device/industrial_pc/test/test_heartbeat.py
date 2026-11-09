#!/usr/bin/env python3
"""隔离计时与假应答测试；不连接机器人或现场地址。"""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

DEVICE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(DEVICE/'src'), str(DEVICE/'host_tools')]
from industrial_pc_heartbeat import HeartbeatWatch
from robot_tcp_common import heartbeat
from robot_tcp_common import request


class HeartbeatTest(unittest.TestCase):
    def test_robot_waits_for_first_heartbeat_then_times_out_and_recovers(self):
        now = [0.]
        watch = HeartbeatWatch(10., clock=lambda: now[0])
        now[0] = 100.
        self.assertEqual(watch.snapshot()['state'], 'waiting')
        watch.observe()
        now[0] = 109.9
        self.assertTrue(watch.snapshot()['connected'])
        now[0] = 110.
        self.assertEqual(watch.snapshot()['state'], 'timed_out')
        watch.observe()
        self.assertEqual(watch.snapshot()['age_seconds'], 0.)
        self.assertTrue(watch.snapshot()['connected'])

    def test_host_times_out_even_if_robot_never_responds(self):
        now = [0.]
        watch = HeartbeatWatch(10., clock=lambda: now[0], armed=True)
        now[0] = 10.
        self.assertEqual(watch.snapshot()['state'], 'timed_out')
        self.assertIsNone(watch.snapshot()['age_seconds'])

    def test_wall_clock_change_does_not_change_heartbeat_age(self):
        now = [10.]
        watch = HeartbeatWatch(10., clock=lambda: now[0])
        watch.observe()
        with patch('time.time', return_value=-100000.):
            now[0] = 11.
            self.assertEqual(watch.snapshot()['age_seconds'], 1.)

    def test_heartbeat_correlates_session_sequence_and_server(self):
        reply = dict(ok=True, heartbeat=dict(session_id='listener-1', sequence=3, server_id='server-1'))
        with patch('robot_tcp_common.request', return_value=reply) as request:
            self.assertEqual(heartbeat({}, 'listener-1', 3), reply)
            self.assertEqual(request.call_args.kwargs['retries'], 0)
            self.assertEqual(request.call_args.args[1]['schema'], 'paint_robot_heartbeat_v1')
            for echo in (dict(session_id='previous-session', sequence=3, server_id='server-1'),
                         dict(session_id='listener-1', sequence=2, server_id='server-1')):
                request.return_value = dict(ok=True, heartbeat=echo)
                with self.assertRaises(ValueError):
                    heartbeat({}, 'listener-1', 3)

    def test_invalid_timeout_is_rejected(self):
        for timeout in (0, -1, float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                HeartbeatWatch(timeout)

    def test_business_message_is_not_sent_until_handshake_succeeds(self):
        config = dict(robot_host='127.0.0.1', control_port=1, timeout=1., handshake_enabled=True)
        payload = dict(schema='paint_robot_task_request_v1', message_id='test-only', action='stop')
        with patch('robot_tcp_common.socket.create_connection'), \
                patch('robot_tcp_common.perform_handshake', side_effect=ValueError('no hello')), \
                patch('robot_tcp_common.write_message') as write:
            with self.assertRaisesRegex(ValueError, 'no hello'):
                request(config, payload, retries=0)
            write.assert_not_called()


if __name__ == '__main__':
    unittest.main()
