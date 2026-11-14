"""Host-paced photo/detect cycles, reusing the real guarded detector unchanged."""
import json
import math
from time import monotonic
from uuid import uuid4

from std_msgs.msg import String
from std_srvs.srv import Trigger
from masc_chassis_msgs.msg import BmsStatus, CommStatus

from robot_task_control.camera_detection_task import CameraDetectionTask


class CameraDetectionOline(CameraDetectionTask):
    def __init__(self):
        super().__init__()
        if self.target_source != 'tcp' or self.auto_start:
            raise ValueError('oline requires TCP points and workflow-triggered start')
        self.detection_requested = False
        self.home_requested = False
        self.completed_cycles = 0
        self.last_cycle_error = ''
        self.low_battery_enabled = bool(self.declare_parameter('low_battery_return_enabled', False).value)
        self.low_battery_threshold = float(self.declare_parameter('low_battery_threshold', 10.0).value)
        if not math.isfinite(self.low_battery_threshold) or not 0 < self.low_battery_threshold <= 100:
            raise ValueError('low_battery_threshold must be in (0,100]')
        self.power_samples = {}
        self.battery_percent = None
        self.low_battery_return_pending = False
        self.return_reason = ''
        self.create_subscription(BmsStatus, '/masc_chassis/bms_status',
                                 lambda msg: self.cache_power('bms', msg), 10)
        self.create_subscription(CommStatus, '/masc_chassis/comm_status',
                                 lambda msg: self.cache_power('comm', msg), 10)
        self.create_service(Trigger, '~/detect', self.detect_request)
        self.create_service(Trigger, '~/go_home', self.go_home_request)
        self.create_subscription(String, '/industrial_pc/capture_saved', self.on_capture_saved, 10)

    def reset_cycle(self, requested):
        self.stop_capture()
        self.run_id = uuid4().hex
        self.retry_count = 0
        self.detection_requested = requested
        self.home_requested = False
        self.started_detection = False
        self.results.clear()
        self.point_ids = []
        self.targets = []
        self.snapshot = None
        self.holds = 0
        self.ready_since = None
        self.expected_point_ids.clear()
        self.completed_point_ids.clear()
        self.return_events.clear()
        self.completion_wait_started = None
        self.last_cycle_error = ''
        self.operation = None

    def start_request(self, request, response):
        if self.phase == 'DONE':
            self.reset_cycle(False)
            self.root_run_id = self.run_id
            self.home_recorded = False
            self.completed_cycles = 0
            self.low_battery_return_pending = False
            self.return_reason = ''
            self.phase = 'IDLE'
        return super().start_request(request, response)

    def detect_request(self, request, response):
        del request
        if self.phase not in ('WAIT_DETECT', 'WAIT_NEXT'):
            response.success, response.message = False, '当前阶段不能开始检测：'+self.phase
            return response
        self.update_battery()
        if self.low_battery_return_pending:
            response.success, response.message = False, '电量低于阈值，已锁定返回充电，不再开始新检测'
            return response
        if not self.ready():
            response.success, response.message = False, '真机、力反馈或检测接口尚未就绪'
            return response
        self.reset_cycle(True)
        self.transition('WAIT_READY', '收到检测指令：重新稳定拍照、发送并等待本轮新坐标')
        response.success, response.message = True, '检测请求已接受；请根据当前照片批次回传坐标'
        return response

    def go_home_request(self, request, response):
        del request
        if self.phase not in ('WAIT_DETECT', 'WAIT_NEXT'):
            response.success, response.message = False, '请在到位等待或本轮检测回位完成后发回家；当前：'+self.phase
            return response
        if not self.ready():
            response.success, response.message = False, '运动接口未就绪，不放行返回路径'
            return response
        self.home_requested = True
        self.return_reason = 'go_home'
        self.transition('DONE', '收到回家指令；放行外层返程流程（photo入口保持拍照位）')
        response.success, response.message = True, '回家已接受；实际完成请查询任务状态'
        return response

    def begin_capture(self):
        # Arriving at the photo pose starts a session, not a capture. Every detect
        # command, including the first one, obtains a fresh capture identity.
        if not self.detection_requested:
            self.transition('WAIT_DETECT', '已到拍照位；等待 detect 开始拍照传输，或 go_home 返回')
            return
        super().begin_capture()

    def cache_power(self, name, message):
        self.power_samples[name] = (message, monotonic())

    def update_battery(self):
        def fresh(name):
            message, received = self.power_samples.get(name, (None, -math.inf))
            if message is None or monotonic()-received >= 3.:
                return None
            stamp = message.header.stamp
            stamp_ns = stamp.sec*10**9+stamp.nanosec
            age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
            return message if stamp_ns > 0 and -.1 <= age < 3. else None
        bms, comm = fresh('bms'), fresh('comm')
        valid = (bms is not None and comm is not None
                 and comm.comm_state == CommStatus.COMM_STATE_CONNECTED
                 and 0 <= bms.bat_soc <= 100
                 and math.isfinite(bms.bat_voltage) and math.isfinite(bms.bat_current)
                 and bms.bms_health not in (BmsStatus.BMS_HEALTH_TIMEOUT, BmsStatus.BMS_HEALTH_UNKNOWN))
        self.battery_percent = int(bms.bat_soc) if valid else None
        if (self.low_battery_enabled and valid and self.battery_percent < self.low_battery_threshold
                and self.phase not in ('DONE', 'FAULT')):
            self.low_battery_return_pending = True

    def return_if_safe(self):
        if not self.low_battery_return_pending:
            return False
        # STARTING/RUNNING may already have launched physical motion. Let the
        # guarded detector finish and confirm photo-home before releasing return.
        safe = ('WAIT_READY', 'WAIT_DETECT', 'WAIT_NEXT', 'CAPTURE', 'WAIT_SAVED',
                'WAIT_RESULT', 'CLEAR_TARGETS', 'RECORD_HOME', 'WAIT_TARGETS')
        if self.phase not in safe or not self.ready():
            return False
        if self.operation is not None and not self.operation.done():
            return False
        self.stop_capture()
        self.results.clear()
        self.operation = None
        self.home_requested = True
        self.return_reason = 'low_battery'
        self.transition('DONE', f'电量低于{self.low_battery_threshold:g}%；已在拍照原位，收臂后沿标准路线返回充电桩')
        return True

    def after_capture(self):
        self.transition('WAIT_SAVED', '照片已发送；等待麒麟确认本批HK、RGB、深度图校验保存完成')

    def on_capture_saved(self, message):
        if self.phase != 'WAIT_SAVED':
            return
        try:
            payload = json.loads(message.data)
            if (payload['capture_set_id'] != self.run_id or
                    payload['capture_manifest_sha256'] != self.snapshot['capture_manifest_sha256']):
                return
            index = json.loads((self.storage/'.capture_index.json').read_text())
            files = index['sessions'][self.run_id]['files']
            if any(payload['files_sha256'].get(name) != files[name]
                   for name in ('hk.jpg', 'rgb.png', 'depth.png')):
                raise ValueError('麒麟保存的三图哈希与本机采集不一致')
            self.snapshot['host_saved'] = True
            self.transition('WAIT_RESULT' if self.detection_requested else 'WAIT_DETECT',
                            '麒麟已确认三图保存；等待本批坐标' if self.detection_requested
                            else '麒麟已确认三图保存；等待检测或回家指令')
        except (ValueError, KeyError, TypeError, OSError) as error:
            self.fail('照片保存确认失败：'+str(error))

    def schedule_reacquire(self, reason):
        # The base calls this only before motion or after confirmed recovery/home.
        self.started_detection = False
        self.detection_requested = False
        self.last_cycle_error = str(reason)
        self.transition('WAIT_NEXT', '本轮未完成且已在拍照原位；等待主机重新检测或回家：'+str(reason))

    def advance(self):
        self.check_hardware()
        if self.return_if_safe():
            return
        if self.phase in ('WAIT_SAVED', 'WAIT_DETECT', 'WAIT_NEXT'):
            if not self.connected() or monotonic()-self.detect_at > 2.:
                raise ValueError('等待主机期间真机外控或检测状态断流')
            if self.detect.get('state') == 'FAULT':
                raise ValueError(self.detect.get('message', '检测器故障'))
            if self.phase in ('WAIT_SAVED', 'WAIT_DETECT'):
                self.check_stationary()
            if self.phase == 'WAIT_SAVED' and monotonic()-self.since > self.timeouts['capture']:
                raise ValueError('等待麒麟确认三图保存超时')
            return
        super().advance()

    def tick(self):
        try:
            self.update_battery()
            if self.phase not in ('IDLE', 'FAULT', 'DONE'):
                self.advance()
        except Exception as error:
            self.fail(str(error))
        self.publish_oline_status()

    def transition(self, phase, detail):
        # A detector batch finishing must never publish session DONE; only the
        # explicit host home command or latched low-power return releases it.
        if phase == 'DONE' and not self.home_requested:
            self.completed_cycles += 1
            self.detection_requested = False
            phase, detail = 'WAIT_NEXT', '本轮检测完成且已回拍照原位；等待再次检测或回家'
        super().transition(phase, detail)
        self.publish_oline_status()

    def publish_oline_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'workflow': 'camera_detection_oline', 'oline_mode': True,
            'state': self.phase, 'phase': self.phase, 'message': self.detail,
            'active': self.phase not in ('IDLE', 'DONE', 'FAULT'),
            'run_id': self.run_id, 'root_run_id': self.root_run_id,
            'completed_cycles': self.completed_cycles, 'last_cycle_error': self.last_cycle_error,
            'mode': 'online', 'battery_percent': self.battery_percent,
            'low_battery_return_enabled': self.low_battery_enabled,
            'low_battery_return_pending': self.low_battery_return_pending,
            'return_reason': self.return_reason,
            'detect_state': self.detect.get('state'),
            'capture_manifest_sha256': (self.snapshot or {}).get('capture_manifest_sha256'),
            'capture_saved_by_host': (self.snapshot or {}).get('host_saved', False),
            'completed_points': self.detect.get('completed_points', 0),
        }, ensure_ascii=False)))


def main():
    # Retain the shared bounded SIGINT/SIGTERM cancellation/cleanup implementation.
    from robot_task_control import camera_detection_task
    camera_detection_task.main(node_factory=CameraDetectionOline)
