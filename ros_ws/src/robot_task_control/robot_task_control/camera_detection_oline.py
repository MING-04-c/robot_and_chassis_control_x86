"""Host-paced photo/detect cycles, reusing the real guarded detector unchanged."""
import json
from time import monotonic
from uuid import uuid4

from std_msgs.msg import String
from std_srvs.srv import Trigger

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
            self.phase = 'IDLE'
        return super().start_request(request, response)

    def detect_request(self, request, response):
        del request
        if self.phase not in ('WAIT_DETECT', 'WAIT_NEXT'):
            response.success, response.message = False, '当前阶段不能开始检测：'+self.phase
            return response
        if not self.ready():
            response.success, response.message = False, '真机、力反馈或检测接口尚未就绪'
            return response
        self.detection_requested = True
        if self.phase == 'WAIT_DETECT':
            self.results.clear()
            self.transition('WAIT_RESULT', '已收到检测指令；等待本次已保存照片的坐标')
        else:
            self.reset_cycle(True)
            self.transition('WAIT_READY', '新一轮检测：重新稳定拍照、发送并等待新坐标')
        response.success, response.message = True, '检测请求已接受；请根据当前照片批次回传坐标'
        return response

    def go_home_request(self, request, response):
        del request
        if self.phase not in ('WAIT_DETECT', 'WAIT_NEXT'):
            response.success, response.message = False, '请等待照片保存或本轮检测回位完成后再发回家；当前：'+self.phase
            return response
        if not self.ready():
            response.success, response.message = False, '运动接口未就绪，不放行返回路径'
            return response
        self.home_requested = True
        self.transition('DONE', '收到回家指令；放行倒序中间点、过渡位、行车位及小车返程')
        response.success, response.message = True, '回家已接受；实际完成请查询任务状态'
        return response

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
        if self.phase in ('WAIT_SAVED', 'WAIT_DETECT', 'WAIT_NEXT'):
            self.check_hardware()
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
            if self.phase not in ('IDLE', 'FAULT', 'DONE'):
                self.advance()
        except Exception as error:
            self.fail(str(error))
        self.publish_oline_status()

    def transition(self, phase, detail):
        # A detector batch finishing must never publish session DONE; only the
        # explicit host home command is allowed to release the return workflow.
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
            'detect_state': self.detect.get('state'),
            'capture_manifest_sha256': (self.snapshot or {}).get('capture_manifest_sha256'),
            'capture_saved_by_host': (self.snapshot or {}).get('host_saved', False),
            'completed_points': self.detect.get('completed_points', 0),
        }, ensure_ascii=False)))


def main():
    # Retain the shared bounded SIGINT/SIGTERM cancellation/cleanup implementation.
    from robot_task_control import camera_detection_task
    camera_detection_task.main(node_factory=CameraDetectionOline)
