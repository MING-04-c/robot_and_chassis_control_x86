"""Real localhost TCP/ROS; synthetic photos and detector, no hardware drivers."""
import hashlib
import json
import os
from pathlib import Path
import socket
import sys
import threading
import time
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
import yaml

DEVICE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(DEVICE/'src'), str(DEVICE/'host_tools'), str(DEVICE.parents[2]/'robot_task_control'),
               str(DEVICE.parents[2]/'geminipro_camer_calibration/scripts')]
os.environ['ROS_DOMAIN_ID'] = '193'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOG_DIR'] = '/tmp/oline-session-tests'

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from masc_chassis_msgs.msg import BmsStatus, ChassisStatus, CommStatus

import capture_storage as storage
from industrial_pc_node import IndustrialPC
from industrial_pc_protocol import validate_command, ProtocolError
from listen_robot import RobotListener
from robot_tcp_common import request, status
from runtime_monitor import RuntimeMonitor
from robot_task_control.camera_detection_task import CameraDetectionTask
from robot_task_control.camera_detection_oline import CameraDetectionOline
from robot_task_control.final_capture import load_config
from robot_task_control.workflow_controller import WorkflowController


def test_oline_commands_and_saved_receipt_have_strict_fields():
    for action in ('start', 'detect', 'go_home', 'stop'):
        assert validate_command(dict(schema='paint_robot_task_request_v1', message_id='test', action=action)) == action
    packet = dict(schema='paint_robot_capture_saved_v1', message_id='save-test', capture_set_id='photo',
                  capture_manifest_sha256='a'*64, files_sha256={n: 'b'*64 for n in ('hk.jpg', 'rgb.png', 'depth.png')})
    assert validate_command(packet) == 'capture_saved'
    packet['files_sha256'].pop('hk.jpg')
    with pytest.raises(ProtocolError):
        validate_command(packet)


def test_camera_error_and_waiting_state_are_exported():
    monitor = RuntimeMonitor()
    monitor.receive('task', dict(active=True, phase='camera_detection_running', action='camera_detection'), now=1.)
    monitor.receive('camera_detection', dict(state='WAIT_NEXT', last_cycle_error='测厚无回包'), now=1.)
    normal, abnormal = monitor.snapshot(now=1.1)
    assert {x['error_code'] for x in abnormal['active_faults']} == {'E5002'}
    monitor.receive('camera_detection', dict(state='WAIT_NEXT', last_cycle_error=''), now=1.2)
    normal, abnormal = monitor.snapshot(now=1.3)
    assert normal['state'] == 'waiting' and not abnormal['has_fault']


def test_two_remote_cycles_wait_for_home_and_reject_old_photos(tmp_path):
    recipe = DEVICE.parents[2]/'robot_task_control/config/robot_task.yaml'
    capture_config = load_config(recipe)
    capture_config.update(output_dir=str(tmp_path/'robot-images'), local_only=False)
    capture_file = tmp_path/'capture.yaml'
    capture_file.write_text(yaml.safe_dump(capture_config))
    workflow_file = tmp_path/'workflow.yaml'
    workflow_file.write_text(yaml.safe_dump({'workflow': {
        'name': 'test_oline', 'auto_start': False, 'remote_detection_cycles': True,
        'steps': [{'action': 'delay', 'seconds': 0.}, {'action': 'camera_detection', 'step_timeout_sec': 30.},
                  {'action': 'delay', 'seconds': 0.}],
    }}))
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    rclpy.init(args=['--ros-args', '-p', 'bind_host:=127.0.0.1', '-p', 'port:='+str(port),
        '-p', 'oline_mode:=true', '-p', 'auto_start:=false', '-p', 'ready_stable_sec:=0.0',
        '-p', 'status_topic:=/camera_detection_task/status', '-p', 'stop_service:=/camera_detection_task/stop',
        '-p', 'charging_status_layout:=byte_flags', '-p', 'target_source:=tcp',
        '-p', 'workflow_file:='+str(workflow_file), '-p', 'capture_config_file:='+str(capture_file),
        '-p', 'output_dir:='+str(tmp_path/'bridge'), '-p', 'report_directory:='+str(tmp_path/'reports')])
    client = dict(robot_host='127.0.0.1', control_port=port, timeout=3., use_shared_key=False, handshake_enabled=True)
    host = tmp_path/'host'
    captures, starts, homes, targets = [], [], [], []
    running = [None]
    def fake_capture(command, **kwargs):
        identity = command[command.index('--run-id')+1]
        captures.append(identity)
        digest = hashlib.sha256(identity.encode()).hexdigest()
        for root in (tmp_path/'robot-images', host):
            with storage.index(root) as index:
                record = storage.session(root, index, identity)
                for name in ('hk.jpg', 'rgb.png', 'depth.png'):
                    storage.store_file(root, record, name, payload=(identity+name).encode())
                record['rgbd'] = {'capture_manifest_sha256': digest}
        process = MagicMock()
        process.poll.return_value = process.returncode = 0
        return process
    def pose(node, frame):
        result = np.eye(4)
        if frame == node.camera:
            result[0, 3] = .2
        return result
    executor, thread = SingleThreadedExecutor(), None
    nodes = []
    listener = None
    try:
        with patch.object(CameraDetectionTask, 'check_hardware', lambda _: None), \
                patch.object(CameraDetectionTask, 'connected', lambda _: True), \
                patch.object(CameraDetectionTask, 'ready', lambda _: True), \
                patch.object(CameraDetectionTask, 'pose', pose), \
                patch('robot_task_control.camera_detection_task.subprocess.Popen', side_effect=fake_capture):
            task, bridge, workflow, detector = CameraDetectionOline(), IndustrialPC(), WorkflowController(), Node('oline_detector_fixture')
            nodes = [task, bridge, workflow, detector]
            def success(req, response):
                response.success, response.message = True, 'synthetic detector'
                return response
            def record(req, response):
                homes.append(True)
                return success(req, response)
            def begin(req, response):
                starts.append(task.run_id)
                running[0] = time.monotonic()
                return success(req, response)
            detector.create_service(Trigger, '/detect_controller/clear_targets', success)
            detector.create_service(Trigger, '/detect_controller/record_home', record)
            detector.create_service(Trigger, '/detect_controller/start', begin)
            detector.create_service(Trigger, '/detect_controller/cancel', success)
            detector.create_subscription(PoseArray, '/geminipro_calibration/targets_base', targets.append, 10)
            pub = detector.create_publisher(String, '/detect_controller/status', 10)
            points = detector.create_publisher(String, '/detect_controller/point_result',
                QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            power_pubs = [detector.create_publisher(cls, '/masc_chassis/'+name, 10) for cls, name in (
                (BmsStatus, 'bms_status'), (ChassisStatus, 'chassis_status'), (CommStatus, 'comm_status'))]
            def broadcast():
                elapsed = -1. if running[0] is None else time.monotonic()-running[0]
                state = 'IDLE' if elapsed < 0 else 'HOLD' if elapsed < .8 else 'DONE'
                batch = 'batch-'+str(len(starts))
                pub.publish(String(data=json.dumps(dict(state=state, batch_id=batch, motion_ready=True,
                    returned_home=state == 'DONE', queued_points=len(targets[-1].poses) if targets else 0,
                    completed_points=1 if state == 'DONE' else 0))))
                if state == 'DONE':
                    stamp = detector.get_clock().now().to_msg()
                    points.publish(String(data=json.dumps(dict(schema='paint_detection_point_result_v1',
                        result_id=batch+'_1', batch_id=batch, point_index=1, success=True, returned_home=True,
                        measurement=dict(schema='paint_thickness_measurement_v1', value=123., unit='um',
                                         sequence=len(starts), received_at_ros=dict(sec=stamp.sec, nanosec=stamp.nanosec))))))
                messages = [BmsStatus(bat_soc=73, bat_voltage=48.2, bat_current=-1.5,
                                     bms_health=BmsStatus.BMS_HEALTH_GOOD),
                            ChassisStatus(), CommStatus(comm_state=CommStatus.COMM_STATE_CONNECTED)]
                for publisher, message in zip(power_pubs, messages):
                    message.header.stamp = detector.get_clock().now().to_msg()
                    publisher.publish(message)
            detector.create_timer(.05, broadcast)
            for node in nodes:
                executor.add_node(node)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            listener = RobotListener(dict(client, output_dir=str(host), rgbd_bind='127.0.0.1', rgbd_port=0))
            def wait(predicate, timeout=8.):
                deadline = time.monotonic()+timeout
                while time.monotonic() < deadline:
                    if predicate():
                        return
                    time.sleep(.02)
                raise AssertionError(f'timed out: camera={task.phase} {task.detail}; workflow={workflow.phase}')
            def command(action, identity):
                return request(client, dict(schema='paint_robot_task_request_v1', action=action, message_id=identity))
            def result(identity, digest):
                return dict(schema='paint_v54_pi_remote_result_v1', message_id=identity, status='OK_3D_POINTS',
                    capture_manifest_sha256=digest, coordinate_frame='CAMERA_PINHOLE_PRELIMINARY', depth_source='MEASURED',
                    robot_motion_authorized=False, robot_base_coordinate_calibrated=False, robot_command=None,
                    points_xyz_camera_mm=[dict(point_id='same-point', xyz_camera_mm=[10., 20., 300.])])
            def remote_phase():
                return (status(client)['robot_status']['camera_detection']['data'] or {}).get('state')
            wait(lambda: bridge.start_task.service_is_ready() and bridge.detect_camera.service_is_ready())
            assert not captures and not starts
            assert command('start', 'start-once')['ok']
            assert command('start', 'start-once')['disposition'] == 'duplicate'
            wait(lambda: task.phase == 'WAIT_SAVED')
            wait(lambda: remote_phase() == 'WAIT_SAVED')
            assert not starts
            listener.acknowledge_saved_capture(status(client)['robot_status'])
            wait(lambda: task.phase == 'WAIT_DETECT')
            first = task.snapshot['capture_manifest_sha256']
            assert not request(client, result('too-early', first))['ok']
            assert command('detect', 'detect-one')['ok']
            wait(lambda: remote_phase() == 'WAIT_RESULT')
            assert not request(client, result('old-one', '0'*64))['ok']
            packet = result('points-one', first)
            assert request(client, packet)['ok']
            wait(lambda: task.phase == 'RUNNING')
            assert not command('go_home', 'home-while-moving')['ok']
            wait(lambda: task.phase == 'WAIT_NEXT')
            assert workflow.active and workflow.step_index == 1 and task.completed_cycles == 1
            workflow.step_started -= 100.  # Waiting for the host is not a motion timeout.
            time.sleep(.2)
            assert workflow.active and not workflow.failed
            assert request(client, packet)['disposition'] == 'duplicate'
            assert command('detect', 'detect-two')['ok']
            assert command('detect', 'detect-two')['disposition'] == 'duplicate'
            wait(lambda: task.phase == 'WAIT_SAVED' and len(captures) == 2)
            wait(lambda: remote_phase() == 'WAIT_SAVED')
            listener.acknowledge_saved_capture(status(client)['robot_status'])
            wait(lambda: remote_phase() == 'WAIT_RESULT')
            assert not request(client, result('old-after-recapture', first))['ok']
            second = task.snapshot['capture_manifest_sha256']
            assert second != first
            assert request(client, result('points-two', second))['ok']
            wait(lambda: task.phase == 'WAIT_NEXT')
            assert task.completed_cycles == 2 and len(starts) == 2 and len(homes) == 1
            wait(lambda: len(bridge.detection_results) == 2)
            returned = status(client)['robot_status']
            assert returned['chassis_power']['battery_percent'] == 73
            assert returned['camera_detection']['data']['state'] == 'WAIT_NEXT'
            assert len(returned['detection_results']) == 2
            task.last_cycle_error = 'synthetic measurement retry reason'
            wait(lambda: any(item['error_code'] == 'E5002' for item in
                            (status(client)['robot_status']['abnormal_status'] or {}).get('active_faults', [])))
            task.last_cycle_error = ''
            assert command('go_home', 'home-after-done')['ok']
            wait(lambda: not workflow.active and workflow.step_index == len(workflow.steps))
            assert task.phase == 'DONE'
    finally:
        executor.shutdown(timeout_sec=2.)
        if thread:
            thread.join(2.)
        if listener:
            listener.server.close()
        for node in nodes:
            if isinstance(node, IndustrialPC):
                node.close()
            if isinstance(node, CameraDetectionTask):
                node.live.close()
            node.destroy_node()
        rclpy.shutdown()
