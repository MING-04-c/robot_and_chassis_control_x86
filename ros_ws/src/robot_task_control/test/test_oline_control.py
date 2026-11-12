"""Host-paced states and launch recipe checks without devices or a ROS graph."""
from collections import deque
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml
from launch import LaunchContext

from robot_task_control.camera_detection_task import CameraDetectionTask
from robot_task_control.camera_detection_oline import CameraDetectionOline
from robot_task_control.oline_config import load_oline_config
from robot_task_control.workflow_controller import WorkflowController

ROOT = Path(__file__).resolve().parents[1]


def task(phase='WAIT_DETECT'):
    node = SimpleNamespace(phase=phase, ready=Mock(return_value=True), results=deque(),
                           reset_cycle=Mock(), detection_requested=False)
    def transition(state, detail):
        node.phase, node.detail = state, detail
    node.transition = transition
    return node


def test_oline_recipe_reuses_motion_and_returns_only_after_remote_session(tmp_path):
    config = load_oline_config(ROOT/'config/robot_task_oline.yaml')
    original = yaml.safe_load((ROOT/'config/robot_task.yaml').read_text())
    assert config['system']['auto_start'] is False
    assert config['tcp']['enabled'] and config['tcp']['enable_status_monitor']
    assert config['camera_detection_task']['ros__parameters']['target_source'] == 'tcp'
    assert config['capture']['local_only'] is False and config['capture']['hk_bind_ip'] == '0.0.0.0'
    assert config['detect_controller'] == original['detect_controller']
    assert config['workflow']['steps'] == original['workflow']['steps']
    path = tmp_path/'recipe.yaml'
    path.write_text(yaml.safe_dump(config))
    steps = WorkflowController.load_workflow(path)['steps']
    index = next(i for i, s in enumerate(steps) if s['action'] == 'camera_detection')
    assert [s.get('name') for s in steps[index+1:index+3]] == [
        'return_camera_intermediate', 'return_camera_via']
    assert config['workflow']['remote_detection_cycles'] is True


def test_shared_launch_selects_oline_node_and_validates_before_devices(tmp_path):
    config = load_oline_config(ROOT/'config/robot_task_oline.yaml')
    route = tmp_path/'route.json'
    route.write_text(json.dumps(dict(version=1, valid=True, finished=True,
        frame_id='odom', child_frame_id='base_link', points=[[i*.02, 0., 0.] for i in range(11)])))
    config['system']['standard_route_file'] = str(route)
    recipe = tmp_path/'oline.yaml'
    recipe.write_text(yaml.safe_dump(config))
    spec = importlib.util.spec_from_file_location('oline_shared_launch_test', ROOT/'launch/robot_task.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    context = LaunchContext()
    context.launch_configurations.update(config_file=str(recipe), oline_mode='true', validate_only='true')
    with patch.object(launch, 'Node') as node, patch.object(launch, 'IncludeLaunchDescription') as include:
        assert launch.launch_setup(context) == []
        node.assert_not_called()
        include.assert_not_called()
    context.launch_configurations['validate_only'] = 'false'
    with patch.object(launch, 'Node') as node, patch.object(launch, 'IncludeLaunchDescription') as include:
        launch.launch_setup(context)
        cameras = [call.kwargs for call in node.call_args_list if call.kwargs.get('name') == 'camera_detection_task']
        assert cameras[0]['executable'] == 'camera_detection_oline'
        assert cameras[0]['parameters'][0]['target_source'] == 'tcp'
        serial = [dict(call.kwargs['launch_arguments']) for call in include.call_args_list]
        assert any(args.get('oline_mode') == 'true' for args in serial)
        assert any(args.get('auto_start') == 'false' for args in serial)


def test_detect_first_uses_saved_photo_and_next_detect_requests_a_new_capture():
    node = task()
    result = CameraDetectionOline.detect_request(node, Trigger.Request(), Trigger.Response())
    assert result.success and node.phase == 'WAIT_RESULT'
    node.reset_cycle.assert_not_called()
    node.phase = 'WAIT_NEXT'
    result = CameraDetectionOline.detect_request(node, Trigger.Request(), Trigger.Response())
    assert result.success and node.phase == 'WAIT_READY'
    node.reset_cycle.assert_called_once_with(True)
    for phase in ('IDLE', 'CAPTURE', 'WAIT_SAVED', 'WAIT_RESULT', 'RUNNING', 'DONE', 'FAULT'):
        node.phase = phase
        assert not CameraDetectionOline.detect_request(node, Trigger.Request(), Trigger.Response()).success


def test_go_home_is_separate_from_stop_and_never_interrupts_contact():
    node = task('RUNNING')
    assert not CameraDetectionOline.go_home_request(node, Trigger.Request(), Trigger.Response()).success
    assert node.phase == 'RUNNING'
    for phase in ('WAIT_DETECT', 'WAIT_NEXT'):
        node.phase = phase
        assert CameraDetectionOline.go_home_request(node, Trigger.Request(), Trigger.Response()).success
        assert node.phase == 'DONE'
    node = task()
    node.ready.return_value = False
    assert not CameraDetectionOline.go_home_request(node, Trigger.Request(), Trigger.Response()).success


def test_saved_capture_requires_both_identity_and_all_file_hashes(tmp_path):
    node = task('WAIT_SAVED')
    node.run_id, node.storage = 'current-photo', tmp_path
    node.snapshot = {'capture_manifest_sha256': 'a'*64}
    files = {name: 'b'*64 for name in ('hk.jpg', 'rgb.png', 'depth.png')}
    (tmp_path/'.capture_index.json').write_text(json.dumps({'sessions': {
        node.run_id: {'files': files}}}))
    node.fail = Mock()
    packet = dict(capture_set_id='old-photo', capture_manifest_sha256='a'*64, files_sha256=files)
    CameraDetectionOline.on_capture_saved(node, String(data=json.dumps(packet)))
    assert node.phase == 'WAIT_SAVED'
    packet['capture_set_id'] = node.run_id
    CameraDetectionOline.on_capture_saved(node, String(data=json.dumps(packet)))
    assert node.phase == 'WAIT_DETECT' and node.snapshot['host_saved']
    node.phase, node.detection_requested = 'WAIT_SAVED', True
    CameraDetectionOline.on_capture_saved(node, String(data=json.dumps(packet)))
    assert node.phase == 'WAIT_RESULT'
    node.phase = 'WAIT_SAVED'
    packet['files_sha256'] = dict(files, **{'hk.jpg': 'c'*64})
    CameraDetectionOline.on_capture_saved(node, String(data=json.dumps(packet)))
    node.fail.assert_called_once()


def test_cycle_done_waits_at_photo_without_finishing_workflow():
    # Exercise the real subclass dispatch; base movement/completion is isolated.
    node = CameraDetectionOline.__new__(CameraDetectionOline)
    node.phase, node.completed_cycles, node.detection_requested = 'RUNNING', 0, True
    node.home_requested = False
    node.save_report, node.get_logger = Mock(), Mock()
    with patch.object(CameraDetectionTask, 'advance', lambda subject: subject.transition('DONE', 'batch done')), \
            patch.object(CameraDetectionOline, 'publish_oline_status'):
        CameraDetectionOline.advance(node)
    assert node.phase == 'WAIT_NEXT' and node.completed_cycles == 1
    assert node.detection_requested is False


def test_new_cycle_clears_old_points_but_preserves_session_home_and_root():
    node = task('WAIT_NEXT')
    node.run_id, node.root_run_id = 'old-photo', 'root-session'
    node.home_recorded = True
    node.stop_capture = Mock()
    node.results.append({'old': True})
    node.completed_point_ids, node.expected_point_ids, node.return_events = {'p1'}, {'p1'}, {('batch', 1)}
    CameraDetectionOline.reset_cycle(node, True)
    assert node.run_id != 'old-photo' and node.root_run_id == 'root-session'
    assert node.home_recorded and node.detection_requested
    assert not node.results and not node.expected_point_ids and not node.completed_point_ids
    assert not node.return_events and node.snapshot is None


def test_recoverable_failure_waits_for_host_instead_of_recapturing():
    node = task('RUNNING')
    node.started_detection = True
    CameraDetectionOline.schedule_reacquire(node, '本次没有测厚回包，已回位')
    assert node.phase == 'WAIT_NEXT' and not node.started_detection
    assert node.last_cycle_error
    node.reset_cycle.assert_not_called()
