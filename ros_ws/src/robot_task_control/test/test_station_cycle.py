"""Dedicated station recipe and host-paced handoffs, without hardware."""
from copy import deepcopy
import json
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
import yaml
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_task_control.camera_detection_task import CameraDetectionTask
from robot_task_control.camera_detection_oline import CameraDetectionOline
from robot_task_control.oline_config import load_oline_config
from robot_task_control.station_cycle import station_cycle_steps, excursion_steps, middle_home_steps
from robot_task_control.workflow_controller import WorkflowController
from test_oline_control import ROOT, task


def recipe(tmp_path):
    config = load_oline_config(ROOT/'config/robot_task_station_cycle.yaml')
    path = tmp_path/'recipe.yaml'
    path.write_text(yaml.safe_dump(config))
    return config, WorkflowController.load_workflow(path)['steps']


def test_dedicated_recipe_reuses_every_arm_point_and_keeps_old_entry(tmp_path):
    config, steps = recipe(tmp_path)
    original = deepcopy(steps)
    normal = load_oline_config(ROOT/'config/robot_task_oline.yaml')
    assert not normal['camera_detection_task']['ros__parameters'].get('station_return_enabled', False)
    assert config['camera_detection_task']['ros__parameters']['station_return_enabled']
    assert config['detect_controller'] == normal['detect_controller']
    forward = next(s for s in steps if s.get('drive_direction') == 'forward')
    assert not forward['finish_at_first_marker'] and forward['skip_intermediate_markers']
    parts = station_cycle_steps(steps)
    camera = next(i for i, s in enumerate(steps) if s['action'] == 'camera_detection')
    assert parts['inward'] == steps[camera+1:-2]
    assert parts['inward'][-1]['posture'] == 'travel'
    excursion = excursion_steps(steps)
    assert [s['station_role'] for s in excursion if 'station_role' in s] == ['mid_photo', 'auto_terminal']
    assert [s.get('route_segment') for s in excursion if s['action'] == 'line_follow'] == ['station', 'forward', None]
    assert [s.get('route_segment') for s in middle_home_steps(steps) if s['action'] == 'line_follow'] == ['home']
    assert [s for s in excursion if s['action'] in ('arm_move', 'arm_posture')] == [
        s for block in (parts['inward'], parts['outward'], parts['inward'], parts['outward'], parts['inward'])
        for s in block if s['action'] in ('arm_move', 'arm_posture')]
    excursion[0]['name'] = 'changed-copy'
    assert steps == original


@pytest.mark.parametrize('enabled,role,phase,low,accepted', [
    (True, 'terminal', 'WAIT_DETECT', False, True),
    (True, 'terminal', 'WAIT_NEXT', False, True),
    (False, 'terminal', 'WAIT_DETECT', False, False),
    (True, 'mid_photo', 'WAIT_DETECT', False, False),
    (True, 'terminal', 'RUNNING', False, False),
    (True, 'terminal', 'WAIT_DETECT', True, False),
])
def test_return_station_gates(enabled, role, phase, low, accepted):
    node = task(phase)
    node.station_return_enabled, node.station_role = enabled, role
    node.low_battery_return_pending = low
    response = CameraDetectionOline.return_station_request(node, None, Trigger.Response())
    assert response.success == accepted
    if accepted:
        assert node.phase == 'DONE' and node.return_reason == 'return_station'


def test_mid_photo_only_moves_on_matching_three_image_save_ack(tmp_path):
    node = task('WAIT_SAVED')
    node.station_role, node.run_id, node.storage = 'mid_photo', 'photo-id', tmp_path
    node.snapshot, node.fail = {'capture_manifest_sha256': 'a'*64}, Mock()
    files = {name: 'b'*64 for name in ('hk.jpg', 'rgb.png', 'depth.png')}
    (tmp_path/'.capture_index.json').write_text(json.dumps({'sessions': {node.run_id: {'files': files}}}))
    packet = dict(capture_set_id='old', capture_manifest_sha256='a'*64, files_sha256=files)
    CameraDetectionOline.on_capture_saved(node, String(data=json.dumps(packet)))
    assert node.phase == 'WAIT_SAVED'
    packet['capture_set_id'] = node.run_id
    CameraDetectionOline.on_capture_saved(node, String(data=json.dumps(packet)))
    assert node.phase == 'DONE' and node.return_reason == 'next_station'
    node.fail.assert_not_called()


def test_auto_terminal_measurement_completes_session_instead_of_waiting_next():
    node = CameraDetectionOline.__new__(CameraDetectionOline)
    node.station_role, node.home_requested = 'auto_terminal', False
    node.get_logger, node.save_report = Mock(), Mock()
    with patch.object(CameraDetectionOline, 'publish_oline_status'):
        node.transition('DONE', 'batch finished and photo home confirmed')
    assert node.phase == 'DONE' and node.return_reason == 'detection_complete'


@pytest.mark.parametrize('role,reason,expected', [
    ('terminal', 'return_station', ['station', 'forward', None]),
    ('mid_photo', 'go_home', ['home']),
    ('mid_photo', 'low_battery', ['home']),
])
def test_workflow_replaces_remaining_branch_only_after_camera_done(tmp_path, role, reason, expected):
    _, steps = recipe(tmp_path)
    node = SimpleNamespace(phase='camera_detection_running',
        camera_detection_status={'state': 'DONE', 'return_reason': reason},
        current_step=lambda: {'station_role': role}, step_index=0,
        steps=[{'action': 'camera_detection'}, {'action': 'old_tail'}], original_steps=steps,
        complete_step=Mock())
    WorkflowController.advance_camera_detection(node)
    assert [s.get('route_segment') for s in node.steps[1:] if s['action'] == 'line_follow'] == expected
    node.complete_step.assert_called_once()
