"""Mode selection, launch wiring and low-power return; no hardware or motion."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
import yaml
from launch import LaunchContext
from masc_chassis_msgs.msg import BmsStatus, CommStatus

from robot_task_control.camera_detection_oline import CameraDetectionOline
from robot_task_control.camera_detection_task import CameraDetectionTask
from robot_task_control.task_config import load_task_config
from robot_task_control.task_modes import mode_config
from robot_task_control.workflow_controller import WorkflowController

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('mode', ['local', 'online'])
@pytest.mark.parametrize('profile', ['full', 'line', 'photo'])
def test_modes_keep_shared_protection_and_select_only_requested_steps(tmp_path, mode, profile):
    _, original = load_task_config(ROOT/'config/robot_task.yaml')
    config = mode_config(original, mode, profile)
    path = tmp_path/'effective.yaml'
    path.write_text(yaml.safe_dump(config))
    load_task_config(path)
    flow = WorkflowController.load_workflow(path)
    assert config['detect_controller'] == original['detect_controller']
    assert config['travel'] == original['travel']
    assert config['system']['auto_start'] is (mode == 'local')
    assert config['system']['line_charging_mode'] is (mode == 'online' and profile != 'photo')
    camera = config['camera_detection_task']['ros__parameters']
    assert camera['target_source'] == ('tcp' if mode == 'online' else 'local')
    assert camera['low_battery_return_enabled'] is (mode == 'online' and profile == 'full')
    assert camera['low_battery_threshold'] == 10.
    actions = [s['action'] for s in flow['steps']]
    if profile == 'photo':
        assert actions == ['camera_detection']
    else:
        assert actions.count('line_follow') == 2
        for index, step in enumerate(flow['steps']):
            if step['action'] == 'line_follow':
                assert flow['steps'][index-1]['posture'] == 'travel'
        if profile == 'line':
            assert set(actions) == {'wait_chassis_stopped', 'arm_posture', 'line_follow'}
        else:
            assert config['workflow']['steps'] == original['workflow']['steps']
    assert original['camera_detection_task']['ros__parameters']['target_source'] == 'local'


@pytest.mark.parametrize('mode', ['local', 'online'])
@pytest.mark.parametrize('auto', [True, False])
def test_either_mode_can_start_locally_or_wait_for_local_remote_trigger(mode, auto):
    _, original = load_task_config(ROOT/'config/robot_task.yaml')
    config = mode_config(original, mode, auto_start=auto)
    assert config['system']['auto_start'] is auto
    assert config['tcp']['enabled'] is True


@pytest.mark.parametrize('mode', ['local', 'online'])
@pytest.mark.parametrize('profile', ['full', 'line', 'photo'])
def test_launch_uses_effective_recipe_and_photo_never_starts_chassis(tmp_path, mode, profile):
    spec = importlib.util.spec_from_file_location('task_modes_launch', ROOT/'launch/robot_task.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    from test_task_config import recipe
    path, _ = recipe(tmp_path)
    if profile == 'photo':
        document = yaml.safe_load(path.read_text())
        document['system']['standard_route_file'] = '/no-route-needed-for-photo'
        path.write_text(yaml.safe_dump(document))
    context = LaunchContext()
    context.launch_configurations.update(config_file=str(path), mode=mode, workflow=profile,
                                         auto_start='false', validate_only='true')
    with patch.object(launch, 'Node') as node, patch.object(launch, 'IncludeLaunchDescription') as include:
        assert launch.launch_setup(context) == []
        node.assert_not_called()
        include.assert_not_called()
    context.launch_configurations['validate_only'] = 'false'
    with patch.object(launch, 'Node') as node, patch.object(launch, 'IncludeLaunchDescription') as include:
        launch.launch_setup(context)
    arguments = [dict(call.kwargs['launch_arguments']) for call in include.call_args_list]
    system = next(a for a in arguments if 'workflow_file' in a)
    effective = Path(system['workflow_file'])
    try:
        config = yaml.safe_load(effective.read_text())
        assert config['workflow']['task_mode'] == mode
        assert config['workflow']['task_profile'] == profile
        assert system['auto_start'] == 'false'
        assert system['line_charging_mode'] == str(mode == 'online' and profile != 'photo').lower()
        if profile == 'photo':
            assert system['start_chassis'] == system['start_line_camera'] == 'false'
            assert system['use_velocity_mux'] == 'false'
        camera = next(c.kwargs for c in node.call_args_list
                      if c.kwargs.get('name') == 'camera_detection_task')
        assert camera['executable'] == ('camera_detection_oline' if mode == 'online' else 'camera_detection_task')
    finally:
        effective.unlink(missing_ok=True)


def power_node():
    node = CameraDetectionOline.__new__(CameraDetectionOline)
    node.power_samples = {}
    node.low_battery_enabled = True
    node.low_battery_threshold = 10.
    node.low_battery_return_pending = False
    node.phase = 'WAIT_NEXT'
    node.get_clock = Mock(return_value=SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1000*10**9)))
    return node


@pytest.mark.parametrize('soc,health,connected,stamp,received,expected', [
    (10, 1, True, 1000, 100., False),
    (9, 1, True, 1000, 100., True),
    (0, 1, True, 1000, 100., True),
    (101, 1, True, 1000, 100., False),
    (9, 0, True, 1000, 100., False),
    (9, 255, True, 1000, 100., False),
    (9, 1, False, 1000, 100., False),
    (9, 1, True, 996, 100., False),
    (9, 1, True, 1000, 96., False),
    (9, 1, True, 0, 100., False),
])
def test_only_fresh_connected_valid_soc_below_ten_latches_return(soc, health, connected, stamp, received, expected):
    node = power_node()
    bms = BmsStatus(bat_soc=soc, bms_health=health, bat_voltage=48., bat_current=-1.)
    comm = CommStatus(comm_state=CommStatus.COMM_STATE_CONNECTED if connected else 0)
    bms.header.stamp.sec = comm.header.stamp.sec = stamp
    node.power_samples = {'bms': (bms, received), 'comm': (comm, received)}
    with patch('robot_task_control.camera_detection_oline.monotonic', return_value=100.):
        node.update_battery()
    assert node.low_battery_return_pending is expected


def test_low_battery_never_interrupts_running_detection_and_survives_stale_feedback():
    node = power_node()
    node.low_battery_return_pending = True
    node.stop_capture, node.transition, node.ready = Mock(), Mock(), Mock(return_value=True)
    node.results = []
    node.operation = None
    node.phase = 'RUNNING'
    assert not node.return_if_safe()
    node.stop_capture.assert_not_called()
    node.transition.assert_not_called()
    node.update_battery()  # Missing feedback cannot clear a latched request.
    assert node.battery_percent is None and node.low_battery_return_pending
    node.phase = 'WAIT_NEXT'
    assert node.return_if_safe()
    assert node.home_requested and node.return_reason == 'low_battery'
    assert node.transition.call_args.args[0] == 'DONE'


def test_low_battery_will_not_release_home_if_motion_is_not_ready():
    node = power_node()
    node.low_battery_return_pending = True
    node.ready, node.transition = Mock(return_value=False), Mock()
    assert not node.return_if_safe()
    node.transition.assert_not_called()


def test_low_battery_blocks_new_detection_even_before_the_next_timer_tick():
    from test_oline_control import task
    from std_srvs.srv import Trigger
    node = task('WAIT_NEXT')
    node.low_battery_return_pending = True
    response = CameraDetectionOline.detect_request(node, Trigger.Request(), Trigger.Response())
    assert not response.success and '返回充电' in response.message
    node.reset_cycle.assert_not_called()


@pytest.mark.parametrize('phase', ['STARTING', 'RUNNING', 'FAULT', 'IDLE', 'DONE'])
def test_low_battery_cannot_release_return_outside_confirmed_safe_phases(phase):
    node = power_node()
    node.phase, node.low_battery_return_pending = phase, True
    node.ready, node.transition = Mock(return_value=True), Mock()
    assert not node.return_if_safe()
    node.transition.assert_not_called()


def test_arrival_waits_for_detect_before_first_capture():
    node = power_node()
    node.detection_requested = False
    node.transition = Mock()
    with patch.object(CameraDetectionTask, 'begin_capture') as capture:
        node.begin_capture()
        capture.assert_not_called()
        assert node.transition.call_args.args[0] == 'WAIT_DETECT'
        node.detection_requested = True
        node.begin_capture()
        capture.assert_called_once()
