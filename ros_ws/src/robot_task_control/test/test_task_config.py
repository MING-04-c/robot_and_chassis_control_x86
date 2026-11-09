"""Check the production YAML and launch wiring without touching hardware."""

import importlib.util
import json
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml
from launch import LaunchContext
from launch.actions import OpaqueFunction

from robot_task_control.final_capture import load_config
from robot_task_control.task_config import load_task_config
from robot_task_control.workflow_controller import WorkflowController


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'config/robot_task.yaml'


def recipe(tmp_path, mutate=None):
    data = yaml.safe_load(CONFIG.read_text())
    route = tmp_path/'route.json'
    route.write_text(json.dumps({
        'version': 1, 'valid': True, 'finished': True,
        'frame_id': 'odom', 'child_frame_id': 'base_link',
        'points': [[i*.02, 0., 0.] for i in range(11)],
    }))
    data['system']['standard_route_file'] = str(route)
    if mutate:
        mutate(data)
    path = tmp_path/'robot_task.yaml'
    path.write_text(yaml.safe_dump(data))
    return path, data


def launch_module():
    spec = importlib.util.spec_from_file_location('unified_robot_task_launch', ROOT/'launch/robot_task.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(path, validate_only=False):
    result = LaunchContext()
    result.launch_configurations.update(config_file=str(path), validate_only=str(validate_only).lower())
    return result


def test_production_workflow_preserves_sequence_pose_and_route_mode():
    _, data = load_task_config(CONFIG)
    flow = WorkflowController.load_workflow(CONFIG)
    assert [step['action'] for step in flow['steps']] == [
        'wait_chassis_stopped', 'arm_posture', 'line_follow', 'wait_chassis_stopped',
        'arm_posture', 'arm_move', 'camera_detection', 'arm_posture', 'arm_posture',
        'line_follow', 'wait_chassis_stopped',
    ]
    moves = [s for s in flow['steps'] if s['action'] == 'line_follow']
    assert [s['drive_direction'] for s in moves] == ['forward', 'reverse']
    assert moves[0]['finish_at_first_marker'] and moves[1]['return_to_start']
    photo = next(s for s in flow['steps'] if s['action'] == 'arm_move')
    assert photo['target_pose']['position'] == [.375, .360, .590]
    assert photo['target_pose']['orientation'] == [-175., 0., -90.]
    assert photo['after_arrival_delay_sec'] == .5
    assert data['system']['line_navigation_mode'] == 'replay'
    assert data['system']['line_record_route'] is False
    assert data['system']['line_follow_speed'] == .1
    assert data['system']['line_reverse_speed'] == .05
    assert load_config(CONFIG) == data['capture']


def test_yaml_edits_reach_every_component(tmp_path):
    def mutate(data):
        data['system']['line_follow_speed'] = .09
        data['system']['auto_start'] = False
        data['system']['line_camera_device'] = '/dev/v4l/by-id/test-camera'
        data['arm_joint_postures']['ros__parameters']['initial_settle_sec'] = .75
        data['camera_detection_task']['ros__parameters']['ready_stable_sec'] = .6
        data['camera_detection_task']['ros__parameters']['camera_points_mm'] = [1., 2., 3.]
        data['detect_controller']['ros__parameters']['hold_seconds'] = 4.
        data['capture']['robot_settle_sec'] = .8
        data['workflow']['travel_posture_timeout_sec'] = 123.
        data['workflow']['steps'][4]['after_arrival_delay_sec'] = .7
    path, data = recipe(tmp_path, mutate)
    module, nodes, includes = launch_module(), {}, []
    def node(**kwargs):
        nodes[kwargs['name']] = kwargs['parameters']
        return OpaqueFunction(function=lambda ctx: [])
    def include(source, launch_arguments):
        includes.append(dict(launch_arguments))
        return OpaqueFunction(function=lambda ctx: [])
    ctx = context(path)
    with patch.object(module, 'Node', side_effect=node), patch.object(module, 'IncludeLaunchDescription', side_effect=include):
        module.launch_setup(ctx)
    system = next(args for args in includes if 'workflow_file' in args)
    assert system['workflow_file'] == str(path)
    assert system['capture_config_file'] == str(path)
    assert system['line_follow_speed'] == '0.09'
    assert system['auto_start'] == 'false'
    assert system['line_camera_device'] == data['system']['line_camera_device']
    assert system['require_arm_ready'] == system['check_devices_on_startup'] == 'true'
    assert system['use_velocity_mux'] == system['launch_force_sensor'] == 'true'
    posture = nodes['arm_joint_postures'][0]
    assert posture['postures_file'] == str(path) and posture['initial_settle_sec'] == .75
    camera = nodes['camera_detection_task'][0]
    assert camera['capture_config_file'] == str(path)
    assert camera['ready_stable_sec'] == .6 and camera['camera_points_mm'] == [1., 2., 3.]
    assert nodes['detect_controller'][-1]['hold_seconds'] == 4.
    assert nodes['detect_controller'][-1]['joint_postures_file'] == str(path)
    assert load_config(path)['robot_settle_sec'] == .8
    flow = WorkflowController.load_workflow(path)
    assert next(s for s in flow['steps'] if s['action'] == 'arm_move')['after_arrival_delay_sec'] == .7
    assert next(s for s in flow['steps'] if s['action'] == 'arm_posture')['step_timeout_sec'] == 123.


def test_validation_only_never_constructs_device_actions(tmp_path):
    path, _ = recipe(tmp_path)
    module = launch_module()
    with patch.object(module, 'Node') as node, patch.object(module, 'IncludeLaunchDescription') as include:
        assert module.launch_setup(context(path, True)) == []
        node.assert_not_called()
        include.assert_not_called()


def test_workflow_steps_can_change_without_another_task_implementation(tmp_path):
    def mutate(data):
        data['workflow']['steps'] = [
            step for step in data['workflow']['steps'] if step['action'] != 'camera_detection']
        data['workflow']['steps'].insert(5, {'action': 'delay', 'seconds': .5})
    path, _ = recipe(tmp_path, mutate)
    flow = WorkflowController.load_workflow(path)
    assert not any(step['action'] == 'camera_detection' for step in flow['steps'])
    assert next(step for step in flow['steps'] if step['action'] == 'delay')['seconds'] == .5


@pytest.mark.parametrize('change', [
    lambda d: d['system'].update(line_follow_speed=float('nan')),
    lambda d: d['system'].update(line_record_route=True),
    lambda d: d['system'].update(line_show_window='false'),
    lambda d: d['system'].update(line_follow_speeed=.1),
    lambda d: d['workflow']['steps'][1].update(marker_confirm_timeout_sec=-1.),
    lambda d: d['workflow'].update(line_stream_service_timeout_sec=0.),
    lambda d: d['workflow'].update(arm_postures_before_chassis=False),
    lambda d: d['camera_detection_task']['ros__parameters'].update(auto_start=True),
    lambda d: d['camera_detection_task']['ros__parameters'].update(camera_points_mm=[1., 2.]),
    lambda d: d['detect_controller']['ros__parameters'].update(contact_force=50.),
    lambda d: d['capture'].update(camera_warmup_sec=40.),
    lambda d: d['travel'].pop(),
])
def test_bad_config_fails_before_any_device_action(tmp_path, change):
    path, _ = recipe(tmp_path, change)
    module = launch_module()
    with patch.object(module, 'Node') as node, patch.object(module, 'IncludeLaunchDescription') as include:
        with pytest.raises((ValueError, KeyError)):
            module.launch_setup(context(path))
        node.assert_not_called()
        include.assert_not_called()


def test_invalid_route_does_not_start_drivers(tmp_path):
    path, data = recipe(tmp_path)
    Path(data['system']['standard_route_file']).write_text('{}')
    module = launch_module()
    with patch.object(module, 'Node') as node:
        with pytest.raises(ValueError, match='Standard route rejected'):
            module.launch_setup(context(path))
        node.assert_not_called()


def test_internal_launch_accepts_all_system_keys(tmp_path):
    from launch.actions import DeclareLaunchArgument
    spec = importlib.util.spec_from_file_location('robot_system_schema', ROOT/'launch/internal/robot_system.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    names = {item.name for item in module.generate_launch_description().entities
             if isinstance(item, DeclareLaunchArgument)}
    _, data = recipe(tmp_path)
    assert set(data['system']) <= names


def test_only_one_production_line_task_remains():
    assert not list((ROOT/'launch').glob('line_*.launch.py'))
    assert not list((ROOT/'config').glob('line_*.yaml'))
    assert (ROOT/'launch/robot_task.launch.py').is_file()
    assert (ROOT/'archive/config/line_robot_test.yaml').is_file()
