#!/usr/bin/env python3
"""Evaluate launch with fake Nodes: no hardware, GUI or robot commands."""

from pathlib import Path

import launch_ros.actions
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.utilities import evaluate_parameters, normalize_parameters


seen = []
original = launch_ros.actions.Node


def fake_node(**kwargs):
    def inspect(context):
        params = evaluate_parameters(context, normalize_parameters(kwargs.get('parameters', [])))
        overrides = {}
        for item in params:
            if isinstance(item, dict):
                overrides.update(item)
        seen.append((kwargs.get('name'), overrides))
        return []
    return OpaqueFunction(function=inspect, condition=kwargs.get('condition'))


try:
    launch_ros.actions.Node = fake_node
    source = Path(__file__).resolve().parents[1] / 'launch' / 'line_follow.launch.py'
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(source)),
            launch_arguments={'show_window': 'true', 'dry_run': 'true',
                              'enabled': 'false'}.items(),
        ),
    ]))
    assert service.run() == 0
    controllers = [params for name, params in seen if name == 'line_follow_controller']
    trackers = [params for name, params in seen if name == 'line_tracker']
    cameras = [params for name, params in seen if name == 'camera_node']
    assert len(controllers) == 1 and controllers[0]['show_window'] is True, seen
    assert controllers[0]['enabled'] is False, seen
    assert len([p for name, p in seen if name == 'line_exposure_controller']) == 1, seen
    assert len(trackers) == 1 and trackers[0]['show_window'] is False, seen
    assert len(cameras) == 1 and cameras[0]['video_device'].endswith(
        'USB_2.0_Camera_SN0001-video-index0'), seen
    seen.clear()
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)),
            launch_arguments={'drive_direction': 'reverse', 'route_file': '/tmp/route-scope.json', 'dry_run': 'true',
                              'enabled': 'false', 'route_camera_stop': 'false', 'route_visual_finish': 'false'}.items()),
    ]))
    assert service.run() == 0
    controllers = [params for name, params in seen if name == 'line_follow_controller']
    assert len(controllers) == 1 and controllers[0]['drive_direction'] == 'reverse'
    assert controllers[0]['reverse_steering_sign'] == -1.
    assert controllers[0]['reverse_mode'] == 'recorded_route'
    assert controllers[0]['route_file'] == '/tmp/route-scope.json'
    assert controllers[0]['route_camera_stop'] is False
    assert controllers[0]['route_visual_finish'] is False
    assert controllers[0]['forward_speed'] == .125 and controllers[0]['dry_run'] is True
    print('PASS: camera preview disabled locally; controller preview remains enabled')
    keyboard_source = source.with_name('keyboard_teleop.launch.py')
    for arguments, preview in [({}, True), ({'show_window': 'false'}, False),
                               ({'camera_device': '/dev/v4l/by-id/test-chassis-camera'}, True)]:
        seen.clear()
        service = LaunchService()
        service.include_launch_description(LaunchDescription([
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(keyboard_source)),
                launch_arguments={'start_driver': 'false', **arguments}.items()),
        ]))
        assert service.run() == 0
        assert len([p for name, p in seen if name == 'keyboard_teleop']) == 1, seen
        assert not any(name == 'line_follow_controller' for name, _ in seen), seen
        cameras = [p for name, p in seen if name == 'camera_node']
        trackers = [p for name, p in seen if name == 'line_tracker']
        assert len(cameras) == len(trackers) == int(preview), seen
        if preview:
            assert trackers[0]['show_window'] is True, seen
            assert cameras[0]['video_device'] == arguments.get('camera_device',
                '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'), seen
    print('PASS: keyboard camera preview defaults on, can be disabled, respects device override; no automatic driving')
finally:
    launch_ros.actions.Node = original
