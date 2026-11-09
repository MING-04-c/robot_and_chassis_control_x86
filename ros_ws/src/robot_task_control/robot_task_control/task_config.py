"""Load the single production recipe without starting ROS nodes or hardware."""

import math
import ipaddress
from pathlib import Path

import yaml


SYSTEM_KEYS = {
    'ros_discovery_range', 'robot_ip', 'local_ip', 'auto_start',
    'arm_control_start_delay', 'launch_rviz', 'line_camera_device',
    'line_show_window', 'line_processing_fps', 'line_opencv_threads',
    'line_marker_opencv_threads', 'line_navigation_mode', 'standard_route_file',
    'line_record_route', 'line_charging_mode', 'line_follow_speed',
    'line_reverse_speed', 'line_reverse_route_max_curvature',
    'line_start_alignment_enabled', 'line_start_alignment_min_lateral_speed',
    'line_start_alignment_lateral_speed',
}


def positive(value, name, allow_zero=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0 or (value == 0 and not allow_zero)):
        raise ValueError(f'{name} must be a finite {"nonnegative" if allow_zero else "positive"} number')


def node_parameters(document, name):
    section = document.get(name)
    if not isinstance(section, dict) or set(section) != {'ros__parameters'}:
        raise ValueError(f'{name} must contain only ros__parameters')
    params = section['ros__parameters']
    if not isinstance(params, dict) or not params:
        raise ValueError(f'{name}.ros__parameters must be a nonempty mapping')
    return dict(params)


def launch_arguments(values):
    return {name: str(value).lower() if isinstance(value, bool) else str(value)
            for name, value in values.items()}


def load_task_config(path):
    path = Path(path).expanduser().resolve(strict=True)
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    sections = {'system', 'tcp', 'thickness_detector', 'workflow', 'travel',
                'detection_ready', 'arm_joint_postures', 'camera_detection_task',
                'detect_controller', 'capture'}
    if not isinstance(document, dict) or set(document) != sections:
        raise ValueError('Task YAML requires exactly these sections: '+', '.join(sorted(sections)))
    system = document['system']
    if not isinstance(system, dict) or set(system) != SYSTEM_KEYS:
        raise ValueError('system keys do not match the production launch settings')
    for name in ('auto_start', 'launch_rviz', 'line_show_window', 'line_record_route',
                 'line_charging_mode', 'line_start_alignment_enabled'):
        if type(system[name]) is not bool:
            raise ValueError(f'system.{name} must be boolean')
    for name in ('arm_control_start_delay', 'line_processing_fps'):
        positive(system[name], 'system.'+name, allow_zero=True)
    for name in ('line_follow_speed', 'line_reverse_speed', 'line_reverse_route_max_curvature',
                 'line_start_alignment_min_lateral_speed', 'line_start_alignment_lateral_speed'):
        positive(system[name], 'system.'+name)
    for name in ('line_opencv_threads', 'line_marker_opencv_threads'):
        if type(system[name]) is not int or system[name] < -1:
            raise ValueError(f'system.{name} must be an integer >= -1')
    for name in ('robot_ip', 'local_ip'):
        ipaddress.IPv4Address(system[name])
    for name in ('line_camera_device', 'standard_route_file'):
        if not isinstance(system[name], str) or not system[name].strip():
            raise ValueError(f'system.{name} requires a nonempty path')
    if system['ros_discovery_range'] not in ('LOCALHOST', 'SUBNET', 'SYSTEM_DEFAULT', 'OFF'):
        raise ValueError('Invalid ros_discovery_range')
    if system['line_navigation_mode'] != 'replay' or system['line_record_route']:
        raise ValueError('Production task replays the standard route; recording belongs to route teaching')
    if system['line_charging_mode']:
        raise ValueError('Production task returns to the start without charging search')
    for name, keys in (
        ('tcp', {'enabled', 'enable_status_monitor', 'key_file', 'charging_status_layout'}),
        ('thickness_detector', {'transport', 'device', 'baudrate', 'data_directory'}),
    ):
        if not isinstance(document[name], dict) or set(document[name]) != keys:
            raise ValueError(f'Invalid {name} settings')
    for name in ('enabled', 'enable_status_monitor'):
        if type(document['tcp'][name]) is not bool:
            raise ValueError(f'tcp.{name} must be boolean')
    detector = document['thickness_detector']
    if detector['transport'] != 'serial' or not isinstance(detector['device'], str) or not detector['device']:
        raise ValueError('Thickness detector requires a serial device')
    if type(detector['baudrate']) is not int or detector['baudrate'] <= 0:
        raise ValueError('Thickness detector baudrate must be a positive integer')
    for name in ('arm_joint_postures', 'camera_detection_task', 'detect_controller'):
        params = node_parameters(document, name)
        for key, value in params.items():
            if key.endswith(('_timeout', '_sec', '_seconds')) or key == 'hold_seconds':
                positive(value, name+'.'+key, allow_zero=key in ('initial_settle_sec', 'ready_stable_sec'))
    camera = node_parameters(document, 'camera_detection_task')
    if (not isinstance(document['workflow'], dict)
            or document['workflow'].get('arm_postures_before_chassis') is not True):
        raise ValueError('Production workflow requires arm posture protection before chassis motion')
    if camera.get('auto_start') is not False:
        raise ValueError('camera_detection_task must be triggered by the workflow')
    if camera.get('status_topic') != '/camera_detection_task/status' or camera.get('stop_service') != '/camera_detection_task/stop':
        raise ValueError('Camera status/stop endpoints must match the workflow controller')
    if camera.get('target_source') not in ('local', 'tcp'):
        raise ValueError('camera target_source must be local or tcp')
    points = camera.get('camera_points_mm')
    if (not isinstance(points, list) or not points or len(points) % 3
            or not all(type(v) in (int, float) and math.isfinite(v) for v in points)):
        raise ValueError('camera_points_mm must contain finite xyz triples in millimetres')
    detect = node_parameters(document, 'detect_controller')
    if detect.get('touch_control_mode') != 'moveit_force_guard' or detect.get('require_thickness_measurement') is not True:
        raise ValueError('Production task requires force-guarded touch and thickness measurement')
    positive(detect.get('initial_force_limit'), 'initial_force_limit')
    positive(detect.get('contact_force'), 'contact_force')
    positive(detect.get('overload_force'), 'overload_force')
    if not detect['initial_force_limit'] < detect['contact_force'] < detect['overload_force']:
        raise ValueError('Require initial_force_limit < contact_force < overload_force')
    for name, params in (('arm_joint_postures', node_parameters(document, 'arm_joint_postures')),
                         ('detect_controller', detect)):
        for key in ('velocity_scaling', 'acceleration_scaling'):
            if key in params:
                positive(params[key], name+'.'+key)
                limit = .3 if name == 'arm_joint_postures' else 1.
                if params[key] > limit:
                    raise ValueError(f'{name}.{key} must be <= {limit}')
    if camera.get('joint_postures_enabled') is not False or detect.get('joint_postures_enabled') is not False:
        raise ValueError('Only the workflow may schedule joint postures')
    if camera.get('capture_local') is not True or detect.get('initial_position_enabled') is not False:
        raise ValueError('Workflow requires capture at its photo pose without a second initial move')
    return path, document
