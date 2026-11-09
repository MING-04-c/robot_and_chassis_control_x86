"""One production entry point; all task settings come from robot_task.yaml."""

import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from robot_task_control.task_config import launch_arguments, load_task_config, node_parameters


def launch_setup(context):
    from eli_cs_robot_moveit_config.joint_postures import load_postures
    from robot_task_control.final_capture import load_config
    from robot_task_control.standard_route import inspect_standard
    from robot_task_control.workflow_controller import WorkflowController

    path, config = load_task_config(LaunchConfiguration('config_file').perform(context))
    # Finish validation before returning any action that can open a device.
    workflow = WorkflowController.load_workflow(path)
    load_postures(path)
    load_config(path)
    system = dict(config['system'])
    route = Path(system['standard_route_file']).expanduser()
    report = inspect_standard(json.loads(route.read_text(encoding='utf-8')))
    if not report['accepted']:
        raise ValueError('Standard route rejected: '+'; '.join(report['issues']))
    get_logger('robot_task').info(f'Configuration: {path}; workflow: {workflow["name"]}; steps: {len(workflow["steps"])}')
    if LaunchConfiguration('validate_only').perform(context).lower() == 'true':
        get_logger('robot_task').info('Validation only: no drivers, nodes or robot motion started')
        return []

    def include(package, filename, arguments, scoped=True):
        action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare(package), 'launch', filename])),
            launch_arguments=launch_arguments(arguments).items(),
        )
        return GroupAction([action], scoped=True) if scoped else action

    # Hardware readiness, velocity arbitration and force protection are mandatory.
    system.update({
        'workflow_file': str(path), 'capture_config_file': str(path), 'require_arm_ready': True,
        'check_devices_on_startup': True, 'joint_posture_only': False,
        'manage_line_stream': True, 'launch_admittance': False,
        'chassis_pid_enabled': False, 'use_velocity_mux': True,
        'launch_arm_executor': True, 'arm_use_cartesian_path': False,
        'launch_force_sensor': True, 'start_hk_camera': False,
        'start_image_transfer': False, 'start_line_camera': True,
        'prepare_line_camera_early': False,
    })
    posture = node_parameters(config, 'arm_joint_postures')
    posture.update(postures_file=str(path), simulation=False, manage_admittance=False)
    camera = node_parameters(config, 'camera_detection_task')
    camera['capture_config_file'] = str(path)
    detect = node_parameters(config, 'detect_controller')
    detect['joint_postures_file'] = str(path)
    actions = [
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', system['ros_discovery_range']),
        Node(package='robot_task_control', executable='arm_joint_postures',
             name='arm_joint_postures', output='screen', parameters=[posture]),
        # The connection gate dispatches its stack later; keep its launch scope alive.
        include('robot_task_control', 'internal/robot_system.launch.py', system, scoped=False),
        include('eli_cs_robot_serial', 'detactor.launch.py', {
            **config['thickness_detector'], 'auto_request': False,
        }),
        Node(package='detect_control', executable='detect_controller.py',
             name='detect_controller', output='screen',
             parameters=[PathJoinSubstitution([
                 FindPackageShare('detect_control'), 'config', 'detect.yaml']),
                 detect],
             sigterm_timeout='3.0', sigkill_timeout='2.0'),
        Node(package='robot_task_control', executable='camera_detection_task',
             name='camera_detection_task', output='screen', parameters=[camera],
             sigterm_timeout='3.0', sigkill_timeout='2.0'),
    ]
    tcp = dict(config['tcp'])
    if tcp.pop('enabled'):
        actions.append(include('eli_cs_robot_serial', 'industrial_pc.launch.py', {
            **tcp, 'start_hk_transfer': False,
        }))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([
            FindPackageShare('robot_task_control'), 'config', 'robot_task.yaml'])),
        DeclareLaunchArgument('validate_only', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
