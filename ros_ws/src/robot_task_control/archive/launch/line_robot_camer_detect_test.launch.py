"""Line-follow, camera capture, probe touch and thickness measurement test."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('robot_task_control')
    task_config = PathJoinSubstitution([
        share, 'archive', 'config', 'line_robot_camera_detection.yaml'])
    capture_config = PathJoinSubstitution([
        share, 'archive', 'config', 'capture', 'line_robot_local_capture.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('line_show_window', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'archive', 'launch', 'line_follow_arm_test.launch.py'])),
            launch_arguments={
                'test_profile': 'return_charge',
                'workflow_file': PathJoinSubstitution([
                    share, 'archive', 'config', 'line_robot_camer_detect_test.yaml']),
                'line_follow_speed': '0.1',
                'line_show_window': LaunchConfiguration('line_show_window'),
                'line_navigation_mode': 'replay',
                'standard_route_file': '~/.ros/line_follow/standards/route_01/standard_route.json',
                'line_record_route': 'false',
                'charging_mode': 'false',
                'line_opencv_threads': '-1',
                'line_marker_opencv_threads': '-1',
                'launch_force_sensor': 'true',
                'joint_posture_only': 'false',
                'manage_line_stream': 'true',
                'launch_arm_executor': 'true',
            }.items(),
        ),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    share, 'launch', 'internal', 'detection_system.launch.py'])),
                launch_arguments={
                    'simulation': 'false',
                    'start_test_task': 'false',
                    'start_arm_driver': 'false',
                    'start_arm_control': 'false',
                    'start_thickness_detector': 'true',
                    'launch_force_sensor': 'false',
                    'launch_rviz': 'false',
                    'touch_control_mode': 'moveit_force_guard',
                    'test_config': task_config,
                    'targets_file': '~/.ros/detect_control/line_robot_camera_targets.yaml',
                }.items(),
            ),
        ], scoped=True),
        Node(
            package='robot_task_control',
            executable='camera_detection_task',
            name='camera_detection_task',
            output='screen',
            parameters=[task_config, {
                'target_source': 'local',
                'capture_config_file': capture_config,
            }],
            sigterm_timeout='3.0',
            sigkill_timeout='2.0',
        ),
    ])
