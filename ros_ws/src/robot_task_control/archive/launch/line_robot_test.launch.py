"""Frozen entry point for the verified line-follow/arm round trip."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('robot_task_control')
    return LaunchDescription([
        DeclareLaunchArgument('line_show_window', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'archive', 'launch', 'line_follow_arm_test.launch.py'])),
            launch_arguments={
                'test_profile': 'return_charge',
                'workflow_file': PathJoinSubstitution([
                    share, 'archive', 'config', 'line_robot_test.yaml']),
                'line_follow_speed': '0.1',
                'line_show_window': LaunchConfiguration('line_show_window'),
                'line_navigation_mode': 'replay',
                'standard_route_file': '~/.ros/line_follow/standards/route_01/standard_route.json',
                'line_record_route': 'false',
                'charging_mode': 'false',
            }.items(),
        ),
    ])
