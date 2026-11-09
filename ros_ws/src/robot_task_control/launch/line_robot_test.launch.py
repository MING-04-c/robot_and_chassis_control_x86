"""Frozen entry point for the verified line-follow/arm round trip."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('robot_task_control')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'line_follow_arm_test.launch.py'])),
            launch_arguments={
                'test_profile': 'return_charge',
                'workflow_file': PathJoinSubstitution([
                    share, 'config', 'line_robot_test.yaml']),
                'line_follow_speed': '0.1',
            }.items(),
        ),
    ])
