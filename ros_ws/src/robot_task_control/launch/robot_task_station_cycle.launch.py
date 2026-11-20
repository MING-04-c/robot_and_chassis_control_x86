"""独立入口：终点倒车回中途拍照站，再前进终点检测并回充。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('robot_task_control')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([
            share, 'config', 'robot_task_station_cycle.yaml'])),
        DeclareLaunchArgument('validate_only', default_value='false', choices=['true', 'false']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            share, 'launch', 'robot_task_oline.launch.py'])), launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                'validate_only': LaunchConfiguration('validate_only'),
            }.items()),
    ])
