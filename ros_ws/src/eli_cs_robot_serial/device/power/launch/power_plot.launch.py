"""订阅已运行的底盘BMS，显示功率折线图；不启动运动任务。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bms_topic', default_value='/masc_chassis/bms_status'),
        DeclareLaunchArgument('power_topic', default_value='/power/watts'),
        DeclareLaunchArgument('window_seconds', default_value='120.0'),
        DeclareLaunchArgument('stale_timeout', default_value='2.0'),
        DeclareLaunchArgument('show_plot', default_value='true'),
        Node(package='eli_cs_robot_serial', executable='power_monitor.py',
             name='power_monitor', output='screen', parameters=[{
                 'bms_topic': LaunchConfiguration('bms_topic'),
                 'power_topic': LaunchConfiguration('power_topic'),
                 'window_seconds': ParameterValue(LaunchConfiguration('window_seconds'), value_type=float),
                 'stale_timeout': ParameterValue(LaunchConfiguration('stale_timeout'), value_type=float),
                 'show_plot': ParameterValue(LaunchConfiguration('show_plot'), value_type=bool),
             }]),
    ])
