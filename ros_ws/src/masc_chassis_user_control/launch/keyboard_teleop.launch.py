"""键盘控制：自带底盘驱动，默认显示底盘相机，不启动自动循迹或距离任务。

ros2 launch masc_chassis_user_control keyboard_teleop.launch.py linear_speed:=0.05
W/S 前后，A/D 左右横移，Q/E 左右旋转，空格停车，+/- 调速，Ctrl+C退出。
默认0.6秒无移动按键停车；终端按键依赖长按连发，没有直接的松键事件。
已有独立驱动时 start_driver:=false；使用前关闭循迹及其他速度控制器。
show_window:=false 关闭预览及其相机；相机窗口不接收遥控按键。
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    driver = get_package_share_directory('masc_chassis_ros_driver')
    try:
        terminal = os.ttyname(sys.stdin.fileno())
    except (OSError, ValueError):
        terminal = ''
    return LaunchDescription([
        DeclareLaunchArgument('start_driver', default_value='true',
                              description='一起启动底盘驱动；已有驱动时false'),
        DeclareLaunchArgument('show_window', default_value='true', choices=['true', 'false'],
                              description='显示底盘相机预览；false时不启动相机'),
        DeclareLaunchArgument('camera_device', default_value=(
            '/dev/v4l/by-id/'
            'usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'),
            description='底盘USB相机设备，不自动回退到笔记本摄像头'),
        DeclareLaunchArgument('linear_speed', default_value='0.075', description='平移速度m/s，0.01～0.3'),
        DeclareLaunchArgument('angular_speed', default_value='0.2', description='旋转速度rad/s，0.05～0.8'),
        DeclareLaunchArgument('key_timeout', default_value='0.6', description='无移动按键后停车秒数'),
        DeclareLaunchArgument('terminal_device', default_value=terminal, description='自动取当前交互终端'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('driver_params', default_value=os.path.join(
            driver, 'config', 'ros2', 'driver_params.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(driver, 'launch', 'masc_chassis_ros_driver.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_driver')),
            launch_arguments={
                'enable_pid': 'false', 'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'driver_params': LaunchConfiguration('driver_params'),
            }.items()),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ruier_camer_diver'),
                    'launch', 'line_tracking.launch.py')),
                launch_arguments={
                    'namespace': 'line_camera', 'demo': 'false', 'show_window': 'true',
                    'video_device': LaunchConfiguration('camera_device'),
                    'camera_frame': 'line_camera_optical_frame',
                    'max_processing_fps': '8.0', 'publish_compressed': 'false',
                }.items()),
        ], condition=IfCondition(LaunchConfiguration('show_window')), scoped=True),
        Node(package='masc_chassis_user_control', executable='keyboard_teleop_node.py',
             name='keyboard_teleop', output='screen',
             on_exit=[EmitEvent(event=Shutdown(reason='Keyboard controller exited'))],
             parameters=[{
                 'terminal_device': LaunchConfiguration('terminal_device'),
                 'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                 'linear_speed': ParameterValue(LaunchConfiguration('linear_speed'), value_type=float),
                 'angular_speed': ParameterValue(LaunchConfiguration('angular_speed'), value_type=float),
                 'key_timeout': ParameterValue(LaunchConfiguration('key_timeout'), value_type=float),
             }]),
    ])
