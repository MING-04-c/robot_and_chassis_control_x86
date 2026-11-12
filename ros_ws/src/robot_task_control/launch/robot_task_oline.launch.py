"""Kylin-paced photo/detect/home session using the shared motion/safety stack."""
from pathlib import Path
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from robot_task_control.oline_config import load_oline_config


def launch_setup(context):
    config = load_oline_config(LaunchConfiguration('config_file').perform(context))
    # A per-launch file is needed by the existing capture subprocess and workflow.
    with tempfile.NamedTemporaryFile(mode='w', suffix='_robot_task_oline.yaml', delete=False) as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
        recipe = Path(stream.name)
    def cleanup(_context):
        recipe.unlink(missing_ok=True)
        return []
    include = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('robot_task_control'), 'launch', 'robot_task.launch.py'])),
        launch_arguments={'config_file': str(recipe), 'oline_mode': 'true',
                          'validate_only': LaunchConfiguration('validate_only')}.items())
    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])), include]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([
            FindPackageShare('robot_task_control'), 'config', 'robot_task_oline.yaml'])),
        DeclareLaunchArgument('validate_only', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
