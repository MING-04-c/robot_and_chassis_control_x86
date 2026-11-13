"""Isolated, planning-only preview: no robot driver or trajectory executor."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from eli_cs_robot_moveit_config.launch_common import load_yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown
from launch_ros.actions import Node


PREVIEW_ELBOW_LIMIT_DEG = 250.0


def preview_joint_limits(urdf):
    """User-requested planning-only envelope; never alter the shared hardware model."""
    model = ET.fromstring(urdf)
    elbow = model.find("joint[@name='elbow_joint']")
    if elbow is None or elbow.find('limit') is None:
        raise ValueError('Preview model is missing elbow_joint limits')
    limit = math.radians(PREVIEW_ELBOW_LIMIT_DEG)
    elbow.find('limit').set('lower', str(-limit))
    elbow.find('limit').set('upper', str(limit))
    safety = elbow.find('safety_controller')
    if safety is not None:
        # Match the requested EFFECTIVE +/-250 degree preview range, not 250-8.59.
        safety.set('soft_lower_limit', str(-limit))
        safety.set('soft_upper_limit', str(limit))
    return ET.tostring(model, encoding='unicode')


def launch_setup(context):
    share = Path(get_package_share_directory("robot_task_control"))
    description = Path(get_package_share_directory("eli_cs_robot_description"))
    moveit = Path(get_package_share_directory("eli_cs_robot_moveit_config"))
    urdf = xacro.process_file(str(description / "urdf/cs.urdf.xacro"), mappings={
        "cs_type": "ls65", "name": "cs", "use_fake_hardware": "true",
        "attach_forearm_accessory": "true",
    }).toxml()
    urdf = preview_joint_limits(urdf)
    srdf = xacro.process_file(str(moveit / "srdf/cs.srdf.xacro"), mappings={
        "name": "cs", "prefix": "", "attach_forearm_accessory": "true",
    }).toxml()
    model = {"robot_description": urdf, "robot_description_semantic": srdf}
    ompl = load_yaml("moveit_configs_utils", "default_configs/ompl_planning.yaml")
    ompl.update(load_yaml("eli_cs_robot_moveit_config", "config/ompl_planning.yaml"))
    limits = load_yaml("eli_cs_robot_moveit_config", "config/joint_limits.yaml")
    planning = {
        "robot_description_planning": limits,
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl,
        "allow_trajectory_execution": False,
        "disable_capabilities": "move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction",
        "publish_planning_scene": True, "publish_geometry_updates": True,
        "publish_state_updates": True, "publish_transforms_updates": True,
        "publish_robot_description": True, "publish_robot_description_semantic": True,
        "use_sim_time": False,
    }
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}], output="log", sigterm_timeout="2", sigkill_timeout="2"),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[model, str(moveit / "config/kinematics.yaml"), planning], sigterm_timeout="2", sigkill_timeout="2"),
        Node(package="eli_cs_robot_moveit_config", executable="ground_scene.py", output="screen",
             parameters=[{"base_frame": "base_link", "ground_height": 0.08,
                          "fixed_mount_links": ["base_link", "base_link_inertia"]}], sigterm_timeout="2", sigkill_timeout="2"),
        Node(package="robot_task_control", executable="path_preview.py", output="screen",
             parameters=[{"config_file": LaunchConfiguration("config_file"),
                          "elbow_joint_limit_deg": PREVIEW_ELBOW_LIMIT_DEG,
                          "workflow_file": LaunchConfiguration("workflow_file"),
                          "detection_orientation_mode": LaunchConfiguration("detection_orientation_mode", default="")}], sigterm_timeout="2", sigkill_timeout="2"),
        Node(package="rviz2", executable="rviz2", name="path_preview_rviz", output="log",
             arguments=["-d", str(share / "simulation/path_preview.rviz")],
             parameters=[model, str(moveit / "config/kinematics.yaml")],
             condition=IfCondition(LaunchConfiguration("rviz")), sigterm_timeout="2", sigkill_timeout="2",
             on_exit=[EmitEvent(event=Shutdown(reason="Preview window closed"))]),
    ]


def generate_launch_description():
    simulation = Path(get_package_share_directory("robot_task_control")) / "simulation"
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(simulation / "path_preview.yaml")),
        DeclareLaunchArgument("workflow_file", default_value=""),
        DeclareLaunchArgument("detection_orientation_mode", default_value="",
                             choices=["", "axis_only"],
                             description="Only photo-tool Z axis is constrained at precontact"),
        DeclareLaunchArgument("rviz", default_value="true"),
        GroupAction(scoped=True, actions=[
            SetEnvironmentVariable("ROS_DOMAIN_ID", "190"),
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
            SetEnvironmentVariable("ROS_STATIC_PEERS", ""),
            SetEnvironmentVariable("ROS_DISCOVERY_SERVER", ""),
            OpaqueFunction(function=launch_setup),
        ]),
    ])
