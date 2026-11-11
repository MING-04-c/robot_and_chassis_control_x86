#!/usr/bin/env python3
"""Check launch profiles and startup order without opening robot hardware."""
import os
import yaml
from pathlib import Path
import xml.etree.ElementTree as ET

os.environ.setdefault("ROS_LOG_DIR", "/tmp/arm-description-launch-test")

import launch_ros.actions
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.utilities import evaluate_parameters, normalize_parameters
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from rclpy.utilities import remove_ros_args

root = Path(__file__).resolve().parents[2]
shares = {
    ET.parse(path).getroot().findtext("name"): path.parent
    for path in root.glob("*/package.xml")
}
# Serial launch files are flattened from device/sensor during installation.
shares.pop("eli_cs_robot_serial", None)
seen = []
gate_exit_code = 0
power_exit_code = 0
failed_controller_name = None
original_node = launch_ros.actions.Node
original_find = FindPackageShare.find


def fake_node(**kwargs):
    def inspect(context):
        params = evaluate_parameters(
            context, normalize_parameters(kwargs.get("parameters", []))
        )
        values = {}
        for item in params:
            if isinstance(item, Path):
                # Validate the actual ROS parameter grammar, not just YAML syntax.
                remove_ros_args(args=['parameter_test', '--ros-args', '--params-file', str(item)])
            if isinstance(item, dict):
                values.update(item)
            elif isinstance(item, Path) and kwargs.get('name') in ('camera_detection_task', 'detect_controller'):
                configuration = yaml.safe_load(item.read_text()) or {}
                values.update(configuration.get(kwargs['name'], {}).get('ros__parameters', {}))
        values['dds_discovery_range'] = context.environment.get('ROS_AUTOMATIC_DISCOVERY_RANGE')
        values['parameter_files'] = [str(item) for item in params if isinstance(item, Path)]
        if kwargs.get('executable') in ('spawner', 'ros2_control_node'):
            values['arguments'] = [perform_substitutions(context,
                normalize_to_list_of_substitutions(value)) for value in kwargs.get('arguments', [])]
        seen.append((kwargs.get("executable"), values))
        return []

    if kwargs.get('executable') == 'spawner':
        class FakeSpawner(ExecuteProcess):
            def execute(self, context):
                inspect(context)
                return super().execute(context)
        first_name = str(kwargs['arguments'][0])
        return FakeSpawner(cmd=['/bin/false' if first_name == failed_controller_name else '/bin/true'])
    if kwargs.get('executable') == 'auto_power_on.py':
        class FakePowerGate(ExecuteProcess):
            def execute(self, context):
                inspect(context)
                return super().execute(context)
        return FakePowerGate(cmd=['/bin/true' if power_exit_code == 0 else '/bin/false'])
    if kwargs.get("executable") == "auto_admittance_start.py" and any(
        isinstance(params, dict) and params.get("wait_only") is True
        for params in kwargs.get("parameters", [])
    ):
        # Exercise the actual parent OnProcessExit wiring, without ROS or hardware.
        class FakeConnectionGate(ExecuteProcess):
            def execute(self, context):
                if not any(name == 'ros2_control_node' for name, _ in seen):
                    return [TimerAction(period=.02, actions=[self])]
                inspect(context)
                return super().execute(context)

        return FakeConnectionGate(
            cmd=["/bin/true" if gate_exit_code == 0 else "/bin/false"]
        )
    return OpaqueFunction(function=inspect, condition=kwargs.get("condition"))


def find_share(self, name):
    return str(shares[name]) if name in shares else original_find(self, name)



def run_entry(path, arguments=None, shutdown_reasons=None):
    seen.clear()
    service = LaunchService()
    actions = []
    if shutdown_reasons is not None:
        def on_shutdown(event, context):
            shutdown_reasons.append(event.reason)
            return []
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=on_shutdown)))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=(arguments or {}).items()))
    service.include_launch_description(LaunchDescription(actions))
    return service.run()


try:
    launch_ros.actions.Node = fake_node
    FindPackageShare.find = find_share
    entry = root/'robot_task_control/launch/robot_task.launch.py'
    config = yaml.safe_load((root/'robot_task_control/config/robot_task.yaml').read_text())
    assert run_entry(entry) == 0
    by_executable = {name: values for name, values in seen}
    driver = by_executable["ros2_control_node"]["robot_description"]
    publisher = by_executable["robot_state_publisher"]["robot_description"]
    assert driver == publisher
    order = [name for name, _ in seen]
    assert order.index('eli_components_loader') < order.index('auto_power_on.py') < order.index('ros2_control_node')
    hardware = ET.fromstring(driver).find("ros2_control/hardware")
    params = {item.attrib["name"]: item.text for item in hardware.findall("param")}
    assert params["headless_mode"].lower() == "true", params
    assert params["robot_ip"] == "169.254.199.150", params
    assert params["local_ip"] == config["system"]["local_ip"], params
    planning = by_executable["move_group"]
    assert planning["publish_robot_description"] is False, planning
    # The planner's URDF is valid for planning but has different hardware fields.
    # It must not be offered to controller_manager as a competing description.
    assert planning["robot_description"] != driver
    # 真机与规划器的硬件字段不同，但末端几何和安装关系必须一致。
    descriptions = [ET.fromstring(driver), ET.fromstring(planning["robot_description"])]
    terminal_signatures = []
    for description in descriptions:
        signatures = []
        assert description.find("link[@name='terminal_base_link']") is not None
        for name in ("terminal_assembly_link", "terminal_link_1", "terminal_link_2"):
            link = description.find(f"link[@name='{name}']")
            assert link is not None, name
            for kind in ("visual", "collision"):
                mesh = link.find(f"{kind}/geometry/mesh")
                assert mesh is not None and "/meshes/terminal/" in mesh.attrib["filename"]
                signatures.append((name, kind, tuple(sorted(mesh.attrib.items()))))
        for name in (
            "tool0-terminal_base_fixed",
            "terminal_assembly_fixed",
            "terminal_joint_1",
            "terminal_joint_2",
        ):
            joint = description.find(f"joint[@name='{name}']")
            assert joint is not None and joint.attrib["type"] == "fixed"
            signatures.append(
                (
                    name,
                    joint.find("parent").attrib,
                    joint.find("child").attrib,
                    joint.find("origin").attrib,
                )
            )
        terminal_signatures.append(signatures)
    assert terminal_signatures[0] == terminal_signatures[1]
    production = by_executable
    task = production['task_controller']
    assert task['workflow_file'].endswith('/config/robot_task.yaml')
    assert task['capture_config_file'].endswith('/config/robot_task.yaml')
    assert task['auto_start'] is config['system']['auto_start']
    assert task['require_arm_ready'] is task['check_devices_on_startup'] is True
    assert task['manage_line_stream'] is True
    follower = production['line_follow_node.py']
    assert follower['navigation_mode'] == 'replay' and follower['record_route'] is False
    assert follower['forward_speed'] == config['system']['line_follow_speed']
    assert follower['reverse_max_speed'] == config['system']['line_reverse_speed']
    assert follower['show_window'] is config['system']['line_show_window']
    camera = production['camera_detection_task']
    assert camera['capture_config_file'].endswith('/config/robot_task.yaml')
    assert camera['auto_start'] is False
    assert production['detect_controller.py']['hold_seconds'] == config['detect_controller']['ros__parameters']['hold_seconds']
    assert production['serial_node']['device'] == config['thickness_detector']['device']
    for executable in ('detactor_node', 'serial_node'):
        assert len(production[executable]['parameter_files']) == 1
        assert production[executable]['parameter_files'][0].endswith('/device/detactor/config/detactor.yaml')
    assert production['industrial_pc_node.py']['parameter_files'][0].endswith(
        '/device/industrial_pc/config/industrial_pc.yaml')
    assert production['arm_joint_postures']['initial_settle_sec'] == config['arm_joint_postures']['ros__parameters']['initial_settle_sec']
    assert production['auto_admittance_start.py']['settle_seconds'] == config['system']['arm_control_start_delay']
    order = [name for name, _ in seen]
    assert order.index('ros2_control_node') < order.index('auto_admittance_start.py') < order.index('task_controller')
    assert 'admittance_pose_controller' not in production

    # Arrival diagnostics reuse all arm protections without claiming or driving the chassis.
    station_file = root/'robot_task_control/config/diagnostics/arrival_detect.yaml'
    assert run_entry(entry, {'station_workflow_file': str(station_file)}) == 0
    station_nodes = {name: values for name, values in seen}
    assert not {'masc_chassis_ros_driver_node', 'line_follow_node.py', 'camera_node',
                'line_tracker_node', 'velocity_mux'} & set(station_nodes)
    assert {'protective_stop_guard.py', 'ros2_control_node', 'move_group', 'ground_scene.py',
            'moveit_pose_executor', 'sensor_node', 'detactor_node', 'serial_node',
            'camera_detection_task', 'detect_controller.py'} <= set(station_nodes)
    assert station_nodes['task_controller']['workflow_file'] == str(station_file)
    assert station_nodes['task_controller']['auto_start'] is False
    assert station_nodes['task_controller']['require_arm_ready'] is True
    assert station_nodes['arm_joint_postures']['postures_file'] == str(station_file)

    # Connection failure must cancel the task rather than continuing with motion.
    gate_exit_code = 1
    assert run_entry(entry) == 0
    failed = {name for name, _ in seen}
    assert 'ros2_control_node' in failed
    assert not {'task_controller', 'move_group', 'camera_node'} & failed
    gate_exit_code = 0

    power_exit_code = 1
    assert run_entry(entry) == 0
    failed = {name for name, _ in seen}
    assert 'auto_power_on.py' in failed
    assert not {'ros2_control_node', 'task_controller'} & failed
    power_exit_code = 0

    # Required controller failures still shut down the common driver stack.
    for failure in ('joint_state_broadcaster', 'scaled_joint_trajectory_controller', 'freedrive_controller'):
        failed_controller_name = failure
        reasons = []
        run_entry(root/'eli_cs_robot_driver/launch/elite_control.launch.py',
                  {'auto_power_on': 'false'}, reasons)
        assert f'Controller startup failed: {failure}' in reasons
    failed_controller_name = None

    assert run_entry(entry, {'validate_only': 'true'}) == 0
    assert not seen, 'Validation-only mode must not construct or start nodes'
    print('PASS: single task wiring, model ownership, startup order and failure cancellation')
finally:
    launch_ros.actions.Node = original_node
    FindPackageShare.find = original_find
