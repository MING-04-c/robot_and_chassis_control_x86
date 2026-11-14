"""Simulation configuration/continuity tests; no hardware or ROS graph required."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
from geminipro_camer_calibration.geometry import rpy_degrees_rotation, spin_tool_pose
import pytest
import yaml
from launch import LaunchContext
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


ROOT = Path(__file__).resolve().parents[1]


def module(name, file):
    spec = importlib.util.spec_from_file_location(name, ROOT / "simulation" / file)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


preview = module("path_preview_test", "path_preview.py")


def trajectory(start, end):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = list(reversed(preview.JOINTS))
    result.joint_trajectory.points = [JointTrajectoryPoint(positions=list(reversed(p))) for p in (start, end)]
    return result


def test_preview_reuses_workflow_order_with_simulation_only_targets():
    config, path, initial, postures, steps = preview.load_preview(ROOT / "simulation/path_preview.yaml")
    assert Path(path) == ROOT / "config/diagnostics/arrival_detect.yaml"
    assert initial == postures["travel"]
    assert math.degrees(initial[0]) == -45.0
    assert [s["name"] for s in steps if s["action"] in ("arm_posture", "arm_move")] == [
        "to_transition", "to_transition_2", "to_camera_via", "to_camera_intermediate", "to_photo_pose",
        "return_camera_intermediate", "return_camera_via", "return_transition_2", "return_transition", "return_travel"]
    by_name = {step['name']:step for step in steps}
    assert by_name['capture_and_thickness']["action"] == "camera_detection"
    for name in ('to_camera_via', 'to_camera_intermediate'):
        assert by_name[name]['target_pose']['position'] == config['waypoint_poses'][name]['position']
        assert by_name[name]['target_pose']['orientation'] == config['waypoint_poses'][name]['orientation']
    assert by_name['return_camera_intermediate']['target_pose'] == by_name['to_camera_intermediate']['target_pose']
    assert by_name['return_camera_via']['target_pose'] == by_name['to_camera_via']['target_pose']
    assert config["velocity_scaling"] == 0.1
    assert config["planning_pipeline"] == "ompl"
    assert config["planner_id"] == "RRTConnectkConfigDefault"
    assert config['detect_controller']['preposition_tool_z_spin'] is True
    assert config['detect_controller']['approach_clearance'] == config['approach_clearance']
    assert config['detection_orientation_mode'] == 'axis_only'
    assert by_name['to_photo_pose']['target_pose']['position'] == [.240, .390, .580]
    assert by_name['to_photo_pose']['target_pose']['orientation'] == [-180., 0., -90.]


def test_pose_uses_yaml_units_and_named_tool():
    _, _, _, _, steps = preview.load_preview(ROOT / "simulation/path_preview.yaml")
    constraints = preview.pose_constraints(next(step for step in steps if step['name']=='to_photo_pose'))
    position, orientation = constraints.position_constraints[0], constraints.orientation_constraints[0]
    assert position.link_name == "tool0" and position.header.frame_id == "base_link"
    assert position.constraint_region.primitive_poses[0].position.x == 0.240
    assert list(position.constraint_region.primitives[0].dimensions) == [0.005]
    assert orientation.absolute_x_axis_tolerance == pytest.approx(math.pi / 180)
    q = orientation.orientation
    assert q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w == pytest.approx(1.0)


@pytest.mark.parametrize("value", [[1, 2], [True, 2, 3], [1, math.nan, 3], [1, math.inf, 3]])
def test_invalid_pose_rejected(value):
    with pytest.raises(ValueError):
        preview.finite_vector(value, 3, "target")


def test_segment_end_reordered_without_wrapping_and_continuity_checked():
    start, end = [0.0]*6, [7.0, -1.0, 0.0, 0.0, 0.0, 0.0]
    result = trajectory(start, end)
    assert preview.trajectory_end(start, result) == end
    with pytest.raises(RuntimeError, match="previous segment"):
        preview.trajectory_end([0.2]*6, result)
    with pytest.raises(RuntimeError, match="empty trajectory"):
        preview.trajectory_end(start, RobotTrajectory())


def test_unknown_action_is_not_silently_skipped(tmp_path):
    data = yaml.safe_load((ROOT / "config/diagnostics/arrival_detect.yaml").read_text())
    data["workflow"]["steps"][0]["action"] = "line_follow"
    workflow = tmp_path / "workflow.yaml"
    workflow.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="Unsupported preview action"):
        preview.load_preview(ROOT / "simulation/path_preview.yaml", str(workflow))


def test_unisolated_runner_refuses_to_initialize():
    with patch.dict(preview.os.environ, {"ROS_DOMAIN_ID": "0", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"}):
        with pytest.raises(RuntimeError, match="isolated domain"):
            preview.PathPreview()


def test_rviz_only_shows_model_ground_and_trajectory():
    data = yaml.safe_load((ROOT / "simulation/path_preview.rviz").read_text())
    displays = data["Visualization Manager"]["Displays"]
    assert [d["Class"] for d in displays] == [
        "rviz_default_plugins/Grid", "moveit_rviz_plugin/PlanningScene", "moveit_rviz_plugin/Trajectory"]
    assert displays[-1]["Loop Animation"] is False
    assert displays[-1]["State Display Time"] == "0.1 s"
    assert displays[1]["Scene Robot"]["Show Robot Visual"] is False
    assert displays[-1]["Trajectory Topic"] == "/path_preview/display_planned_path"
    source = (ROOT / "simulation/path_preview.py").read_text()
    assert 'self.create_publisher(DisplayTrajectory, "/path_preview/display_planned_path", qos)' in source


def test_rviz_keeps_native_trajectory_slider_visible_after_single_playback():
    data = yaml.safe_load((ROOT / "simulation/path_preview.rviz").read_text())
    geometry = data["Window Geometry"]
    assert geometry["Planned Motion - Trajectory Slider"]["collapsed"] is False
    assert "Planned Motion - Trajectory Slider".encode("utf-16-be").hex() in geometry["QMainWindow State"]
    view = data["Visualization Manager"]["Views"]["Current"]
    assert view["Distance"] == 2.2
    assert view["Pitch"] == .4
    assert view["Focal Point"]["Z"] == .4


def test_launch_has_only_planning_nodes_and_fake_model():
    launch = module("path_preview_launch_test", "path_preview.launch.py")
    context = LaunchContext()
    context.launch_configurations.update(config_file="unused", workflow_file="", rviz="false")
    with patch.object(launch, "Node", side_effect=lambda **kwargs: kwargs):
        nodes = launch.launch_setup(context)
    assert [n["executable"] for n in nodes] == [
        "robot_state_publisher", "move_group", "ground_scene.py", "path_preview.py", "rviz2"]
    assert nodes[1]["parameters"][2]["allow_trajectory_execution"] is False
    assert nodes[1]["parameters"][2]["default_planning_pipeline"] == "ompl"
    urdf = nodes[0]["parameters"][0]["robot_description"]
    assert "mock_components/GenericSystem" in urdf
    assert "terminal_link_1" in urdf and "terminal_link_2" in urdf
    assert "forearm_accessory" in urdf
    source = (ROOT / "simulation/path_preview.launch.py").read_text()
    assert 'SetEnvironmentVariable("ROS_DOMAIN_ID", "190")' in source
    assert 'SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")' in source
    runner = (ROOT / "simulation/path_preview.py").read_text()
    assert "ActionClient" not in runner and "ExecuteTrajectory" not in runner


def test_elbow_limit_override_is_simulation_only_and_keeps_other_model_limits():
    import xml.etree.ElementTree as ET
    launch = module('preview_elbow_limits_test', 'path_preview.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(config_file='unused', workflow_file='', rviz='false')
    with patch.object(launch, 'Node', side_effect=lambda **kwargs:kwargs):
        nodes = launch.launch_setup(context)
    modified = nodes[0]['parameters'][0]['robot_description']
    assert nodes[1]['parameters'][0]['robot_description'] == modified
    assert nodes[4]['parameters'][0]['robot_description'] == modified
    description = Path(launch.get_package_share_directory('eli_cs_robot_description'))/'urdf/cs.urdf.xacro'
    before = description.read_bytes()
    original = launch.xacro.process_file(str(description), mappings=dict(
        cs_type='ls65', name='cs', use_fake_hardware='true', attach_forearm_accessory='true')).toxml()
    old = {joint.get('name'):joint for joint in ET.fromstring(original).findall('joint')}
    for joint in ET.fromstring(modified).findall('joint'):
        if joint.get('name') != 'elbow_joint':
            for tag in ('limit', 'safety_controller'):
                actual, expected = joint.find(tag), old[joint.get('name')].find(tag)
                if actual is None:
                    assert expected is None
                else:
                    assert expected is not None and actual.attrib == expected.attrib
            continue
        assert math.degrees(float(joint.find('limit').get('lower'))) == pytest.approx(-250.)
        assert math.degrees(float(joint.find('limit').get('upper'))) == pytest.approx(250.)
        assert math.degrees(float(joint.find('safety_controller').get('soft_upper_limit'))) == pytest.approx(250.)
        assert math.degrees(float(joint.find('safety_controller').get('soft_lower_limit'))) == pytest.approx(-250.)
    # 用户可修改原始模型限位；预览不得回写模型或其引用的限位配置。
    after = launch.xacro.process_file(str(description), mappings=dict(
        cs_type='ls65', name='cs', use_fake_hardware='true', attach_forearm_accessory='true')).toxml()
    assert after == original
    assert description.read_bytes() == before
    assert nodes[1]['parameters'][2]['allow_trajectory_execution'] is False


def test_failure_keeps_prefix_and_does_not_request_later_segments():
    config, _, initial, postures, steps = preview.load_preview(ROOT / "simulation/path_preview.yaml")
    first = trajectory(initial, postures["detection_ready"])
    good = SimpleNamespace(motion_plan_response=SimpleNamespace(
        error_code=MoveItErrorCodes(val=1), trajectory=first, planning_time=0.01))
    bad = SimpleNamespace(motion_plan_response=SimpleNamespace(error_code=MoveItErrorCodes(val=-1)))
    node = SimpleNamespace(config=config, initial=initial, postures=postures, steps=steps,
        ground=True, planner=Mock(), validity=Mock(), ik=Mock(), fk=Mock(), cartesian=Mock(),
        animation=SimpleNamespace(trajectory=[]), report={"segments": [], "skipped": []},
        save_report=Mock(), get_logger=Mock(), validate=Mock(), publish_paths=Mock(),
        nearby_ik=Mock(return_value=[0.0]*6),
        call=Mock(side_effect=[good, bad]))
    node.plan_motion = lambda motion, segment: preview.PathPreview.plan_motion(node, motion, segment)
    node.plan_step = lambda step, current, segment=None: preview.PathPreview.plan_step(node, step, current, segment)
    with patch.object(preview.rclpy, "ok", return_value=True), patch.object(preview.rclpy, "spin_once"):
        with pytest.raises(RuntimeError, match="to_transition_2"):
            preview.PathPreview.run(node)
    assert node.call.call_count == 2
    assert node.animation.trajectory == [first]
    assert [s["status"] for s in node.report["segments"]] == ["PLANNED", "FAILED"]
    second_request = node.call.call_args_list[1].args[1]
    assert list(second_request.motion_plan_request.start_state.joint_state.position) == postures["detection_ready"]
    assert second_request.motion_plan_request.pipeline_id == "ompl"
    assert second_request.motion_plan_request.planner_id == "RRTConnectkConfigDefault"
    goal = second_request.motion_plan_request.goal_constraints[0]
    assert len(goal.joint_constraints) == 6
    assert not goal.position_constraints and not goal.orientation_constraints
    assert [j.position for j in goal.joint_constraints] == postures['detection_ready_2']
    node.report["status"] = "FAILED"
    preview.PathPreview.finish_display(node)
    assert node.preview_ready
    assert len(node.animation.trajectory) == 1
    assert list(node.animation.trajectory[0].joint_trajectory.points[-1].positions) == postures["detection_ready"]
    assert node.report["status"] == "FAILED"
    assert node.report["display_status"] == "PARTIAL_STOP"
    node.publish_paths.assert_called_once()


def test_merge_is_single_continuous_time_monotonic_trajectory():
    a, b, c = [0.0]*6, [0.1]*6, [0.2]*6
    first, second = trajectory(a, b), trajectory(b, c)
    for segment in (first, second):
        segment.joint_trajectory.points[-1].time_from_start = Duration(sec=2)
    merged = preview.merge_trajectories([first, second])
    points = merged.joint_trajectory.points
    assert list(merged.joint_trajectory.joint_names) == preview.JOINTS
    assert [list(p.positions) for p in points] == [a, b, c]
    assert [p.time_from_start.sec for p in points] == [0, 2, 4]
    with pytest.raises(ValueError, match="Discontinuous"):
        preview.merge_trajectories([first, trajectory(a, c)])


def test_animation_is_not_republished_each_timer_tick():
    node = SimpleNamespace(preview_ready=True, animation=SimpleNamespace(trajectory=[Mock()]),
                           last_subscribers=-1, subscriber_ready_at=None, display=Mock())
    node.display.get_subscription_count.return_value = 1
    with patch.object(preview, 'monotonic', return_value=10.0):
        for _ in range(10):
            preview.PathPreview.publish_paths(node)
    node.display.publish.assert_not_called()
    with patch.object(preview, 'monotonic', return_value=12.0):
        preview.PathPreview.publish_paths(node)
    with patch.object(preview, 'monotonic', return_value=100.0):
        for _ in range(10):
            preview.PathPreview.publish_paths(node)
    assert node.display.publish.call_count == 1
    node.display.get_subscription_count.return_value = 0
    preview.PathPreview.publish_paths(node)
    node.display.get_subscription_count.return_value = 1
    with patch.object(preview, 'monotonic', side_effect=[200.0, 202.0]):
        preview.PathPreview.publish_paths(node)
        preview.PathPreview.publish_paths(node)
    assert node.display.publish.call_count == 2


def test_animation_waits_for_subscriber_startup_without_periodic_restarts():
    node = SimpleNamespace(preview_ready=True, animation=SimpleNamespace(trajectory=[Mock()]),
                           last_subscribers=-1, subscriber_ready_at=None, display=Mock())
    node.display.get_subscription_count.return_value = 1
    with patch.object(preview, 'monotonic', side_effect=[10.0, 10.5, 12.9, 13.0]):
        for _ in range(4):
            preview.PathPreview.publish_paths(node)
    assert node.display.publish.call_count == 1
    assert node.subscriber_ready_at is None


def test_finish_display_merges_segments_and_preserves_initial_state():
    start, middle, end = [0.0]*6, [0.1]*6, [0.2]*6
    node = SimpleNamespace(initial=start, preview_ready=False, report={"status": "PREVIEW_READY"},
        animation=SimpleNamespace(trajectory=[trajectory(start, middle), trajectory(middle, end)]),
        publish_paths=Mock(), save_report=Mock())
    preview.PathPreview.finish_display(node)
    assert len(node.animation.trajectory) == 1
    points = node.animation.trajectory[0].joint_trajectory.points
    assert [list(p.positions) for p in points] == [start, middle, end]
    assert node.initial == start and node.preview_ready
    assert node.report["display_status"] == "COMPLETE"
    node.publish_paths.assert_called_once()


def test_finish_display_does_not_publish_when_first_segment_failed():
    node = SimpleNamespace(preview_ready=False, report={"status": "FAILED"},
                           animation=SimpleNamespace(trajectory=[]), publish_paths=Mock())
    preview.PathPreview.finish_display(node)
    assert not node.preview_ready
    node.publish_paths.assert_not_called()


def test_probe_preview_has_precontact_nominal_contact_retract_and_photo_return():
    config, _, photo, _, _ = preview.load_preview(ROOT / "simulation/path_preview.yaml")
    camera, probe, tool = [np.eye(4) for _ in range(3)]
    camera[2, 3], probe[2, 3], tool[2, 3] = 0.5, 0.4, 0.3
    steps = preview.detection_steps(config, (camera, probe, tool), photo)
    assert [s["name"] for s in steps] == [
        "detect_1_precontact", "detect_1_nominal_contact", "detect_1_retract", "detect_1_return_photo"]
    nominal_z = .4 + config['camera_detection_task']['camera_points_mm'][2]/1000.
    assert steps[1]["_pose"].position.z == pytest.approx(nominal_z)
    assert steps[0]["_pose"].position.z == pytest.approx(nominal_z-config['detect_controller']['approach_clearance'])
    assert steps[0]["_pose"] == steps[2]["_pose"]
    assert steps[1]["_cartesian"] and steps[2]["_cartesian"]
    assert steps[-1]["_joint_goal"] == photo
    assert steps[1]["_pose"].position.x == pytest.approx(0.)
    assert steps[1]["_pose"].position.y == pytest.approx(0.)
    detect = config["detect_controller"]
    for step in steps:
        assert step["_velocity_scaling"] == detect["velocity_scaling"] == .01
        assert step["_acceleration_scaling"] == detect["acceleration_scaling"] == .01
        assert step["_planning_time_sec"] == 5.0
    constraints = preview.pose_constraints(steps[0])
    assert not constraints.joint_constraints
    assert constraints.orientation_constraints[0].absolute_x_axis_tolerance == pytest.approx(
        min(math.radians(detect["orientation_tolerance_deg"]), detect["pose_tolerance"] / (4 * .1)))
    assert steps[1]["_cartesian_speed"] == .0025
    assert steps[2]["_cartesian_speed"] == .003
    assert steps[1]["_cartesian_step"] == detect["moveit_cartesian_step"]
    assert steps[2]["_cartesian_step"] == .001


def test_preview_rejects_unimplemented_candidate_selection(tmp_path):
    data = yaml.safe_load((ROOT / "config/diagnostics/arrival_detect.yaml").read_text())
    data["detect_controller"] = {"ros__parameters": {"preposition_rp_tolerance_deg": [5., 5.]}}
    workflow = tmp_path / "workflow.yaml"
    workflow.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="Rx/Ry candidate"):
        preview.load_preview(ROOT / "simulation/path_preview.yaml", str(workflow))


def test_probe_endpoint_checks_actual_tip_with_mounting_offset():
    config, _, joints, _, _ = preview.load_preview(ROOT / "simulation/path_preview.yaml")
    mounting = np.eye(4)
    mounting[2, 3] = .146
    target = np.eye(4)
    target[2, 3] = .5
    actual = target.copy()
    actual[:3, :3] = rpy_degrees_rotation([2., 0., 0.])
    result = SimpleNamespace(error_code=MoveItErrorCodes(val=1),
                             pose_stamped=[SimpleNamespace(pose=preview.to_pose(actual))])
    node = SimpleNamespace(config=config, fk=Mock(), call=Mock(return_value=result))
    step = {"_pose": preview.to_pose(target)}
    with pytest.raises(RuntimeError, match="probe endpoint error"):
        preview.PathPreview.verify_probe_endpoint(node, step, joints, mounting)
    result.pose_stamped[0].pose = preview.to_pose(target)
    checked = preview.PathPreview.verify_probe_endpoint(node, step, joints, mounting)
    assert checked["probe_endpoint_error_m"] == pytest.approx(0.)
    assert checked["tool_endpoint_angle_error_deg"] == pytest.approx(0.)


def test_parallel_camera_z_converts_1_2_13_cm_to_1_2_8_cm_precontact():
    config, _, photo, _, _ = preview.load_preview(ROOT / 'simulation/path_preview.yaml')
    config['detect_controller']['approach_clearance'] = .05  # 此例专门验证13→8 cm，与默认配置无关。
    config['camera_detection_task']['camera_points_mm'] = [10., 20., 130.]
    tool = np.eye(4)
    tool[:3, :3] = rpy_degrees_rotation([-175., 0., -90.])
    tool[:3, 3] = [.375, .360, .590]
    tool_camera = np.eye(4)
    tool_camera[:3, :3] = rpy_degrees_rotation([0., 0., -135.])
    tool_camera[:3, 3] = [.03, -.057, .058]
    camera = tool @ tool_camera
    mounting = np.eye(4)
    mounting[:3, 3] = [.025, -.01, .146]
    probe = tool @ mounting
    target = camera @ np.array([.01, .02, .13, 1.])
    expected_pre = camera @ np.array([.01, .02, .08, 1.])
    steps = preview.detection_steps(config, (camera, probe, tool), photo)
    final = preview.pose_matrix(steps[1]['_pose'])
    pre = preview.pose_matrix(steps[0]['_pose'])
    np.testing.assert_allclose(final[:3, 2], tool[:3, 2], atol=1e-12)
    np.testing.assert_allclose((final @ mounting)[:3, 3], target[:3], atol=1e-12)
    np.testing.assert_allclose((pre @ mounting)[:3, 3], expected_pre[:3], atol=1e-12)
    np.testing.assert_allclose(final[:3, 3]-pre[:3, 3], .05*tool[:3, 2], atol=1e-12)
    np.testing.assert_allclose(pre[:3, :3], final[:3, :3], atol=1e-12)


def test_axis_only_goal_uses_frozen_tool_frame_free_yaw_and_probe_offset():
    mounting = np.eye(4)
    mounting[:3, 3] = [.02, -.01, .146]
    pose = np.eye(4)
    pose[:3, :3] = rpy_degrees_rotation([-175., 0., -90.])
    pose[:3, 3] = [.4, .4, .6]
    step = dict(_pose=preview.to_pose(pose), _axis_only=True,
                _probe_mounting=mounting.tolist(), _axis_reference_frame='frozen_photo_tool',
                arrival_tolerance=.0015, arrival_orientation_tolerance_deg=.29)
    constraints = preview.pose_constraints(step)
    assert not constraints.joint_constraints
    pc, oc = constraints.position_constraints[0], constraints.orientation_constraints[0]
    assert pc.header.frame_id == 'base_link'
    np.testing.assert_allclose([pc.target_point_offset.x, pc.target_point_offset.y, pc.target_point_offset.z], mounting[:3, 3])
    center = pc.constraint_region.primitive_poses[0].position
    np.testing.assert_allclose([center.x, center.y, center.z], (pose @ mounting)[:3, 3])
    assert oc.header.frame_id == 'frozen_photo_tool'  # Must NOT free base Z instead.
    assert oc.orientation.w == 1. and oc.orientation.x == oc.orientation.y == oc.orientation.z == 0.
    assert oc.parameterization == preview.OrientationConstraint.ROTATION_VECTOR
    assert oc.absolute_x_axis_tolerance == oc.absolute_y_axis_tolerance == pytest.approx(math.radians(.29))
    assert oc.absolute_z_axis_tolerance == pytest.approx(2.*math.pi)


def test_axis_only_endpoint_accepts_pure_spin_but_rejects_axis_tilt():
    config, _, joints, _, _ = preview.load_preview(ROOT/'simulation/path_preview.yaml')
    mounting = np.eye(4)
    mounting[2, 3] = .146
    target = np.eye(4)
    target[2, 3] = .5
    actual = spin_tool_pose(target, mounting, 90.)
    result = SimpleNamespace(error_code=MoveItErrorCodes(val=1),
                            pose_stamped=[SimpleNamespace(pose=preview.to_pose(actual))])
    node = SimpleNamespace(config=config, fk=Mock(), call=Mock(return_value=result))
    step = dict(_pose=preview.to_pose(target), _axis_only=True)
    checked = preview.PathPreview.verify_probe_endpoint(node, step, joints, mounting)
    assert checked['tool_endpoint_angle_error_deg'] == pytest.approx(0.)
    with pytest.raises(RuntimeError, match='exceeds tolerance'):
        preview.PathPreview.verify_probe_endpoint(node, {'_pose':step['_pose']}, joints, mounting)
    actual[:3, :3] = rpy_degrees_rotation([3., 0., 90.])
    result.pose_stamped[0].pose = preview.to_pose(actual)
    with pytest.raises(RuntimeError, match='exceeds tolerance'):
        preview.PathPreview.verify_probe_endpoint(node, step, joints, mounting)


def test_axis_only_mode_plans_once_without_discrete_angle_loop():
    config, _, photo, _, _ = preview.load_preview(ROOT/'simulation/path_preview.yaml')
    transforms = [np.eye(4) for _ in range(3)]
    transforms[0][2, 3], transforms[1][2, 3], transforms[2][2, 3] = .5, .4, .3
    node = SimpleNamespace(config=config, report={}, save_report=Mock(), get_logger=Mock(),
                           plan_step=Mock(side_effect=RuntimeError('no axis-only goal')))
    with pytest.raises(RuntimeError, match='no axis-only goal'):
        preview.PathPreview.axis_only_detection(node, transforms, photo, 'frozen_photo_tool')
    node.plan_step.assert_called_once()
    step = node.plan_step.call_args.args[0]
    assert step['_axis_only']
    assert step['_axis_reference_frame'] == 'frozen_photo_tool'
    assert node.report['detection_orientation_trials'][0]['mode'] == 'AXIS_ONLY_CONTINUOUS_SPIN'
    assert node.report['detection_orientation_trials'][0]['status'] == 'REJECTED'


@pytest.mark.parametrize('side_error', [0., .005])
def test_axis_only_contact_freezes_selected_spin_and_checks_line_alignment(side_error):
    config, _, photo, _, _ = preview.load_preview(ROOT/'simulation/path_preview.yaml')
    transforms = [np.eye(4) for _ in range(3)]
    transforms[0][2, 3], transforms[1][2, 3], transforms[2][2, 3] = .5, .4, .3
    group = preview.detection_steps(config, transforms, photo)
    mounting = np.linalg.inv(transforms[2]) @ transforms[1]
    actual = spin_tool_pose(preview.pose_matrix(group[0]['_pose']), mounting, 90.)
    actual[0, 3] += side_error
    planned_steps = []
    def plan(step, current):
        planned_steps.append(step)
        return trajectory(current, current), current, {'name':step['name'], 'status':'PLANNED'}
    node = SimpleNamespace(config=config, report={}, save_report=Mock(), get_logger=Mock(),
                           plan_step=plan, tool_fk=Mock(return_value=actual),
                           verify_probe_endpoint=Mock(return_value={}))
    if side_error:
        with pytest.raises(RuntimeError, match='lateral=5.000mm'):
            preview.PathPreview.axis_only_detection(node, transforms, photo, 'frozen_photo_tool')
        assert len(planned_steps) == 1
        return
    result = preview.PathPreview.axis_only_detection(node, transforms, photo, 'frozen_photo_tool')
    assert len(result) == 4
    contact = preview.pose_matrix(planned_steps[1]['_pose'])
    retract = preview.pose_matrix(planned_steps[2]['_pose'])
    np.testing.assert_allclose(contact[:3, :3], actual[:3, :3])
    np.testing.assert_allclose(retract, actual)
    np.testing.assert_allclose(contact[:3, 3]-actual[:3, 3],
                               config['detect_controller']['approach_clearance']*actual[:3, 2], atol=1e-12)
    assert node.report['detection_orientation_trials'][0]['status'] == 'ACCEPTED'


def test_original_ompl_request_is_used_without_planner_switching():
    from moveit_msgs.msg import MotionPlanRequest
    failed = SimpleNamespace(motion_plan_response=SimpleNamespace(error_code=MoveItErrorCodes(val=99999)))
    node = SimpleNamespace(config={"planning_time_sec": 15.0},
                           planner=Mock(), call=Mock(return_value=failed))
    motion = MotionPlanRequest(pipeline_id="ompl", planner_id="RRTConnectkConfigDefault",
        start_state=preview.robot_state([0.0]*6), goal_constraints=[preview.joint_constraints([0.1]*6)])
    segment = {"planner": "RRTConnectkConfigDefault"}
    result = preview.PathPreview.plan_motion(node, motion, segment)
    assert result.error_code.val == 99999 and node.call.call_count == 1
    assert node.call.call_args.args[1].motion_plan_request == motion


def test_preview_only_keeps_axis_only_mode():
    config, _, _, _, _ = preview.load_preview(ROOT/'simulation/path_preview.yaml')
    assert config['detection_orientation_mode'] == 'axis_only'
    assert 'prefer_nearby_ik' not in config
    assert 'detection_tilt_limit_deg' not in config
    for removed in ('position_only', 'photo_orientation', 'cartesian_straight', 'sampled'):
        with pytest.raises(ValueError, match='Only axis_only'):
            preview.load_preview(ROOT/'simulation/path_preview.yaml', orientation_override=removed)


def test_axis_motion_uses_terminal_constraint_without_transport_path_constraint():
    from moveit_msgs.msg import MotionPlanRequest
    config, _, photo, _, _ = preview.load_preview(ROOT/'simulation/path_preview.yaml')
    mounting, pose = np.eye(4), np.eye(4)
    mounting[2, 3] = .146
    step = dict(name='axis_pre', action='arm_move', _pose=preview.to_pose(pose),
                _axis_only=True, _probe_mounting=mounting.tolist(), _axis_reference_frame='frozen_photo')
    failed = SimpleNamespace(error_code=MoveItErrorCodes(val=99999))
    node = SimpleNamespace(config=config, ground=True, plan_motion=Mock(return_value=failed))
    with pytest.raises(RuntimeError, match='MoveIt planning failed'):
        preview.PathPreview.plan_step(node, step, photo)
    motion = node.plan_motion.call_args.args[0]
    assert isinstance(motion, MotionPlanRequest)
    assert not motion.path_constraints.orientation_constraints
    assert not motion.path_constraints.position_constraints
    assert not motion.goal_constraints[0].joint_constraints
    assert motion.goal_constraints[0].orientation_constraints[0].absolute_z_axis_tolerance == pytest.approx(2.*math.pi)


def test_pose_overrides_do_not_rewrite_original_workflow():
    path = ROOT/'config/diagnostics/arrival_detect.yaml'
    before = path.read_bytes()
    _, _, _, _, steps = preview.load_preview(ROOT/'simulation/path_preview.yaml')
    original = yaml.safe_load(before)['workflow']['steps']
    by_name = {step['name']:step for step in original}
    for actual in steps:
        if actual['name'] in ('to_transition_2', 'return_transition_2'):
            continue
        source = by_name[actual['name']]
        if actual['name'] not in ('to_photo_pose', 'to_camera_via', 'to_camera_intermediate',
                                  'return_camera_via', 'return_camera_intermediate'):
            assert actual == source
    assert path.read_bytes() == before


@pytest.mark.parametrize('photo', [{'position':[.2,.3]},
    {'position':[.2,.3,.5], 'orientation':[0.,float('nan'),0.]},
    {'position':[.2,.3], 'orientation':[0.,0.,0.]}])
def test_invalid_photo_override_rejected(tmp_path, photo):
    config = yaml.safe_load((ROOT/'simulation/path_preview.yaml').read_text())
    config.update(workflow_file=str(ROOT/'config/diagnostics/arrival_detect.yaml'),
                  task_config_file=str(ROOT/'config/robot_task.yaml'), photo_pose=photo)
    path = tmp_path/'preview.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='photo_pose'):
        preview.load_preview(path)


def editable_preview(tmp_path, **overrides):
    config = yaml.safe_load((ROOT/'simulation/path_preview.yaml').read_text())
    config.update(workflow_file=str(ROOT/'config/diagnostics/arrival_detect.yaml'),
                  task_config_file=str(ROOT/'config/robot_task.yaml'))
    config.update(overrides)
    path = tmp_path/'preview.yaml'
    path.write_text(yaml.safe_dump(config, sort_keys=False))
    return path


@pytest.mark.parametrize('clearance', [.01, .02, .05, .2])
def test_preview_clearance_changes_precontact_not_nominal_or_real_config(tmp_path, clearance):
    real = ROOT/'config/robot_task.yaml'
    before = real.read_bytes()
    path = editable_preview(tmp_path, approach_clearance=clearance, camera_points_mm=[0., 0., 130.])
    config, _, photo, _, _ = preview.load_preview(path)
    assert config['detect_controller']['approach_clearance'] == clearance
    transforms = [np.eye(4) for _ in range(3)]
    for transform in transforms:
        transform[2, 3] = .5
    steps = preview.detection_steps(config, transforms, photo)
    assert steps[1]['_pose'].position.z == pytest.approx(.63)
    assert steps[0]['_pose'].position.z == pytest.approx(.63-clearance)
    assert steps[2]['_pose'] == steps[0]['_pose']
    document = yaml.safe_load(path.read_text())
    del document['approach_clearance']
    path.write_text(yaml.safe_dump(document, sort_keys=False))
    inherited, *_ = preview.load_preview(path)
    assert inherited['detect_controller']['approach_clearance'] == yaml.safe_load(before)['detect_controller']['ros__parameters']['approach_clearance']
    assert real.read_bytes() == before


@pytest.mark.parametrize('clearance', [None, True, 0., -.01, .201, float('nan'), float('inf'), '0.02'])
def test_invalid_preview_clearance_rejected(tmp_path, clearance):
    with pytest.raises(ValueError, match='approach_clearance'):
        preview.load_preview(editable_preview(tmp_path, approach_clearance=clearance))


def test_camera_point_override_is_preview_only_and_can_be_removed(tmp_path):
    workflow = ROOT/'config/diagnostics/arrival_detect.yaml'
    before = workflow.read_bytes()
    path = editable_preview(tmp_path, camera_points_mm=[1., 2., 80., 3., 4., 90.])
    config, *_ = preview.load_preview(path)
    assert config['camera_detection_task']['camera_points_mm'] == [1., 2., 80., 3., 4., 90.]
    data = yaml.safe_load(path.read_text())
    del data['camera_points_mm']
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    config, *_ = preview.load_preview(path)
    assert config['camera_detection_task']['camera_points_mm'] == yaml.safe_load(before)['camera_detection_task']['ros__parameters']['camera_points_mm']
    assert workflow.read_bytes() == before


@pytest.mark.parametrize('points', [None, 50, [], [0., 50.], [0., True, 50.], [0., 0., -1.], [0., 0., float('nan')]])
def test_invalid_preview_camera_points_rejected(tmp_path, points):
    with pytest.raises(ValueError):
        preview.load_preview(editable_preview(tmp_path, camera_points_mm=points))


def test_waypoints_can_be_added_removed_reordered_and_reverse_automatically(tmp_path):
    pose = dict(position=[.1, .4, .6], orientation=[180., 0., -25.])
    for names in (['to_camera_intermediate', 'to_extra', 'to_camera_via'], ['to_extra'], []):
        overrides = {name: pose for name in names}
        _, _, _, _, steps = preview.load_preview(editable_preview(tmp_path, waypoint_poses=overrides))
        moves = [step for step in steps if step['action'] == 'arm_move']
        assert [step['name'] for step in moves] == names+['to_photo_pose']+[
            'return_'+name[3:] for name in reversed(names)]
        for out, back in zip(moves[:len(names)], reversed(moves[len(names)+1:])):
            assert out['target_pose'] == back['target_pose']
            assert out['frame_id'] == back['frame_id'] == 'base_link'
            assert out['end_effector_frame'] == back['end_effector_frame'] == 'tool0'


@pytest.mark.parametrize('overrides', [[], {'unknown':{}},
    {'to_photo_pose': {'position':[.1,.2,.3], 'orientation':[0.,0.,0.]}},
    {'to_camera_via':{'position':[.1,.2], 'orientation':[0.,0.,0.]}},
    {'to_camera_intermediate':{'position':[.1,.2,.3], 'orientation':[0.,True,0.]}}])
def test_invalid_waypoint_override_rejected(tmp_path, overrides):
    config = yaml.safe_load((ROOT/'simulation/path_preview.yaml').read_text())
    config.update(workflow_file=str(ROOT/'config/diagnostics/arrival_detect.yaml'),
                  task_config_file=str(ROOT/'config/robot_task.yaml'), waypoint_poses=overrides)
    path = tmp_path/'preview.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='waypoint_poses'):
        preview.load_preview(path)


def test_transition2_is_joint_space_and_preserves_unwrapped_requested_degrees():
    _, _, _, postures, steps = preview.load_preview(ROOT/'simulation/path_preview.yaml')
    names = [step['name'] for step in steps]
    assert names.index('to_transition_2') == names.index('to_transition')+1
    assert names.index('return_transition_2')+1 == names.index('return_transition')
    np.testing.assert_allclose(np.rad2deg(postures['detection_ready']), [-45.,0.,0.,0.,90.,135.])
    np.testing.assert_allclose(np.rad2deg(postures['detection_ready_2']), [-45.,-90.,180.,0.,0.,135.])
    for name in ('to_transition_2', 'return_transition_2'):
        step = next(step for step in steps if step['name']==name)
        assert step['action'] == 'arm_posture' and step['posture'] == 'detection_ready_2'
        assert 'target_pose' not in step


@pytest.mark.parametrize('angles', [[-45.,-90.,180.], [0.,0.,True,0.,0.,0.],
                                  [0.,0.,float('inf'),0.,0.,0.]])
def test_invalid_transition2_angles_rejected(tmp_path, angles):
    config = yaml.safe_load((ROOT/'simulation/path_preview.yaml').read_text())
    config.update(workflow_file=str(ROOT/'config/diagnostics/arrival_detect.yaml'),
                  task_config_file=str(ROOT/'config/robot_task.yaml'), transition2_joint_degrees=angles)
    path = tmp_path/'preview.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='transition2_joint_degrees'):
        preview.load_preview(path)


def test_joint_posture_rejects_moveit_success_with_clamped_elbow():
    config, _, initial, postures, _ = preview.load_preview(ROOT/'simulation/path_preview.yaml')
    actual = postures['detection_ready_2'].copy()
    actual[2] = math.radians(171.405633)
    response = SimpleNamespace(error_code=MoveItErrorCodes(val=1),
        trajectory=trajectory(initial, actual), planning_time=.01)
    node = SimpleNamespace(config=config, postures=postures, ground=True,
        validate=Mock(), plan_motion=Mock(return_value=response))
    segment = dict(name='to_transition_2')
    with pytest.raises(RuntimeError, match='elbow_joint requested=180deg, planned=171.406deg'):
        preview.PathPreview.plan_step(node,
            dict(name='to_transition_2', action='arm_posture', posture='detection_ready_2'), initial, segment)
    assert segment['status'] == 'FAILED'
    assert segment['requested_joint_degrees'][2] == pytest.approx(180.)
    assert segment['end_joint_degrees'][2] == pytest.approx(171.405633)
