"""Simulation configuration/continuity tests; no hardware or ROS graph required."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
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


def test_default_points_are_shared_with_arrival_workflow():
    config, path, initial, postures, steps = preview.load_preview(ROOT / "simulation/path_preview.yaml")
    assert Path(path) == ROOT / "config/diagnostics/arrival_detect.yaml"
    assert initial == postures["travel"]
    assert math.degrees(initial[0]) == -45.0
    assert [s["name"] for s in steps if s["action"] in ("arm_posture", "arm_move")] == [
        "to_transition", "to_camera_via", "to_camera_intermediate", "to_photo_pose",
        "return_camera_intermediate", "return_camera_via", "return_transition", "return_travel"]
    assert steps[4]["action"] == "camera_detection"
    assert steps[1]["target_pose"]["position"] == [0.150, 0.530, 0.810]
    assert steps[1]["target_pose"]["orientation"] == [75.0, 20.0, 170.0]
    assert steps[2]["target_pose"]["position"] == [0.300, 0.410, 0.795]
    assert steps[2]["target_pose"]["orientation"] == [-90.0, 45.0, -45.0]
    assert config["velocity_scaling"] == 0.1
    assert config["planning_pipeline"] == "ompl"
    assert config["planner_id"] == "RRTConnectkConfigDefault"


def test_pose_uses_yaml_units_and_named_tool():
    _, _, _, _, steps = preview.load_preview(ROOT / "simulation/path_preview.yaml")
    constraints = preview.pose_constraints(steps[3])
    position, orientation = constraints.position_constraints[0], constraints.orientation_constraints[0]
    assert position.link_name == "tool0" and position.header.frame_id == "base_link"
    assert position.constraint_region.primitive_poses[0].position.x == 0.375
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
        with pytest.raises(RuntimeError, match="to_camera_via"):
            preview.PathPreview.run(node)
    assert node.call.call_count == 2
    assert node.animation.trajectory == [first]
    assert [s["status"] for s in node.report["segments"]] == ["PLANNED", "FAILED"]
    second_request = node.call.call_args_list[1].args[1]
    assert list(second_request.motion_plan_request.start_state.joint_state.position) == postures["detection_ready"]
    assert second_request.motion_plan_request.pipeline_id == "ompl"
    assert second_request.motion_plan_request.planner_id == "RRTConnectkConfigDefault"
    goal = second_request.motion_plan_request.goal_constraints[0]
    assert not goal.joint_constraints
    assert goal.position_constraints and goal.orientation_constraints
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
    assert steps[1]["_pose"].position.z == pytest.approx(0.53)
    assert steps[0]["_pose"].position.z == pytest.approx(0.51)
    assert steps[0]["_pose"] == steps[2]["_pose"]
    assert steps[1]["_cartesian"] and steps[2]["_cartesian"]
    assert steps[-1]["_joint_goal"] == photo
    assert steps[1]["_pose"].position.x == pytest.approx(0.)
    assert steps[1]["_pose"].position.y == pytest.approx(0.)
    detect = config["detect_controller"]
    for step in steps:
        assert step["_velocity_scaling"] == detect["velocity_scaling"] == .05
        assert step["_acceleration_scaling"] == detect["acceleration_scaling"] == .05
        assert step["_planning_time_sec"] == 5.0
    constraints = preview.pose_constraints(steps[0])
    assert not constraints.joint_constraints
    assert constraints.orientation_constraints[0].absolute_x_axis_tolerance == pytest.approx(
        min(math.radians(detect["orientation_tolerance_deg"]), detect["pose_tolerance"] / (4 * .1)))
    assert steps[1]["_cartesian_speed"] == .001
    assert steps[1]["_cartesian_step"] == detect["moveit_cartesian_step"]
    assert steps[2]["_cartesian_step"] == .001


def test_preview_rejects_unimplemented_candidate_selection(tmp_path):
    data = yaml.safe_load((ROOT / "config/diagnostics/arrival_detect.yaml").read_text())
    data["detect_controller"] = {"ros__parameters": {"preposition_tool_z_spin": True}}
    workflow = tmp_path / "workflow.yaml"
    workflow.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="direct pose detection"):
        preview.load_preview(ROOT / "simulation/path_preview.yaml", str(workflow))


def test_tilt_keeps_probe_contact_xyz_and_twenty_mm_approach():
    config, _, photo, _, _ = preview.load_preview(ROOT / "simulation/path_preview.yaml")
    tool = np.eye(4)
    tool[:3, 3] = [.3, .3, .6]
    mounting = np.eye(4)
    mounting[:3, 3] = [.03, -.01, .146]
    probe = tool @ mounting
    camera = tool.copy()
    original = preview.detection_steps(config, (camera, probe, tool), photo)
    tilted = preview.detection_steps(config, (camera, probe, tool), photo, (5., -5.))
    nominal = preview.pose_matrix(original[1]["_pose"])
    contact = preview.pose_matrix(tilted[1]["_pose"])
    pre = preview.pose_matrix(tilted[0]["_pose"])
    np.testing.assert_allclose((contact @ mounting)[:3, 3], (nominal @ mounting)[:3, 3])
    np.testing.assert_allclose(contact[:3, 3] - pre[:3, 3], .02 * contact[:3, 2])
    np.testing.assert_allclose(contact[:3, :3], pre[:3, :3])
    assert tilted[0]["_pose"] == tilted[2]["_pose"]
    assert not np.allclose(contact[:3, :3], nominal[:3, :3])
    assert tilted[3]["_joint_goal"] == photo


def test_search_rejects_partial_path_before_accepting_another_tilt():
    config, _, photo, _, _ = preview.load_preview(ROOT / "simulation/path_preview.yaml")
    transforms = [np.eye(4) for _ in range(3)]
    transforms[0][2, 3] = .5
    transforms[1][2, 3] = .4
    transforms[2][2, 3] = .3
    calls = []
    def plan(step, current):
        calls.append(step["name"])
        if len(calls) == 2:
            raise RuntimeError("Cartesian contact incomplete")
        return trajectory(current, current), current, {"name": step["name"], "status": "PLANNED"}
    node = SimpleNamespace(config=config, report={}, ground=True, save_report=Mock(), get_logger=Mock(),
                           plan_step=plan, verify_probe_endpoint=Mock(return_value={}))
    selected = preview.PathPreview.search_detection(node, transforms, photo)
    trials = node.report["detection_orientation_trials"]
    assert [t["status"] for t in trials] == ["REJECTED", "ACCEPTED"]
    assert trials[0]["checking_segment"] == "detect_1_nominal_contact"
    assert len(selected) == 4 and all("_preplanned" in step for step in selected)
    assert calls[-1] == "detect_1_return_photo"
    assert list(preview.detection_tilts(0.)) == [(0., 0.)]
    assert all(math.hypot(*t) <= 15. + 1e-9 for t in preview.detection_tilts(15.))


def test_search_exhaustion_never_accepts_partial_candidates():
    config, _, photo, _, _ = preview.load_preview(ROOT / "simulation/path_preview.yaml")
    config["detection_tilt_limit_deg"] = 5.
    transforms = [np.eye(4) for _ in range(3)]
    transforms[0][2, 3], transforms[1][2, 3], transforms[2][2, 3] = .5, .4, .3
    node = SimpleNamespace(config=config, report={}, save_report=Mock(), get_logger=Mock(),
                           plan_step=Mock(side_effect=RuntimeError("unreachable")))
    with pytest.raises(RuntimeError, match="no complete collision-checked path"):
        preview.PathPreview.search_detection(node, transforms, photo)
    assert len(node.report["detection_orientation_trials"]) == 9
    assert all(t["status"] == "REJECTED" for t in node.report["detection_orientation_trials"])


def test_probe_endpoint_checks_actual_tip_with_mounting_offset():
    config, _, joints, _, _ = preview.load_preview(ROOT / "simulation/path_preview.yaml")
    mounting = np.eye(4)
    mounting[2, 3] = .146
    target = np.eye(4)
    target[2, 3] = .5
    actual = target.copy()
    actual[:3, :3] = preview.rpy_degrees_rotation([2., 0., 0.])
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
