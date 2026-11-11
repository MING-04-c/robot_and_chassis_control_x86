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
        "to_transition", "to_camera_via", "to_camera_intermediate", "to_photo_pose", "return_transition", "return_travel"]
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
    assert displays[-1]["Loop Animation"] is True
    assert displays[-1]["State Display Time"] == "0.05 s"
    assert displays[1]["Scene Robot"]["Show Robot Visual"] is False
    assert displays[-1]["Trajectory Topic"] == "/display_planned_path"


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
                           last_subscribers=-1, last_path_publish=None, display=Mock())
    node.display.get_subscription_count.return_value = 1
    with patch.object(preview, 'monotonic', return_value=10.0):
        for _ in range(10):
            preview.PathPreview.publish_paths(node)
    assert node.display.publish.call_count == 1
    node.display.get_subscription_count.return_value = 0
    preview.PathPreview.publish_paths(node)
    node.display.get_subscription_count.return_value = 1
    preview.PathPreview.publish_paths(node)
    assert node.display.publish.call_count == 2


def test_animation_retries_for_rviz_that_missed_first_message():
    node = SimpleNamespace(preview_ready=True, animation=SimpleNamespace(trajectory=[Mock()]),
                           last_subscribers=-1, last_path_publish=None, display=Mock())
    node.display.get_subscription_count.return_value = 1
    with patch.object(preview, 'monotonic', side_effect=[10.0, 10.5, 12.9, 13.0]):
        for _ in range(4):
            preview.PathPreview.publish_paths(node)
    assert node.display.publish.call_count == 2
    assert node.last_path_publish == 13.0


def test_finish_display_preserves_original_segment_messages_and_initial_state():
    start, middle, end = [0.0]*6, [0.1]*6, [0.2]*6
    node = SimpleNamespace(initial=start, preview_ready=False,
        animation=SimpleNamespace(trajectory=[trajectory(start, middle), trajectory(middle, end)]),
        publish_paths=Mock())
    preview.PathPreview.finish_display(node)
    assert len(node.animation.trajectory) == 2
    assert list(node.animation.trajectory[0].joint_trajectory.points[-1].positions) == list(reversed(middle))
    assert list(node.animation.trajectory[1].joint_trajectory.points[-1].positions) == list(reversed(end))
    assert node.initial == start and node.preview_ready
    node.publish_paths.assert_called_once()


def test_finish_display_does_not_publish_when_first_segment_failed():
    node = SimpleNamespace(preview_ready=False, animation=SimpleNamespace(trajectory=[]), publish_paths=Mock())
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
    assert steps[1]["_pose"].position.x == pytest.approx(-0.01)
    assert steps[1]["_pose"].position.y == pytest.approx(-0.01)


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
