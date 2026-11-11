"""Offline geometry and mocked service tests; never create ROS nodes or move hardware."""
import importlib.util
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
import yaml

from geminipro_camer_calibration.geometry import pose_matrix, rpy_degrees_rotation

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'preposition_orientation_test', ROOT/'detect_control/scripts/detect_controller.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
CONTROLLER = MODULE.DetectController


def poses():
    final, mounting = np.eye(4), np.eye(4)
    final[:3, :3] = rpy_degrees_rotation([-175., 0., -90.])
    final[:3, 3] = [.375, .36, .59]
    mounting[:3, :3] = rpy_degrees_rotation([15., 5., -20.])
    mounting[:3, 3] = [.025, -.01, .146]
    return final, mounting


def test_candidates_preserve_probe_xyz_and_base_tool_yaw():
    final, mounting = poses()
    candidates = list(MODULE.preposition_orientations(final, mounting, [45., 45.], 15.))
    assert len(candidates) == 49
    assert candidates[0][:2] == (0., 0.)
    np.testing.assert_allclose(candidates[0][2], final, atol=1e-12)
    radii = []
    for rx, ry, candidate in candidates:
        assert abs(rx) <= 45. and abs(ry) <= 45.
        np.testing.assert_allclose((candidate @ mounting)[:3, 3],
                                   (final @ mounting)[:3, 3], atol=1e-12)
        expected = rpy_degrees_rotation([-175. + rx, ry, -90.])
        np.testing.assert_allclose(candidate[:3, :3], expected, atol=1e-12)
        yaw = Rotation.from_matrix(candidate[:3, :3]).as_euler('xyz', degrees=True)[2]
        assert yaw == pytest.approx(-90.)
        radii.append(rx*rx + ry*ry)
    assert radii == sorted(radii)


def test_euler_singularity_does_not_silently_change_yaw():
    final, mounting = poses()
    final[:3, :3] = rpy_degrees_rotation([20., 90., 35.])
    with pytest.raises(ValueError, match='singularity'):
        list(MODULE.preposition_orientations(final, mounting, [45., 45.], 15.))
    final[:3, :3] = rpy_degrees_rotation([20., 80., 35.])
    for rx, ry, candidate in MODULE.preposition_orientations(final, mounting, [45., 45.], 15.):
        assert 80. + ry < 90.
        yaw = Rotation.from_matrix(candidate[:3, :3]).as_euler('xyz', degrees=True)[2]
        assert yaw == pytest.approx(35.)


def node():
    result = MagicMock()
    result.final, result.mounting = poses()
    result.pre = result.final.copy()
    result.cfg = dict(preposition_rp_tolerance_deg=[45., 45.], preposition_angle_step_deg=15.,
                      approach_axis='tool_positive_z', approach_clearance=.02, max_penetration=.05,
                      planning_group='cs_manipulator', tool_frame='tool0', base_frame='base_link',
                      orientation_tolerance=np.deg2rad(1.), pose_tolerance=.003)
    result.point_index = 0
    result.batch = [(result.pre.copy(), result.final.copy())]
    result.joints = JointState(name=MODULE.JOINTS, position=[0.]*6)
    result.ik.service_is_ready.return_value = True
    result.in_workspace.return_value = True
    result.direction_for.side_effect = lambda pose: CONTROLLER.direction_for(result, pose)
    return result


def response(code=MoveItErrorCodes.SUCCESS):
    result = GetPositionIK.Response()
    result.error_code.val = code
    result.solution.joint_state = JointState(name=MODULE.JOINTS, position=[.1]*6)
    return result


def complete(subject, result):
    subject.wait.call_args.args[1](result)


def test_selection_checks_both_poses_then_recomputes_twenty_mm_approach():
    subject, done = node(), MagicMock()
    original_tip = (subject.final @ subject.mounting)[:3, 3].copy()
    CONTROLLER.select_preposition(subject, done)
    complete(subject, response(MoveItErrorCodes.NO_IK_SOLUTION))
    request = subject.ik.call_async.call_args.args[0]
    assert request.ik_request.avoid_collisions
    selected_pre = pose_matrix(request.ik_request.pose_stamped.pose)
    selected = response()
    complete(subject, selected)
    done.assert_not_called()
    final_request = subject.ik.call_async.call_args.args[0]
    assert final_request.ik_request.avoid_collisions
    assert list(final_request.ik_request.robot_state.joint_state.position) == [.1]*6
    complete(subject, response())
    done.assert_called_once_with(selected.solution.joint_state)
    np.testing.assert_allclose(subject.pre, selected_pre, atol=1e-12)
    np.testing.assert_allclose((subject.final @ subject.mounting)[:3, 3], original_tip, atol=1e-12)
    direction = CONTROLLER.direction_for(subject, subject.final)
    np.testing.assert_allclose(subject.final[:3, 3]-subject.pre[:3, 3], direction*.02, atol=1e-12)
    np.testing.assert_allclose((subject.pre @ subject.mounting)[:3, 3], original_tip-direction*.02)
    subject.action.assert_not_called()
    assert subject.batch[0][0] is subject.pre


def test_rejected_contact_pose_tries_another_orientation_without_moving():
    subject, done = node(), MagicMock()
    original = subject.final.copy()
    CONTROLLER.select_preposition(subject, done)
    complete(subject, response())
    complete(subject, response(MoveItErrorCodes.NO_IK_SOLUTION))
    assert subject.ik.call_async.call_count == 3
    done.assert_not_called()
    np.testing.assert_array_equal(subject.final, original)
    subject.action.assert_not_called()


def test_all_failed_candidates_stop_without_execution():
    subject, done = node(), MagicMock()
    CONTROLLER.select_preposition(subject, done)
    for _ in range(48):
        complete(subject, response(MoveItErrorCodes.NO_IK_SOLUTION))
    with pytest.raises(MODULE.RetryableApproachError, match='连续角度'):
        complete(subject, response(MoveItErrorCodes.NO_IK_SOLUTION))
    assert subject.ik.call_async.call_count == 49
    done.assert_not_called()
    subject.action.assert_not_called()


def test_unknown_ik_failure_and_malformed_solution_are_not_treated_as_candidates():
    for answer in (response(MoveItErrorCodes.FRAME_TRANSFORM_FAILURE), response()):
        subject = node()
        if answer.error_code.val == MoveItErrorCodes.SUCCESS:
            answer.solution.joint_state.position = []
        CONTROLLER.select_preposition(subject, MagicMock())
        with pytest.raises(RuntimeError):
            complete(subject, answer)
        assert subject.ik.call_async.call_count == 1
        subject.action.assert_not_called()


def test_selected_pose_keeps_strict_orientation_and_ompl_collision_planning():
    subject = node()
    subject.motion_request.side_effect = lambda: MODULE.MoveGroup.Goal()
    joints = response().solution.joint_state
    CONTROLLER.plan_pose(subject, subject.pre, selected_joints=joints)
    constraints = subject.action.call_args.args[1].request.goal_constraints[0]
    assert len(constraints.joint_constraints) == 6
    oc = constraints.orientation_constraints[0]
    assert oc.absolute_x_axis_tolerance == oc.absolute_y_axis_tolerance
    assert oc.absolute_z_axis_tolerance <= np.deg2rad(1.)
    assert oc.absolute_z_axis_tolerance < .01
    subject.select_preposition.assert_not_called()


def test_disabled_selection_preserves_original_pose_planning():
    subject = node()
    subject.cfg['preposition_rp_tolerance_deg'] = [0., 0.]
    subject.motion_request.side_effect = lambda: MODULE.MoveGroup.Goal()
    CONTROLLER.plan_pose(subject, subject.pre)
    subject.select_preposition.assert_not_called()
    assert not subject.action.call_args.args[1].request.goal_constraints[0].joint_constraints


def test_contact_approach_requests_two_mm_per_second_with_collisions_enabled():
    subject = node()
    subject.cfg.update(continue_until_contact=True, moveit_cartesian_step=.005,
                       approach_speed=.002, velocity_scaling=.1, acceleration_scaling=.1,
                       phase_timeout=120., contact_force=25.)
    subject.contact_stop_requested = False
    subject.current_tool.return_value = subject.pre
    subject.axial_force.return_value = 0.
    subject.line_start = subject.pre
    subject.axis = CONTROLLER.direction_for(subject, subject.pre)
    subject.travel_limit = .07
    CONTROLLER.plan_moveit_descent(subject)
    request = subject.cartesian.call_async.call_args.args[0]
    assert request.max_cartesian_speed == .002
    assert request.avoid_collisions
    assert request.cartesian_speed_limited_link == 'tool0'


def test_production_yaml_uses_requested_limits_and_slowdown():
    document = yaml.safe_load((ROOT/'robot_task_control/config/robot_task.yaml').read_text())
    config = document['detect_controller']['ros__parameters']
    assert config['preposition_rp_tolerance_deg'] == [0., 0.]
    assert config['preposition_angle_step_deg'] == 15.
    assert config['approach_clearance'] == .02
    assert config['approach_speed'] == .001
    assert config['velocity_scaling'] == config['acceleration_scaling'] == .05
    assert document['camera_detection_task']['ros__parameters']['camera_points_mm'] == [-10., -10., 130.]
    assert document['arm_joint_postures']['ros__parameters']['velocity_scaling'] == .1
    assert not config['lock_tool_orientation']
    assert (config['initial_force_limit'], config['contact_force'], config['overload_force']) == (15., 25., 40.)
