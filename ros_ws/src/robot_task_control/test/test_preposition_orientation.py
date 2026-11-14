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


@pytest.mark.parametrize('rpy', [[-175., 0., -90.], [20., 90., 35.], [25., -30., 60.]])
def test_local_z_candidates_preserve_photo_axis_and_probe_xyz(rpy):
    final, mounting = poses()
    final[:3, :3] = rpy_degrees_rotation(rpy)
    candidates = list(MODULE.tool_z_orientations(final, mounting, 15.))
    assert len(candidates) == 24
    assert [item[0] for item in candidates[:5]] == [0., 15., -15., 30., -30.]
    np.testing.assert_allclose(candidates[0][2], final, atol=1e-12)
    for angle, _, candidate in candidates:
        np.testing.assert_allclose(candidate[:3, 2], final[:3, 2], atol=1e-12)
        np.testing.assert_allclose((candidate @ mounting)[:3, 3],
                                   (final @ mounting)[:3, 3], atol=1e-12)
        np.testing.assert_allclose(final[:3, :3].T @ candidate[:3, :3],
                                   rpy_degrees_rotation([0., 0., angle]), atol=1e-12)
    assert not np.allclose(candidates[1][2][:3, 3], final[:3, 3])
    assert len({tuple(np.round(item[2][:3, :3].ravel(), 8)) for item in candidates}) == 24


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


def test_local_z_selection_rejects_original_and_preserves_photo_axis():
    subject, done = node(), MagicMock()
    subject.cfg.update(preposition_tool_z_spin=True, preposition_rp_tolerance_deg=[0., 0.], approach_clearance=.05)
    original = subject.final.copy()
    tip = (original @ subject.mounting)[:3, 3]
    CONTROLLER.select_preposition(subject, done)
    complete(subject, response(MoveItErrorCodes.GOAL_IN_COLLISION))
    selected_pre = pose_matrix(subject.ik.call_async.call_args.args[0].ik_request.pose_stamped.pose)
    complete(subject, response())
    done.assert_not_called()
    complete(subject, response())
    done.assert_called_once()
    np.testing.assert_allclose(subject.final[:3, 2], original[:3, 2], atol=1e-12)
    np.testing.assert_allclose((subject.final @ subject.mounting)[:3, 3], tip, atol=1e-12)
    np.testing.assert_allclose((subject.pre @ subject.mounting)[:3, 3], tip-original[:3, 2]*.05)
    np.testing.assert_allclose(subject.pre, selected_pre, atol=1e-12)
    for call in subject.ik.call_async.call_args_list:
        assert call.args[0].ik_request.avoid_collisions
    subject.action.assert_not_called()


def test_local_z_selection_exhaustion_never_moves():
    subject, done = node(), MagicMock()
    subject.cfg.update(preposition_tool_z_spin=True, preposition_rp_tolerance_deg=[0., 0.])
    CONTROLLER.select_preposition(subject, done)
    for _ in range(23):
        complete(subject, response(MoveItErrorCodes.NO_IK_SOLUTION))
    with pytest.raises(MODULE.RetryableApproachError):
        complete(subject, response(MoveItErrorCodes.NO_IK_SOLUTION))
    assert subject.ik.call_async.call_count == 24
    done.assert_not_called()
    subject.action.assert_not_called()


def test_local_z_mode_uses_selection_but_not_for_return_motion():
    subject = node()
    subject.cfg.update(preposition_tool_z_spin=True, preposition_rp_tolerance_deg=[0., 0.])
    CONTROLLER.plan_pose(subject, subject.pre)
    subject.select_preposition.assert_called_once()
    subject.action.assert_not_called()
    subject.select_preposition.reset_mock()
    subject.motion_request.side_effect = lambda: MODULE.MoveGroup.Goal()
    CONTROLLER.plan_pose(subject, subject.final, phase='RETURN_HOME')
    subject.select_preposition.assert_not_called()
    subject.action.assert_called_once()


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


@pytest.mark.parametrize('speed', [.0005, .002, .01])
def test_retract_never_exceeds_approach_speed_or_old_limit(speed):
    subject = node()
    subject.cfg.update(approach_speed=speed, velocity_scaling=.01,
                       acceleration_scaling=.01, phase_timeout=300.)
    subject.current_tool.return_value = subject.pre
    subject.axis = CONTROLLER.direction_for(subject, subject.pre)
    CONTROLLER.retreat(subject)
    request = subject.cartesian.call_async.call_args.args[0]
    assert request.max_cartesian_speed == min(.005, speed)
    assert request.avoid_collisions
    assert request.max_velocity_scaling_factor == .01
    assert request.max_acceleration_scaling_factor == .01


def test_retract_uses_independent_speed_limit():
    subject = node()
    subject.cfg.update(approach_speed=.0025, contact_approach_speed=.001,
                       retract_speed=.003, velocity_scaling=.01,
                       acceleration_scaling=.01, phase_timeout=300.)
    subject.current_tool.return_value = subject.pre
    subject.axis = CONTROLLER.direction_for(subject, subject.pre)
    CONTROLLER.retreat(subject)
    request = subject.cartesian.call_async.call_args.args[0]
    assert request.max_cartesian_speed == .003
    assert request.avoid_collisions


@pytest.mark.parametrize('phase', ['SELECT_PREPOSITION', 'PREPOSITION', 'VERIFY_PREPOSITION'])
@pytest.mark.parametrize('force', [[30., 0., 0.], [18., 24., 0.]])
def test_preposition_force_at_threshold_cancels_and_faults_without_auto_continue(phase, force):
    subject = node()
    subject.cfg.update(initial_force_limit=15., preposition_force_limit=30., overload_force=40., contact_force=25.)
    subject.phase = phase
    subject.active = True
    subject.touch_control_mode = 'moveit_force_guard'
    subject.generation = 2
    subject.outstanding_actions = {object()}
    subject.fail.side_effect = lambda detail: CONTROLLER.fail(subject, detail)
    handle = subject.goal_handle
    CONTROLLER.handle_force_sample(subject, np.array(force))
    handle.cancel_goal_async.assert_called_once()
    assert subject.operation is None
    assert subject.generation == 3
    assert subject.stop_confirmed is False
    assert subject.state.call_args.args[0] == 'FAULT'
    subject.begin_hold.assert_not_called()
    subject.request_moveit_force_stop.assert_not_called()
    subject.return_home.assert_not_called()


def test_preposition_raw_force_guard_does_not_wait_for_low_pass():
    subject = node()
    subject.cfg.update(initial_force_limit=15., preposition_force_limit=30., overload_force=40., force_filter_alpha=.25,
                       data_timeout=.3)
    subject.phase = 'PREPOSITION'
    subject.active = True
    subject.touch_control_mode = 'moveit_force_guard'
    subject.force_bias = np.array([1., 2., 3.])
    subject.force_filter = np.zeros(3)
    subject.stamp_fresh.return_value = True
    subject.raw_force_in_base.return_value = np.array([31., 2., 3.])
    CONTROLLER.on_raw(subject, MODULE.WrenchStamped())
    assert '30N' in subject.fail.call_args.args[0]
    subject.handle_force_sample.assert_not_called()
    np.testing.assert_array_equal(subject.force_filter, np.zeros(3))


@pytest.mark.parametrize('phase, force', [('PREPOSITION', [29.9, 0., 0.]),
                                         ('MOVEIT_DESCEND', [15., 0., 0.])])
def test_preposition_guard_does_not_change_below_threshold_or_contact_stage(phase, force):
    subject = node()
    subject.cfg.update(initial_force_limit=15., preposition_force_limit=30., overload_force=40., contact_force=25.)
    subject.phase = phase
    subject.active = True
    subject.axial_force.return_value = 0.
    CONTROLLER.handle_force_sample(subject, np.array(force))
    subject.fail.assert_not_called()


@pytest.mark.parametrize('phase', ['MOVEIT_WAIT_CONTACT', 'MOVEIT_PLAN_DESCENT',
                                  'MOVEIT_DESCEND', 'VERIFY_MOVEIT_SEGMENT'])
@pytest.mark.parametrize('force, stop, fault', [
    ([0., 0., 19.9], False, False), ([0., 0., 20.], True, False),
    ([0., 0., -20.], True, False), ([21., 0., 0.], False, False),
    ([15., 20., 0.], False, True)])
def test_raw_contact_stop_at_twenty_does_not_wait_for_filter(phase, force, stop, fault):
    subject = node()
    subject.cfg.update(initial_force_limit=15., preposition_force_limit=25., overload_force=25.,
                       contact_force=20., force_filter_alpha=.25, data_timeout=.3, base_frame='base_link')
    subject.phase = phase
    subject.active = True
    subject.touch_control_mode = 'moveit_force_guard'
    subject.force_bias = np.array([1., 2., 3.])
    subject.force_filter = np.zeros(3)
    subject.stamp_fresh.return_value = True
    subject.raw_force_in_base.return_value = np.array(force)+subject.force_bias
    subject.axial_force.side_effect = lambda values: abs(values[2])
    CONTROLLER.on_raw(subject, MODULE.WrenchStamped())
    assert subject.request_moveit_force_stop.called is stop
    assert subject.fail.called is fault
    if stop:
        assert abs(subject.force_filter[2]) < 20.  # 滤波仍远低于20，停止已请求。


def test_production_yaml_uses_requested_limits_and_slowdown():
    document = yaml.safe_load((ROOT/'robot_task_control/config/robot_task.yaml').read_text())
    config = document['detect_controller']['ros__parameters']
    assert config['preposition_rp_tolerance_deg'] == [0., 0.]
    assert config['preposition_tool_z_spin'] is True
    assert config['preposition_angle_step_deg'] == 15.
    assert config['approach_clearance'] == .03
    assert config['approach_speed'] == .0025
    assert config['velocity_scaling'] == config['acceleration_scaling'] == .01
    assert document['camera_detection_task']['ros__parameters']['camera_points_mm'] == [0., 0., 100.]
    assert document['arm_joint_postures']['ros__parameters']['velocity_scaling'] == .1
    assert not config['lock_tool_orientation']
    assert config['preposition_force_limit'] == 25.
    assert (config['initial_force_limit'], config['contact_force'], config['overload_force']) == (15., 20., 25.)
