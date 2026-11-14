"""Real recipe and mocked pre-execution checks; no ROS nodes, drivers or motion."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import yaml
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from eli_cs_robot_moveit_config.joint_postures import JOINTS, load_postures
from robot_task_control.joint_posture_control import JointPostureControl
from robot_task_control.oline_config import load_oline_config
from robot_task_control.task_config import load_station_config, load_task_config
from robot_task_control.workflow_controller import WorkflowController

ROOT = Path(__file__).resolve().parents[1]
ANGLES = [-45., -90., 180., 0., 0., 90.]


@pytest.mark.parametrize('mode', ['main', 'arrival', 'oline'])
def test_real_cartesian_targets_match_preview_in_both_directions(mode):
    preview = yaml.safe_load((ROOT/'simulation/path_preview.yaml').read_text())
    if mode == 'oline':
        data = load_oline_config(ROOT/'config/robot_task_oline.yaml')
    else:
        path = ROOT/('config/robot_task.yaml' if mode == 'main'
                     else 'config/diagnostics/arrival_detect.yaml')
        data = yaml.safe_load(path.read_text())
    moves = [step for step in data['workflow']['steps'] if step['action'] == 'arm_move']
    expected = preview['waypoint_poses']
    poses = [expected['to_camera_via'], expected['to_camera_intermediate'],
             preview['photo_pose'], expected['to_camera_intermediate'], expected['to_camera_via']]
    assert len(moves) == len(poses)
    for step, pose in zip(moves, poses):
        assert step['frame_id'] == 'base_link'
        assert step['end_effector_frame'] == 'tool0'
        assert step['target_pose']['position_mode'] == 'absolute'
        assert step['target_pose']['position'] == pose['position']
        assert step['target_pose']['orientation'] == pose['orientation']


@pytest.mark.parametrize('name', ['config/robot_task.yaml', 'config/diagnostics/arrival_detect.yaml'])
def test_real_transition2_has_forward_and_reverse_order(name):
    path = ROOT/name
    (_, data) = (load_task_config(path) if name.endswith('robot_task.yaml') else load_station_config(path))
    np.testing.assert_allclose(np.rad2deg(load_postures(path)['detection_ready_2']), ANGLES)
    assert data['detection_ready_2'] == ANGLES
    steps = WorkflowController.load_workflow(path)['steps']
    names = [step.get('name') for step in steps]
    assert names.index('to_transition_2') == names.index('to_transition')+1
    assert names.index('return_transition_2')+1 == names.index('return_transition')
    assert names.index('return_camera_intermediate') < names.index('return_camera_via') < names.index('return_transition_2')


def test_oline_inherits_transition2_without_changing_remote_session_gate():
    data = load_oline_config(ROOT/'config/robot_task_oline.yaml')
    assert data['detection_ready_2'] == ANGLES
    assert data['workflow']['remote_detection_cycles'] is True
    names = [step.get('name') for step in data['workflow']['steps']]
    assert names.index('return_camera_via') < names.index('return_transition_2') < names.index('return_transition')


def test_synced_camera_and_clearance_keep_real_slowdown_and_force_guards():
    preview = yaml.safe_load((ROOT/'simulation/path_preview.yaml').read_text())
    main = yaml.safe_load((ROOT/'config/robot_task.yaml').read_text())
    for name in ('config/robot_task.yaml', 'config/diagnostics/arrival_detect.yaml',
                 'config/diagnostics/capture_detect.yaml'):
        data = yaml.safe_load((ROOT/name).read_text())
        assert data['camera_detection_task']['ros__parameters']['camera_points_mm'] == preview['camera_points_mm']
    detect = main['detect_controller']['ros__parameters']
    assert detect['approach_clearance'] == preview['approach_clearance'] == .03
    assert detect['approach_speed'] == .001
    assert detect['velocity_scaling'] == detect['acceleration_scaling'] == .01
    assert (detect['initial_force_limit'], detect['preposition_force_limit'],
            detect['contact_force'], detect['overload_force']) == (15., 25., 20., 25.)
    remote = load_oline_config(ROOT/'config/robot_task_oline.yaml')
    assert remote['camera_detection_task']['ros__parameters']['target_source'] == 'tcp'
    assert remote['detect_controller']['ros__parameters']['approach_clearance'] == .03


def test_old_posture_file_without_transition2_is_still_supported(tmp_path):
    path = tmp_path/'old.yaml'
    path.write_text(yaml.safe_dump(dict(travel=[0.]*6, detection_ready=[1.]*6)))
    assert set(load_postures(path)) == {'travel', 'detection_ready'}


@pytest.mark.parametrize('angles', [[0.]*5, [0.,0.,True,0.,0.,0.], [0.,0.,float('nan'),0.,0.,0.]])
def test_malformed_real_transition2_is_rejected(tmp_path, angles):
    path = tmp_path/'invalid.yaml'
    path.write_text(yaml.safe_dump(dict(travel=[0.]*6, detection_ready=[0.]*6, detection_ready_2=angles)))
    with pytest.raises(ValueError, match='detection_ready_2'):
        load_postures(path)


@pytest.mark.parametrize('clamped', [True, False])
def test_real_posture_checks_final_joint_goal_before_any_execution(clamped):
    postures = load_postures(ROOT/'config/robot_task.yaml')
    start, target = postures['detection_ready'], postures['detection_ready_2']
    actual = target.copy()
    if clamped:
        actual[2] = np.deg2rad(171.405633)
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(reversed(JOINTS))
    trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=list(reversed(actual)))]
    result = SimpleNamespace(error_code=MoveItErrorCodes(val=1), trajectory=trajectory)
    node = SimpleNamespace(postures=postures, prepare_control_mode=Mock(), wait_ready=Mock(),
        current=Mock(return_value=start), validate=Mock(), write=Mock(), execute_segments=Mock(),
        planner=Mock(), future=Mock(return_value=SimpleNamespace(motion_plan_response=result)),
        report={'segments':[]}, scale=.1, state=JointPostureControl.state)
    node.plan = lambda a,b,label: JointPostureControl.plan(node,a,b,label)
    if clamped:
        with pytest.raises(RuntimeError, match='拒绝执行'):
            JointPostureControl.move_posture(node,'detection_ready_2')
        node.execute_segments.assert_not_called()
        assert node.report['native_error_code'] == MoveItErrorCodes.INVALID_MOTION_PLAN
        assert node.report['segments'][0]['planned_end_rad'][2] == pytest.approx(np.deg2rad(171.405633))
    else:
        JointPostureControl.move_posture(node,'detection_ready_2')
        segments = node.execute_segments.call_args.args[0]
        assert len(segments) == 1 and segments[0][0] == 'to_detection_ready_2'
        assert segments[0][2] == target
    request = node.planner.call_async.call_args.args[0].motion_plan_request
    assert request.goal_constraints[0].joint_constraints[2].position == pytest.approx(np.pi)
