"""Mid-station direction changes preserve the original route anchor."""
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from collections import deque
from dataclasses import replace

import pytest
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from line_follow_node import LineFollowNode
from recorded_route import RouteConfig, StandardRouteFollower, ReverseRouteFollower
from line_follow_control import ControlConfig


def context():
    route = {'points': [[i*.1, 0., 0.] for i in range(21)]}
    config = RouteConfig()
    forward = StandardRouteFollower(route, (0., 0., 0.), config)
    reverse = ReverseRouteFollower(route, (2., 0., 0.), config)
    reverse.progress = 1.2
    return SimpleNamespace(enabled=False, navigation_mode='replay', record_route=False,
        odom_reason=Mock(return_value=None), standard_follower=forward,
        return_tracker=reverse, return_fault=None,
        latest_odom=dict(pose=(.8, 0., 0.), v=0., vy=0., w=0.),
        config=SimpleNamespace(drive_direction='reverse'),
        marker_gate=SimpleNamespace(arrived=True, latched_kind='intermediate'),
        standard_finish=object(), standard_navigator=object(), standard_fault='old-fault',
        standard_visual_loss=object())


def test_forward_resume_preserves_world_route_and_sets_midpoint_progress():
    node = context()
    follower, tracker = node.standard_follower, node.return_tracker
    points = list(follower.tracker.points)
    response = LineFollowNode.prepare_station_segment(node, 'forward', Trigger.Response())
    assert response.success
    assert node.station_segment_pending == 'forward'
    assert node.standard_follower is follower and node.return_tracker is tracker
    assert follower.tracker.points == points
    assert follower.tracker.progress == pytest.approx(.8)
    assert follower.tracker.previous == (.8, 0.)
    v, _, state = follower.command((.8, 0., 0.), .05)
    assert v > 0 and state == 'standard_route_tracking'


def test_home_resume_keeps_reverse_progress_and_finish_guard():
    node = context()
    node.return_finish = object()
    guard, tracker = node.return_finish, node.return_tracker
    response = LineFollowNode.prepare_station_segment(node, 'home', Trigger.Response())
    assert response.success and node.station_segment_pending == 'home'
    assert node.return_tracker is tracker and tracker.progress == 1.2
    assert node.return_finish is guard


@pytest.mark.parametrize('change', ['enabled', 'stale', 'moving', 'unconfirmed', 'off_route'])
def test_reject_unconfirmed_or_discontinuous_midpoint(change):
    node = context()
    if change == 'enabled':
        node.enabled = True
    elif change == 'stale':
        node.odom_reason.return_value = 'odom_timeout'
    elif change == 'moving':
        node.latest_odom['v'] = .02
    elif change == 'unconfirmed':
        node.marker_gate.arrived = False
    else:
        node.latest_odom['pose'] = (.8, .3, 0.)
    response = LineFollowNode.prepare_station_segment(node, 'forward', Trigger.Response())
    assert not response.success
    assert node.standard_follower.tracker.progress == 0.


def test_reverse_to_middle_requires_confirmed_final_station():
    node = context()
    assert not LineFollowNode.prepare_station_segment(node, 'station', Trigger.Response()).success
    node.config.drive_direction, node.marker_gate.latched_kind = 'forward', 'final'
    assert LineFollowNode.prepare_station_segment(node, 'station', Trigger.Response()).success


@pytest.mark.parametrize('segment', ['forward', 'home'])
def test_parameter_handoff_does_not_reload_or_reanchor_prepared_segment(segment):
    node = context()
    assert LineFollowNode.prepare_station_segment(node, segment, Trigger.Response()).success
    node.config = replace(ControlConfig(), drive_direction='reverse')
    node.checkpoint_invalid_reason = None
    node.station_forward_active = False
    node.start_alignment_required = lambda config: LineFollowNode.start_alignment_required(node, config)
    node.start_aligner = SimpleNamespace()
    node.marker_gate.reset = Mock()
    node.return_camera_frames = deque()
    node.departure = SimpleNamespace(reset=Mock())
    node.start_alignment_active = False
    node.stop, node.load_return_route, node.read_standard = Mock(), Mock(), Mock()
    node.flush_route = Mock()
    node.previous_v, node.previous_w, node.previous_vy = 0., 0., 0.
    tracker = node.return_tracker
    direction = 'forward' if segment == 'forward' else 'reverse'
    result = LineFollowNode.on_parameters(node, [Parameter('drive_direction', value=direction)])
    assert result.successful
    result = LineFollowNode.on_parameters(node, [Parameter('enabled', value=True)])
    assert result.successful and node.enabled
    assert node.return_tracker is tracker
    assert node.station_forward_active == (segment == 'forward')
    assert node.station_segment_pending == ''
    node.load_return_route.assert_not_called()
    node.read_standard.assert_not_called()
