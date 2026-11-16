"""返程终点横线编号与路线终点位置的回归检查。"""
import sys
from pathlib import Path
from types import SimpleNamespace
from dataclasses import replace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from line_follow_node import LineFollowNode
from line_follow_control import ControlConfig
from stop_marker import StopMarkerGate


def test_near_recorded_origin_marker_is_final_even_when_reverse_start_bar_was_unseen():
    node = SimpleNamespace(
        config=SimpleNamespace(reverse_return_to_start=True),
        recorded_station_order=lambda: [dict(id=2), dict(id=1)],
        route_document=dict(station_events=[dict(action='ignored_departure')]),
        marker_gate=SimpleNamespace(current_identity=1, ledger=SimpleNamespace(
            counts=dict(encountered=1, rejected=0))),
        return_finish=object(),
        return_tracker=SimpleNamespace(diagnostics=dict(remaining_m=.159)),
        route_config=SimpleNamespace(finish_distance=.30),
    )
    assert not LineFollowNode.finish_marker_is_final(node)
    node.return_tracker.diagnostics['remaining_m'] = .057
    assert LineFollowNode.finish_marker_is_final(node)
    node.return_tracker.diagnostics['remaining_m'] = .31
    assert not LineFollowNode.finish_marker_is_final(node)


def test_matching_station_number_remains_final_in_30cm_zone():
    node = SimpleNamespace(
        config=SimpleNamespace(reverse_return_to_start=True),
        recorded_station_order=lambda: [dict(id=2), dict(id=1)],
        route_document=dict(station_events=[dict(action='ignored_departure')]),
        marker_gate=SimpleNamespace(current_identity=2, ledger=SimpleNamespace(
            counts=dict(encountered=2, rejected=0))),
        return_finish=object(),
        return_tracker=SimpleNamespace(diagnostics=dict(remaining_m=.20)),
        route_config=SimpleNamespace(finish_distance=.30),
    )
    assert LineFollowNode.finish_marker_is_final(node)


def test_arrival_near_origin_latches_final_instead_of_auto_resume_intermediate():
    config = replace(ControlConfig(), drive_direction='reverse', reverse_return_to_start=True,
                     reverse_ignore_start_marker=False)
    gate = StopMarkerGate()
    node = SimpleNamespace(
        config=config,
        recorded_station_order=lambda: [dict(id=2), dict(id=1)],
        route_document=dict(station_events=[dict(action='ignored_departure')]),
        marker_gate=gate,
        return_finish=object(),
        return_tracker=SimpleNamespace(diagnostics=dict(remaining_m=.159)),
        route_config=SimpleNamespace(finish_distance=.30),
    )
    observations = [(150., .159), (165., .145), (180., .13), (195., .115),
                    (210., .09), (230., .065), (238., .057), (238., .057)]
    for index, (y, remaining) in enumerate(observations):
        node.return_tracker.diagnostics['remaining_m'] = remaining
        kind = 'final' if LineFollowNode.finish_marker_is_final(node) else 'intermediate'
        gate.observe([dict(x=320., y=y, kind=kind)], 480, config,
                     now=index*.05, acquisition_frames=3)
    assert gate.arrived
    assert gate.latched_kind == 'final'
    assert gate.state(config) == 'stopped_final'
