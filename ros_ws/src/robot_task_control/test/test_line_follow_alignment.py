#!/usr/bin/env python3
"""Pure state-machine checks for startup alignment status handling."""

import unittest
from time import monotonic
from types import SimpleNamespace
from unittest.mock import Mock

from robot_task_control.workflow_controller import WorkflowController


class StartAlignmentWorkflowTests(unittest.TestCase):
    @staticmethod
    def workflow_state(status):
        now = monotonic()
        return SimpleNamespace(
            use_velocity_mux=True,
            phase='follow_tracking',
            follow_last_tick=now-.1,
            follow_status_time=now,
            follow_status=status,
            follow_tracking_seconds=3.0,
            follow_last_tracking=now-.5,
            follow_boundary_since=now-.2,
            follow_station_confirm_since=now-.2,
            fail_workflow=Mock(),
        )

    def test_alignment_does_not_consume_line_progress_timeout(self):
        for status in ('start_alignment_settling', 'start_alignment_waiting_for_stable_image',
                       'start_alignment_moving_forward', 'start_alignment_moving_reverse',
                       'start_alignment_moving_yaw', 'start_alignment_moving_lateral',
                       'start_alignment_departing_dock', 'start_alignment_settling_after_departure',
                       'start_alignment_waiting_for_line', 'start_alignment_waiting_for_stop'):
            with self.subTest(status=status):
                node = self.workflow_state(status)
                previous = node.follow_last_tracking
                WorkflowController.advance_line_follow(node, {'seconds': 1.0})
                self.assertEqual(node.follow_tracking_seconds, 3.0)
                self.assertGreater(node.follow_last_tracking, previous)
                self.assertIsNone(node.follow_boundary_since)
                self.assertIsNone(node.follow_station_confirm_since)
                node.fail_workflow.assert_not_called()

    def test_alignment_failure_stops_workflow(self):
        node = self.workflow_state('start_alignment_failed_ground_mapping_invalid')
        WorkflowController.advance_line_follow(node, {'seconds': 1.0})
        node.fail_workflow.assert_called_once()
        self.assertIn('起步横线对齐失败', node.fail_workflow.call_args.args[0])

    def test_standard_route_failure_stops_without_advancing_workflow(self):
        for status in ('standard_visual_loss_timeout', 'standard_visual_loss_distance_limit',
                       'standard_route_deviation', 'standard_finish_search_limit'):
            node = self.workflow_state(status)
            WorkflowController.advance_line_follow(node, {'seconds': 1.0})
            node.fail_workflow.assert_called_once()
            self.assertIn(status, node.fail_workflow.call_args.args[0])
            self.assertEqual(node.follow_tracking_seconds, 3.0)

    def test_marker_confirmation_uses_yaml_timeout_not_a_fixed_sleep(self):
        for timeout, should_fail in ((1., True), (5., False)):
            node = self.workflow_state('stop_marker_settling')
            node.follow_station_confirm_since = monotonic()-2.
            WorkflowController.advance_line_follow(node, {
                'seconds': 1., 'marker_confirm_timeout_sec': timeout})
            assert node.fail_workflow.called is should_fail
            self.assertEqual(node.follow_tracking_seconds, 3.)



if __name__ == '__main__':
    unittest.main()
