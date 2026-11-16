"""倒车充电末段：横线定位、先摆正回中、有限直线接近；单位米/弧度。"""
import math
from dataclasses import replace
from reverse_visual import AlignmentSequencer, strafe_command


class ChargingDock:
    def __init__(self, route, max_distance=.10):
        if not .01 <= max_distance <= .10:
            raise ValueError('充电额外搜索距离必须在0.01～0.10m内')
        self.approach_route = route
        self.route = replace(route, finish_goal_heading=math.radians(3.), finish_goal_lateral=.04,
                             visual_goal_lateral_m=.015)
        self.limit = max_distance
        self.sequencer = AlignmentSequencer()
        self.anchor = self.previous = None
        self.travel = self.forward_travel = 0.
        self.marker_anchor = None
        self.started = self.marker_at = None
        self.state = 'waiting_entry'
        self.fault = None
        self.done = False
        self.last_stamp = None
        self.last_direction = 0

    def fail(self, reason):
        self.fault = self.state = 'charging_dock_'+reason
        return 0., 0., 0., self.fault

    def command(self, pose, feedback, stamp, now, config, stopped, marker_arrived, marker_error):
        """输入实际里程计、同帧近端线、横线平均误差；输出vx/vy/w及状态。"""
        if self.fault:
            return 0., 0., 0., self.fault
        if self.started is None:
            self.started, self.anchor, self.previous = now, tuple(pose[:2]), tuple(pose[:2])
        step = math.dist(pose[:2], self.previous)
        self.travel += step
        if self.last_direction == 1:
            self.forward_travel += step
        if self.forward_travel > .05+1e-6:
            return self.fail('forward_limit')
        self.previous = tuple(pose[:2])
        if marker_arrived and self.marker_anchor is None:
            self.marker_anchor, self.marker_at = tuple(pose[:2]), now
            self.marker_travel = self.travel
        if now-self.started > 90.:
            return self.fail('timeout')
        if self.marker_anchor is not None:
            if self.travel-self.marker_travel > self.limit+1e-6 or math.dist(pose[:2], self.marker_anchor) > self.limit+1e-6:
                return self.fail('search_limit')
            if now-self.marker_at > 30.:
                return self.fail('charge_not_detected')
        if feedback is None:
            return self.fail('line_unavailable')
        error = abs(feedback['lateral_error_m'] if feedback.get('metric') else feedback['error_normalized'])
        if error > (.10 if feedback.get('metric') else .35) or abs(feedback['near_heading_rad']) > math.radians(35.):
            return self.fail('alignment_out_of_range')
        # 接近和越线微调共用横线慢速参数，不能落入底盘0.01m/s驻车死区。
        # 不抬高操作员设置的速度上限；上限不足以有效行驶时明确停车报错。
        speed_limit = min(config.forward_speed, config.reverse_max_speed)
        if speed_limit == 0.:
            return 0., 0., 0., 'speed_zero'
        creep_speed = min(config.stop_marker_creep_speed, speed_limit)
        if creep_speed <= .01:
            return self.fail('speed_below_motion_threshold')
        # 横线尚未到位时沿用普通返程容差和迟滞；横线到位后才严格对齐。
        alignment_route = self.route if marker_arrived else self.approach_route
        # 最终对齐也保留ready状态的1.5倍回差：横线附近像素噪声不能
        # 让门限两侧反复切换。目标切换/到位仍由sequencer用3张新帧确认。
        phase = self.sequencer.update(feedback, alignment_route, stamp, now,
                                       terminal=False, stopped=stopped)
        vy, w = strafe_command(feedback, config, alignment_route)
        # sequencer已负责连续新帧确认和停稳间隔，不再叠加第二个原始
        # 阈值计数；否则最终横线处的小抖动会反复清零纵向速度。
        if phase != 'ready':
            self.state = 'align_'+phase
            return 0., vy if phase == 'lateral' else 0., w if phase == 'heading' else 0., 'reverse_visual_finish_aligning'
        # 已摆正后只走直线；图像偏差明显增大时回到上面的对齐步骤。
        if marker_error is not None and marker_error < -config.stop_marker_overshoot_tolerance_ratio and not marker_arrived:
            if self.forward_travel > .05:
                return self.fail('forward_limit')
            if self.last_direction != 1 and not stopped:
                self.state = 'direction_settle'
                return 0., 0., 0., 'reverse_visual_finish_aligning'
            self.last_direction = 1
            self.state = 'forward_reposition'
            return creep_speed, 0., 0., 'reverse_visual_finish_tracking'
        if self.last_direction != -1 and not stopped:
            self.state = 'direction_settle'
            return 0., 0., 0., 'reverse_visual_finish_aligning'
        self.last_direction = -1
        self.state = 'charge_search' if marker_arrived else 'straight_approach'
        speed = creep_speed if marker_arrived or marker_error is not None else min(.02, speed_limit)
        return -speed, 0., 0., 'reverse_visual_finish_tracking'

    def snapshot(self):
        return dict(state=self.state, fault=self.fault, charged=self.done,
                    search_limit_m=self.limit, marker_reached=self.marker_anchor is not None,
                    marker_search_travel_m=(self.travel-self.marker_travel if self.marker_anchor is not None else 0.),
                    lateral_limit_m=.10, uncalibrated_lateral_limit_half_width=.35,
                    goal_heading_deg=3., goal_lateral_m=.015, goal_lateral_half_width=.04)
