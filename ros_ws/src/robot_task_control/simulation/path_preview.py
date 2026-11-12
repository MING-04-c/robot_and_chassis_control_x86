#!/usr/bin/env python3
"""Plan YAML arm steps and publish RViz animation, never execute trajectories."""
from collections import deque
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import signal
from time import monotonic

import rclpy
import numpy as np
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
import yaml
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import (
    Constraints, DisplayTrajectory, MotionPlanRequest, MoveItErrorCodes,
    OrientationConstraint, PositionConstraint, RobotState, RobotTrajectory,
)
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK, GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool

from eli_cs_robot_moveit_config.joint_postures import JOINTS, joint_constraints, load_postures
from geminipro_camer_calibration.capture_geometry import frozen_probe_targets
from geminipro_camer_calibration.geometry import pose_matrix, to_pose, tool_goal, rpy_degrees_rotation


class PreviewSceneError(RuntimeError):
    """The planning environment is unavailable, rather than a rejected goal."""


def finite_vector(value, size, name):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{name}: expected {size} finite numbers")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError(f"{name}: expected {size} finite numbers")
    return [float(v) for v in value]


def quaternion_from_degrees(rpy):
    roll, pitch, yaw = [math.radians(v) / 2.0 for v in finite_vector(rpy, 3, "orientation")]
    sr, cr, sp, cp, sy, cy = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch), math.sin(yaw), math.cos(yaw)
    return Quaternion(x=sr*cp*cy-cr*sp*sy, y=cr*sp*cy+sr*cp*sy,
                      z=cr*cp*sy-sr*sp*cy, w=cr*cp*cy+sr*sp*sy)


def step_pose(step):
    if "_pose" in step:
        return step["_pose"]
    target = step["target_pose"]
    xyz = finite_vector(target["position"], 3, "position")
    return Pose(position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                orientation=quaternion_from_degrees(target["orientation"]))


def pose_constraints(step):
    pose = step_pose(step)
    tolerance = float(step.get("arrival_tolerance", 0.005))
    angle = math.radians(float(step.get("arrival_orientation_tolerance_deg", 1.0)))
    if not math.isfinite(tolerance) or not 0 < tolerance <= 0.05 or not math.isfinite(angle) or not 0 < angle <= math.pi:
        raise ValueError("Invalid Cartesian goal tolerance")
    position = PositionConstraint(link_name=step.get("end_effector_frame", "tool0"), weight=1.0)
    position.header.frame_id = step.get("frame_id", "base_link")
    position.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tolerance])]
    position.constraint_region.primitive_poses = [Pose(position=deepcopy(pose.position), orientation=Quaternion(w=1.0))]
    orientation = OrientationConstraint(link_name=position.link_name, orientation=pose.orientation, weight=1.0,
        absolute_x_axis_tolerance=angle, absolute_y_axis_tolerance=angle, absolute_z_axis_tolerance=angle)
    orientation.header.frame_id = position.header.frame_id
    return Constraints(position_constraints=[position], orientation_constraints=[orientation])


def load_preview(config_path, workflow_override=""):
    config_path = Path(config_path).expanduser().resolve()
    config = yaml.safe_load(config_path.read_text())
    workflow_path = Path(workflow_override or config["workflow_file"]).expanduser()
    if not workflow_path.is_absolute():
        workflow_path = config_path.parent / workflow_path
    workflow_path = workflow_path.resolve()
    data = yaml.safe_load(workflow_path.read_text())
    postures = load_postures(workflow_path)
    initial = config.get("initial_posture", "travel")
    if initial not in postures:
        raise ValueError("initial_posture must name a configured joint posture")
    steps = data["workflow"]["steps"]
    if not isinstance(steps, list) or not steps:
        raise ValueError("workflow.steps must be a nonempty list")
    for step in steps:
        action = step["action"]
        if action == "arm_posture" and step.get("posture") not in postures:
            raise ValueError(f"Unknown posture: {step.get('posture')}")
        if action == "arm_move":
            if step.get("control_mode", "moveit") != "moveit" or step.get("target_pose", {}).get("position_mode") != "absolute":
                raise ValueError("Preview supports only absolute MoveIt arm_move targets")
            if step.get("frame_id", "base_link") != "base_link":
                raise ValueError("Preview Cartesian targets must use base_link")
            pose_constraints(step)
        if action not in ("arm_posture", "arm_move", "camera_detection", "stop", "wait"):
            raise ValueError(f"Unsupported preview action: {action}; not silently skipped")
    for key, default, lower, upper in (
        ("planning_time_sec", 15.0, 0.1, 120.0), ("planning_attempts", 5, 1, 50),
        ("velocity_scaling", 0.1, 0.001, 0.3), ("acceleration_scaling", 0.1, 0.001, 0.3),
        ("startup_timeout_sec", 120.0, 1.0, 600.0),
        ("detection_tilt_limit_deg", 0.0, 0.0, 15.0),
    ):
        value = config.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(f"Invalid {key}")
        if key == "planning_attempts" and int(value) != value:
            raise ValueError("planning_attempts must be an integer")
        config[key] = value
    config.setdefault("report_file", "/tmp/robot_task_path_preview.json")
    config.setdefault("planning_pipeline", "ompl")
    config.setdefault("planner_id", "RRTConnectkConfigDefault")
    if (config["planning_pipeline"], config["planner_id"]) not in (
        ("ompl", "RRTConnectkConfigDefault"), ("ompl", "RRTstarkConfigDefault"),
    ):
        raise ValueError("Preview uses the original OMPL planning pipeline")
    if any(step["action"] == "camera_detection" for step in steps):
        base_path = Path(config.get("task_config_file", "../config/robot_task.yaml")).expanduser()
        if not base_path.is_absolute():
            base_path = config_path.parent / base_path
        base = yaml.safe_load(base_path.read_text())
        for key in ("camera_detection_task", "detect_controller"):
            parameters = dict(base[key]["ros__parameters"])
            parameters.update(data.get(key, {}).get("ros__parameters", {}))
            config[key] = parameters
        camera = config["camera_detection_task"]
        detect = config["detect_controller"]
        if detect.get("preposition_tool_z_spin", False) or any(detect.get("preposition_rp_tolerance_deg", [0., 0.])):
            raise ValueError("Preview requires direct pose detection; disable orientation candidate selection")
        if config["detection_tilt_limit_deg"] and (
                detect.get("approach_axis") != "tool_positive_z" or detect.get("lock_tool_orientation", False)):
            raise ValueError("Preview tilt search requires tool_positive_z and unlocked path orientation")
        if camera["target_source"] != "local":
            raise ValueError("Simulation requires local camera_points_mm; no TCP receiver is started")
        points = camera["camera_points_mm"]
        finite_vector(points, len(points), "camera_points_mm")
        if not points or len(points) % 3:
            raise ValueError("camera_points_mm requires xyz triples")
        frozen_probe_targets(np.eye(4), np.eye(4), np.asarray(points).reshape(-1, 3))
    return config, str(workflow_path), postures[initial], postures, steps


def robot_state(positions):
    return RobotState(joint_state=JointState(name=list(JOINTS), position=list(positions)), is_diff=False)


def trajectory_end(start, trajectory):
    joints = trajectory.joint_trajectory
    if not joints.points or set(joints.joint_names) != set(JOINTS):
        raise RuntimeError("Planner returned empty trajectory or unexpected joints")
    values = dict(zip(joints.joint_names, joints.points[-1].positions))
    result = [values[name] for name in JOINTS]
    if not all(math.isfinite(value) for value in result):
        raise RuntimeError("Planner returned nonfinite positions")
    first = dict(zip(joints.joint_names, joints.points[0].positions))
    if max(abs(first[name]-value) for name, value in zip(JOINTS, start)) > 0.01:
        raise RuntimeError("Planned segment does not start at the previous segment endpoint")
    return result


def merge_trajectories(segments):
    """One message/trajectory prevents segment restarts and competing displays."""
    merged = RobotTrajectory()
    offset_ns = 0
    previous = None
    for segment in segments:
        joints = segment.joint_trajectory
        if not joints.points or set(joints.joint_names) != set(JOINTS):
            raise ValueError("Cannot merge empty trajectory or unexpected joints")
        if not merged.joint_trajectory.joint_names:
            merged.joint_trajectory.joint_names = list(JOINTS)
        indices = [joints.joint_names.index(name) for name in JOINTS]
        for index, original in enumerate(joints.points):
            point = deepcopy(original)
            for field in ("positions", "velocities", "accelerations", "effort"):
                values = getattr(point, field)
                if values:
                    setattr(point, field, [values[i] for i in indices])
            if index == 0 and previous is not None:
                if max(abs(a-b) for a, b in zip(previous, point.positions)) > 0.01:
                    raise ValueError("Discontinuous preview segments")
                continue
            stamp = offset_ns + original.time_from_start.sec*1_000_000_000 + original.time_from_start.nanosec
            if merged.joint_trajectory.points:
                last = merged.joint_trajectory.points[-1].time_from_start
                stamp = max(stamp, last.sec*1_000_000_000 + last.nanosec + 1)
            point.time_from_start = Duration(sec=stamp//1_000_000_000, nanosec=stamp % 1_000_000_000)
            merged.joint_trajectory.points.append(point)
        previous = [joints.points[-1].positions[i] for i in indices]
        last = merged.joint_trajectory.points[-1].time_from_start
        offset_ns = last.sec*1_000_000_000 + last.nanosec
    return merged


def detection_steps(config, transforms, photo_joints, tool_tilt_degrees=(0.0, 0.0)):
    """Reuse production camera/probe geometry; preview nominal contact, not force."""
    camera, detect = config["camera_detection_task"], config["detect_controller"]
    base_camera, base_probe, base_tool = transforms
    mounting = np.linalg.inv(base_tool) @ base_probe
    lever_arm = float(np.linalg.norm(mounting[:3, 3]))
    planning_angle = min(math.radians(float(detect["orientation_tolerance_deg"])),
                         float(detect["pose_tolerance"]) / (4.0 * max(lever_arm, 1e-6)))
    motion_settings = {"_velocity_scaling": float(detect["velocity_scaling"]),
                       "_acceleration_scaling": float(detect["acceleration_scaling"]),
                       "_planning_time_sec": 5.0}
    points = np.asarray(camera["camera_points_mm"], dtype=float).reshape(-1, 3)
    targets = frozen_probe_targets(base_camera, base_probe, points)
    steps = []
    for index, target in enumerate(targets, 1):
        final = tool_goal(target, mounting)
        # Simulation experiment: keep the frozen BASE-frame probe contact XYZ.
        # A tilt changes tool0 XYZ as well as the contact axis; recompute both.
        final[:3, :3] = final[:3, :3] @ rpy_degrees_rotation([*tool_tilt_degrees, 0.0])
        final[:3, 3] = target[:3, 3] - final[:3, :3] @ mounting[:3, 3]
        if detect["approach_axis"] == "tool_positive_z":
            axis = final[:3, 2]
        elif detect["approach_axis"] == "base_negative_z":
            axis = np.array([0., 0., -1.])
        else:
            raise ValueError("Unsupported detection approach_axis")
        clearance = float(detect["approach_clearance"])
        if not math.isfinite(clearance) or not 0 < clearance <= 0.2:
            raise ValueError("Invalid detection approach_clearance")
        pre = final.copy()
        pre[:3, 3] -= axis*clearance
        for transform in (target, final, pre, pre @ mounting):
            if (np.any(transform[:3, 3] < detect["workspace_min"]) or
                    np.any(transform[:3, 3] > detect["workspace_max"])):
                raise ValueError("Detection preview target outside configured workspace")
        for name, transform, cartesian, speed in (
            ("precontact", pre, False, 0.0),
            ("nominal_contact", final, True, float(detect["approach_speed"])),
            ("retract", pre, True, 0.005),
        ):
            steps.append({"name": f"detect_{index}_{name}", "action": "arm_move",
                          "end_effector_frame": detect["tool_frame"], "_pose": to_pose(transform),
                          "_cartesian": cartesian, "_cartesian_speed": speed,
                          "arrival_tolerance": float(detect["pose_tolerance"])/2,
                          "arrival_orientation_tolerance_deg": math.degrees(planning_angle),
                          "_cartesian_step": float(detect["moveit_cartesian_step"]) if name == "nominal_contact" else 0.001,
                          **motion_settings})
        steps.append({"name": f"detect_{index}_return_photo", "action": "arm_posture",
                      "_joint_goal": list(photo_joints), **motion_settings})
    return steps


def detection_tilts(limit):
    """Try the original orientation first, then increasing local X/Y tilt rings."""
    yield (0.0, 0.0)
    radii = sorted(set([float(v) for v in np.arange(5.0, limit, 5.0)] + ([limit] if limit else [])))
    for radius in radii:
        for angle in range(0, 360, 45):
            yield (radius * math.cos(math.radians(angle)), radius * math.sin(math.radians(angle)))


class PathPreview(Node):
    def __init__(self):
        if os.environ.get("ROS_DOMAIN_ID") != "190" or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST":
            raise RuntimeError("Use path_preview.launch.py: preview requires isolated domain 190 / LOCALHOST")
        super().__init__("path_preview")
        path = self.declare_parameter("config_file", "").value
        override = self.declare_parameter("workflow_file", "").value
        self.config, workflow, self.initial, self.postures, self.steps = load_preview(path, override)
        self.report = {"mode": "PLANNING_ONLY", "domain": 190, "workflow_file": workflow,
                       "planning_pipeline": self.config["planning_pipeline"], "planner_id": self.config["planner_id"],
                       "initial_joint_degrees": [math.degrees(v) for v in self.initial],
                       "segments": [], "skipped": [], "status": "WAITING"}
        self.ground = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/robot_control/ground_ready", self.ground_status, qos)
        self.joints = self.create_publisher(JointState, "/joint_states", 10)
        self.display = self.create_publisher(DisplayTrajectory, "/path_preview/display_planned_path", qos)
        self.planner = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.animation = DisplayTrajectory(model_id="cs", trajectory_start=robot_state(self.initial))
        self.preview_ready = False
        self.last_subscribers = -1
        self.subscriber_ready_at = None
        self.create_timer(0.05, self.publish_joints)
        self.create_timer(0.5, self.publish_paths)

    def ground_status(self, message):
        self.ground = message.data

    def publish_joints(self):
        message = JointState(name=list(JOINTS), position=self.initial)
        message.header.stamp = self.get_clock().now().to_msg()
        self.joints.publish(message)

    def publish_paths(self):
        if not self.preview_ready or not self.animation.trajectory:
            return
        subscribers = self.display.get_subscription_count()
        now = monotonic()
        # Allow RViz's model to load after subscribing, but never keep queuing
        # the same path: queued messages can restart a finished animation.
        if subscribers > self.last_subscribers:
            self.subscriber_ready_at = now + 2.0
        if not subscribers:
            self.subscriber_ready_at = None
        if subscribers and self.subscriber_ready_at is not None and now >= self.subscriber_ready_at:
            self.display.publish(self.animation)
            self.subscriber_ready_at = None
        self.last_subscribers = subscribers

    def finish_display(self):
        if self.animation.trajectory:
            self.animation.trajectory = [merge_trajectories(self.animation.trajectory)]
            self.report["display_status"] = "COMPLETE" if self.report["status"] == "PREVIEW_READY" else "PARTIAL_STOP"
            self.save_report()
            self.preview_ready = True
            self.publish_paths()

    def save_report(self):
        path = Path(self.config.get("report_file", "/tmp/robot_task_path_preview.json")).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.report, ensure_ascii=False, indent=2) + "\n")

    def call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not rclpy.ok():
            raise ExternalShutdownException()
        if not future.done():
            future.cancel()
            raise RuntimeError(f"Service timeout: {client.srv_name}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"Empty service response: {client.srv_name}")
        return result

    def validate(self, positions, label):
        response = self.call(self.validity, GetStateValidity.Request(
            robot_state=robot_state(positions), group_name="cs_manipulator"))
        if not response.valid:
            contacts = [f"{c.contact_body_1} <-> {c.contact_body_2} (depth={c.depth:.5f}m)" for c in response.contacts]
            raise RuntimeError(f"{label}: invalid/colliding state; " + ("; ".join(contacts) or "check joint limits"))

    def expand_detection(self, current):
        camera, detect = self.config["camera_detection_task"], self.config["detect_controller"]
        request = GetPositionFK.Request(robot_state=robot_state(current),
            fk_link_names=[camera["camera_frame"], camera["probe_frame"], detect["tool_frame"]])
        request.header.frame_id = "base_link"
        response = self.call(self.fk, request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS or len(response.pose_stamped) != 3:
            raise RuntimeError("Cannot freeze camera/probe transforms at planned photo pose")
        self.get_logger().warning("检测仅预览 YAML 测试点的名义接触位置，不模拟力反馈，不读取厚度")
        transforms = [pose_matrix(p.pose) for p in response.pose_stamped]
        if self.config.get("detection_tilt_limit_deg", 0.0):
            return self.search_detection(transforms, current)
        return detection_steps(self.config, transforms, current)

    def search_detection(self, transforms, photo_joints):
        """Accept a tilt only after precontact/contact/retract/photo-return ALL plan."""
        trials = self.report.setdefault("detection_orientation_trials", [])
        accepted = []
        points = len(self.config["camera_detection_task"]["camera_points_mm"]) // 3
        start_joints = list(photo_joints)
        for point_index in range(points):
            for tilt in detection_tilts(self.config["detection_tilt_limit_deg"]):
                trial = {"point_index": point_index + 1, "local_tool_xy_tilt_deg": list(tilt),
                         "status": "CHECKING", "simulation_only": True}
                trials.append(trial)
                self.save_report()
                self.get_logger().info(f"仿真检测点{point_index+1}：尝试工具局部倾角 {tilt} 度；探针接触XYZ不变")
                candidate = []
                current = list(start_joints)
                try:
                    group = detection_steps(self.config, transforms, photo_joints, tilt)[4*point_index:4*point_index+4]
                    contact = pose_matrix(group[1]["_pose"])
                    mounting = np.linalg.inv(transforms[2]) @ transforms[1]
                    trial["probe_contact_xyz_m"] = (contact @ mounting)[:3, 3].tolist()
                    trial["approach_axis_base"] = contact[:3, 2].tolist()
                    for step in group:
                        trial["checking_segment"] = step["name"]
                        planned, current, segment = self.plan_step(step, current)
                        if step["action"] == "arm_move":
                            segment.update(self.verify_probe_endpoint(step, current, mounting))
                        elif max(abs(a-b) for a, b in zip(current, photo_joints)) > .01:
                            raise RuntimeError("Candidate did not return to the photo joint posture")
                        candidate.append(dict(step, _preplanned=(planned, current, segment)))
                except PreviewSceneError:
                    raise
                except RuntimeError as error:
                    trial.update(status="REJECTED", reason=str(error))
                    self.save_report()
                    continue
                trial.update(status="ACCEPTED", segments=[s["name"] for s in group])
                self.save_report()
                self.get_logger().warning(f"仅仿真选定检测倾角 {tilt} 度；已规划接近、名义接触、撤离和回拍照位")
                accepted.extend(candidate)
                start_joints = list(current)
                break
            else:
                raise RuntimeError(f"Detection point {point_index+1}: no complete collision-checked path within preview tilt limit")
        return accepted

    def verify_probe_endpoint(self, step, joints, mounting):
        """Check the actual planned tip, not just the requested tool0 position."""
        request = GetPositionFK.Request(robot_state=robot_state(joints),
            fk_link_names=[self.config["detect_controller"]["tool_frame"]])
        request.header.frame_id = "base_link"
        response = self.call(self.fk, request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS or len(response.pose_stamped) != 1:
            raise RuntimeError("Cannot verify candidate tool endpoint with FK")
        actual = pose_matrix(response.pose_stamped[0].pose)
        target = pose_matrix(step_pose(step))
        error = float(np.linalg.norm((actual @ mounting)[:3, 3] - (target @ mounting)[:3, 3]))
        angle = math.degrees(math.acos(float(np.clip(
            (np.trace(target[:3, :3].T @ actual[:3, :3])-1.)/2., -1., 1.))))
        detect = self.config["detect_controller"]
        if error > detect["pose_tolerance"] or angle > detect["orientation_tolerance_deg"]:
            raise RuntimeError(f"Candidate probe endpoint error {error*1000:.3f}mm / {angle:.3f}deg exceeds tolerance")
        return {"probe_endpoint_error_m": error, "tool_endpoint_angle_error_deg": angle}

    def plan_motion(self, motion, segment):
        response = self.call(self.planner, GetMotionPlan.Request(motion_plan_request=motion),
                             self.config["planning_time_sec"] + 30.0).motion_plan_response
        return response

    def plan_step(self, step, current, segment=None):
        # A five-second failed OMPL call can delay the ground monitor's scene
        # query. Require a fresh successful acknowledgement before the retry.
        deadline = monotonic() + 10.0
        while not self.ground and rclpy.ok() and monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.ground:
            raise PreviewSceneError("Ground collision scene no longer ready")
        segment = segment if segment is not None else {"name": step["name"], "status": "PLANNING"}
        if step["action"] == "arm_posture":
            goal = step["_joint_goal"] if "_joint_goal" in step else self.postures[step["posture"]]
            self.validate(goal, step["name"])
            constraints = joint_constraints(goal)
        else:
            pose = step_pose(step)
            segment["target_xyz_m"] = [pose.position.x, pose.position.y, pose.position.z]
            segment["target_quaternion_xyzw"] = [pose.orientation.x, pose.orientation.y,
                                                 pose.orientation.z, pose.orientation.w]
            constraints = Constraints() if step.get("_cartesian") else pose_constraints(step)
        segment["planner"] = "Cartesian" if step.get("_cartesian") else self.config["planner_id"]
        planning_time = float(step.get("_planning_time_sec", self.config["planning_time_sec"]))
        velocity = float(step.get("_velocity_scaling", self.config["velocity_scaling"]))
        acceleration = float(step.get("_acceleration_scaling", self.config["acceleration_scaling"]))
        segment.update(velocity_scaling=velocity, acceleration_scaling=acceleration)
        if step.get("_cartesian"):
            request = GetCartesianPath.Request(start_state=robot_state(current),
                group_name="cs_manipulator", link_name=step.get("end_effector_frame", "tool0"),
                waypoints=[step_pose(step)], max_step=float(step.get("_cartesian_step", 0.001)), jump_threshold=1.5,
                avoid_collisions=True, cartesian_speed_limited_link=step.get("end_effector_frame", "tool0"),
                max_cartesian_speed=step["_cartesian_speed"], max_velocity_scaling_factor=velocity,
                max_acceleration_scaling_factor=acceleration)
            request.header.frame_id = "base_link"
            response = self.call(self.cartesian, request, 30.0)
            segment.update(error_code=response.error_code.val, fraction=response.fraction)
            if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 0.999:
                segment["status"] = "FAILED"
                raise RuntimeError(f'{step["name"]}: Cartesian path incomplete, fraction={response.fraction:.3f}')
            planned, planning_time = response.solution, 0.0
        else:
            motion = MotionPlanRequest(group_name="cs_manipulator", pipeline_id=self.config["planning_pipeline"],
                planner_id=self.config["planner_id"], start_state=robot_state(current), goal_constraints=[constraints],
                allowed_planning_time=planning_time, num_planning_attempts=int(self.config["planning_attempts"]),
                max_velocity_scaling_factor=velocity, max_acceleration_scaling_factor=acceleration)
            response = self.plan_motion(motion, segment)
            segment["error_code"] = response.error_code.val
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                segment["status"] = "FAILED"
                raise RuntimeError(f'{step["name"]}: MoveIt planning failed, error_code={response.error_code.val}')
            planned, planning_time = response.trajectory, response.planning_time
        endpoint = trajectory_end(current, planned)
        self.validate(endpoint, step["name"] + " endpoint")
        segment.update(status="PLANNED", planning_time_sec=planning_time,
                       end_joint_degrees=[math.degrees(v) for v in endpoint],
                       joint_delta_degrees=[math.degrees(b-a) for a, b in zip(current, endpoint)],
                       waypoints=len(planned.joint_trajectory.points))
        return planned, endpoint, segment

    def run(self):
        self.save_report()
        self.get_logger().info("仅规划预览：ROS_DOMAIN_ID=190；不执行轨迹，不连接真实设备")
        deadline = monotonic() + self.config["startup_timeout_sec"]
        while rclpy.ok() and monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.ground and all(c.service_is_ready() for c in (self.planner, self.validity, self.fk, self.cartesian)):
                break
        else:
            raise RuntimeError("MoveIt / ground scene startup timeout")
        self.validate(self.initial, "initial posture")
        current = list(self.initial)
        pending = deque(self.steps)
        while pending:
            step = pending.popleft()
            action = step["action"]
            if action == "camera_detection":
                pending.extendleft(reversed(self.expand_detection(current)))
                continue
            if action not in ("arm_posture", "arm_move"):
                item = {"name": step.get("name", action), "action": action,
                        "reason": "NOT SIMULATED: preview includes arm pose moves only"}
                self.report["skipped"].append(item)
                self.get_logger().warning(str(item))
                continue
            if not self.ground:
                raise RuntimeError("Ground collision scene no longer ready")
            label = step["name"]
            self.get_logger().info(f"规划第{len(self.report['segments'])+1}段: {label}")
            segment = {"name": label, "status": "PLANNING"}
            self.report["segments"].append(segment)
            self.save_report()
            if "_preplanned" in step:
                planned, endpoint, saved = step["_preplanned"]
                trajectory_end(current, planned)
                segment.update(saved)
            else:
                planned, endpoint, _ = self.plan_step(step, current, segment)
            self.animation.trajectory.append(planned)
            current = endpoint
            self.save_report()
        self.report["status"] = "PREVIEW_READY"
        self.save_report()
        self.get_logger().info("规划预览完成，RViz 按轨迹时间连续播放一次。报告: " + self.config["report_file"])


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = None
    try:
        node = PathPreview()
        try:
            node.run()
        except ExternalShutdownException:
            raise
        except Exception as error:
            node.report.update(status="FAILED", error=str(error))
            if node.report["segments"] and node.report["segments"][-1]["status"] == "PLANNING":
                node.report["segments"][-1]["status"] = "FAILED"
            node.save_report()
            node.get_logger().error(str(error) + "；停止后续规划，成功部分仅预览一次并停在最后有效点，不循环，不移动真机")
        node.finish_display()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
