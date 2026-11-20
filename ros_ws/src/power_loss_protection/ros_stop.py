#!/usr/bin/env python3
"""Bounded stop requests; never power on, unlock, recover or move home."""
import argparse
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, required=True)
    args = parser.parse_args()
    # Imports and DDS discovery are also bounded by the parent process deadline.
    import rclpy
    from action_msgs.srv import CancelGoal
    from std_srvs.srv import Trigger, SetBool
    from masc_chassis_msgs.srv import TaskControl
    rclpy.init()
    node = rclpy.create_node("power_loss_stop_requests")
    requests = []
    for name in ("/task_controller/stop", "/camera_detection_task/stop",
                 "/detect_controller/cancel", "/arm_joint_postures/cancel",
                 "/protective_stop_guard/cancel_recovery", "/dashboard_client/stop"):
        requests.append((name, Trigger, Trigger.Request()))
    for name in ("/chassis_velocity_mux/enable", "/line_follow_controller/enable",
                 "/admittance_controller/enable"):
        request = SetBool.Request()
        request.data = False
        requests.append((name, SetBool, request))
    request = TaskControl.Request()
    request.command = TaskControl.Request.STOP
    requests.append(("/chassis/task_control", TaskControl, request))
    for name in ("/scaled_joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal",
                 "/execute_trajectory/_action/cancel_goal", "/move_action/_action/cancel_goal"):
        requests.append((name, CancelGoal, CancelGoal.Request()))
    pending = [(name, node.create_client(kind, name), request) for name, kind, request in requests]
    sent, futures = set(), {}
    deadline = time.monotonic()+args.seconds
    try:
        while time.monotonic() < deadline:
            for name, client, request in pending:
                if name not in sent and client.service_is_ready():
                    futures[name] = client.call_async(request)
                    sent.add(name)
            rclpy.spin_once(node, timeout_sec=min(.02, max(0., deadline-time.monotonic())))
        for name, _, _ in pending:
            future = futures.get(name)
            if future is None:
                node.get_logger().warning(f"Stop service absent: {name}")
            elif not future.done():
                node.get_logger().warning(f"Stop response timeout: {name}")
            else:
                try:
                    node.get_logger().info(f"Stop response {name}: {future.result()}")
                except Exception as error:
                    node.get_logger().warning(f"Stop service failed {name}: {error}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
