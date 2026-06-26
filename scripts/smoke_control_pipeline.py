#!/usr/bin/env python3

import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger


LOG_DIR = Path(os.environ.get("ANTSY_TEST_LOG_DIR", "/tmp/antsy_smoke_tests"))
ACTUATOR_COUNT = 18
MOTION_DELTA_MIN = 0.01
RESET_DELTA_MAX = 0.02
ODOM_FORWARD_MIN = 0.03
ODOM_LATERAL_MAX = 0.04


def start_process(name, command):
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"{name}.log"
    log_file = log_path.open("w")
    process = subprocess.Popen(
        command,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    return process, log_file, log_path


def stop_process(process, log_file):
    if process.poll() is None:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        try:
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=4.0)
    log_file.close()


def fail(message, logs):
    print(f"FAIL: {message}", file=sys.stderr)
    for label, path in logs:
        if path.exists():
            print(f"\n--- {label}: {path} ---", file=sys.stderr)
            print("".join(path.read_text(errors="replace").splitlines(True)[-80:]), file=sys.stderr)
    raise SystemExit(1)


def max_delta(a, b):
    return max(abs(x - y) for x, y in zip(a.position, b.position))


class ControlSmokeNode(Node):
    def __init__(self):
        super().__init__("antsy_control_pipeline_smoke")
        self.latest_actuators = None
        self.latest_odom = None
        self.create_subscription(Actuators, "/actuators", self._actuator_callback, 10)
        self.create_subscription(Odometry, "/leg_odom", self._odom_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.reset_client = self.create_client(Trigger, "/control/reset")

    def _actuator_callback(self, msg):
        if len(msg.position) == ACTUATOR_COUNT and all(math.isfinite(v) for v in msg.position):
            self.latest_actuators = msg

    def _odom_callback(self, msg):
        pose = msg.pose.pose.position
        twist = msg.twist.twist
        values = [
            pose.x,
            pose.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
        ]
        if all(math.isfinite(v) for v in values):
            self.latest_odom = msg

    def wait_for_actuators(self, timeout):
        self.latest_actuators = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_actuators is not None:
                return self.latest_actuators
        raise TimeoutError("timed out waiting for /actuators")

    def collect_latest_actuators(self, duration):
        self.latest_actuators = None
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.latest_actuators is None:
            raise TimeoutError("timed out collecting fresh /actuators")
        return self.latest_actuators

    def wait_for_odom(self, timeout):
        self.latest_odom = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_odom is not None:
                return self.latest_odom
        raise TimeoutError("timed out waiting for /leg_odom")

    def collect_latest_odom(self, duration):
        self.latest_odom = None
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.latest_odom is None:
            raise TimeoutError("timed out collecting fresh /leg_odom")
        return self.latest_odom

    def publish_forward_command(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = 0.18
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def reset_control(self):
        if not self.reset_client.wait_for_service(timeout_sec=8.0):
            raise TimeoutError("/control/reset service did not appear")
        future = self.reset_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                response = future.result()
                if not response.success:
                    raise RuntimeError(f"/control/reset failed: {response.message}")
                return
        raise TimeoutError("/control/reset service call timed out")


def main():
    launches = []
    logs = []
    rclpy.init()
    node = None
    try:
        description = start_process(
            "control_description",
            ["ros2", "launch", "antsy_description", "description.launch.py", "use_sim_time:=false"],
        )
        control = start_process(
            "control_node",
            ["ros2", "launch", "antsy_control", "follow_velocity_rectangle.launch.xml", "use_sim_time:=false"],
        )
        launches.extend([description, control])
        logs.extend([("description", description[2]), ("control", control[2])])

        node = ControlSmokeNode()
        idle = node.wait_for_actuators(timeout=12.0)
        idle_odom = node.wait_for_odom(timeout=4.0)

        node.publish_forward_command(duration=2.0)
        moving = node.wait_for_actuators(timeout=4.0)
        moving_odom = node.collect_latest_odom(duration=0.5)
        movement_delta = max_delta(idle, moving)
        if movement_delta < MOTION_DELTA_MIN:
            fail(
                f"/actuators did not respond enough to cmd_vel; max delta {movement_delta:.5f}",
                logs,
            )
        odom_forward_delta = moving_odom.pose.pose.position.x - idle_odom.pose.pose.position.x
        odom_lateral_delta = moving_odom.pose.pose.position.y - idle_odom.pose.pose.position.y
        if odom_forward_delta < ODOM_FORWARD_MIN:
            fail(
                f"/leg_odom did not move forward enough; x delta {odom_forward_delta:.5f}",
                logs,
            )
        if abs(odom_lateral_delta) > ODOM_LATERAL_MAX:
            fail(
                f"/leg_odom drifted laterally too much; y delta {odom_lateral_delta:.5f}",
                logs,
            )

        node.reset_control()
        reset = node.collect_latest_actuators(duration=0.5)
        reset_odom = node.collect_latest_odom(duration=0.5)
        reset_delta = max_delta(idle, reset)
        if reset_delta > RESET_DELTA_MAX:
            fail(
                f"/control/reset did not return close to startup actuators; max delta {reset_delta:.5f}",
                logs,
            )
        if abs(reset_odom.pose.pose.position.x) > 0.01 or abs(reset_odom.pose.pose.position.y) > 0.01:
            fail(
                "/control/reset did not reset /leg_odom close to the origin; "
                f"x={reset_odom.pose.pose.position.x:.5f}, y={reset_odom.pose.pose.position.y:.5f}",
                logs,
            )

        print(
            "PASS: cmd_vel changed /actuators "
            f"(max delta {movement_delta:.4f} rad) and /control/reset restored startup commands "
            f"(max delta {reset_delta:.4f} rad); /leg_odom moved forward "
            f"{odom_forward_delta:.4f} m with lateral drift {odom_lateral_delta:.4f} m."
        )
    except Exception as exc:
        fail(str(exc), logs)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        for process, log_file, _ in reversed(launches):
            stop_process(process, log_file)


if __name__ == "__main__":
    main()
