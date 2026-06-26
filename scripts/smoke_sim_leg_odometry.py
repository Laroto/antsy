#!/usr/bin/env python3

import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter


LOG_DIR = Path(os.environ.get("ANTSY_TEST_LOG_DIR", "/tmp/antsy_smoke_tests"))
CMD_LINEAR_X = float(os.environ.get("ANTSY_SIM_SMOKE_LINEAR_X", "0.25"))
WARMUP_DURATION_SEC = float(os.environ.get("ANTSY_SIM_SMOKE_WARMUP", "2.0"))
RUN_DURATION_SEC = float(os.environ.get("ANTSY_SIM_SMOKE_DURATION", "10.0"))
COOLDOWN_DURATION_SEC = float(os.environ.get("ANTSY_SIM_SMOKE_COOLDOWN", "2.0"))
MAX_FORWARD_RELATIVE_ERROR = float(
    os.environ.get("ANTSY_SIM_SMOKE_MAX_FORWARD_RELATIVE_ERROR", "0.01")
)
MAX_HEADING_ERROR_RAD = float(os.environ.get("ANTSY_SIM_SMOKE_MAX_HEADING_ERROR", "0.03"))
MAX_COOLDOWN_POSITION_ERROR_M = float(
    os.environ.get("ANTSY_SIM_SMOKE_MAX_COOLDOWN_POSITION_ERROR", "0.03")
)
MAX_COOLDOWN_HEADING_ERROR_RAD = float(
    os.environ.get("ANTSY_SIM_SMOKE_MAX_COOLDOWN_HEADING_ERROR", "0.03")
)
MIN_SIM_FORWARD_M = float(os.environ.get("ANTSY_SIM_SMOKE_MIN_FORWARD", "1.80"))
MAX_SIM_LATERAL_M = float(os.environ.get("ANTSY_SIM_SMOKE_MAX_LATERAL", "0.15"))
MAX_LEG_LATERAL_M = float(os.environ.get("ANTSY_SIM_SMOKE_MAX_LEG_LATERAL", "0.08"))
LEG_ODOM_TRANSLATION_SCALE = float(
    os.environ.get("ANTSY_SIM_SMOKE_LEG_ODOM_TRANSLATION_SCALE", "1.0")
)
LEG_ODOM_PROPAGATE = os.environ.get(
    "ANTSY_SIM_SMOKE_LEG_ODOM_PROPAGATE", "0"
).lower() in ("1", "true", "yes", "on")
LEG_ODOM_MAX_PREDICTION_TIME = float(
    os.environ.get("ANTSY_SIM_SMOKE_LEG_ODOM_MAX_PREDICTION_TIME", "0.08")
)
MAX_WALL_TIMEOUT_SEC = float(os.environ.get("ANTSY_SIM_SMOKE_WALL_TIMEOUT", "40.0"))

DUMP_PROCESS_LOGS_ON_FAILURE = os.environ.get(
    "ANTSY_SIM_SMOKE_DUMP_PROCESS_LOGS_ON_FAILURE", "0"
).lower() in ("1", "true", "yes", "on")


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
        if not path.exists():
            continue

        print(f"{label} log: {path}", file=sys.stderr)

        if DUMP_PROCESS_LOGS_ON_FAILURE:
            print(f"\n--- {label}: {path} ---", file=sys.stderr)
            print(
                "".join(path.read_text(errors="replace").splitlines(True)[-100:]),
                file=sys.stderr,
            )

    raise SystemExit(1)


def stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def unwrap_angles(angles):
    if not angles:
        return []

    unwrapped = [angles[0]]

    for angle in angles[1:]:
        value = angle

        value = unwrapped[-1] + normalize_angle(value - unwrapped[-1])

        unwrapped.append(value)

    return unwrapped


def fit_slope(samples, index):
    if len(samples) < 2:
        raise ValueError("not enough samples for regression")

    times = [sample[0] - samples[0][0] for sample in samples]
    values = [sample[index] for sample in samples]

    if index == 3:
        values = unwrap_angles(values)

    n = float(len(samples))
    sum_t = sum(times)
    sum_v = sum(values)
    sum_tt = sum(t * t for t in times)
    sum_tv = sum(t * v for t, v in zip(times, values))

    denominator = n * sum_tt - sum_t * sum_t

    if abs(denominator) < 1e-9:
        raise ValueError("degenerate sample window for regression")

    return (n * sum_tv - sum_t * sum_v) / denominator


def sample_nearest(history, timestamp):
    if not history:
        raise ValueError("empty history")
    return min(history, key=lambda sample: abs(sample[0] - timestamp))


class SimulationOdometrySmokeNode(Node):
    def __init__(self):
        super().__init__(
            "antsy_sim_leg_odometry_smoke",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )

        self.sim_history = []
        self.leg_history = []

        self.create_subscription(Odometry, "/odom", self._sim_odom_callback, 20)
        self.create_subscription(Odometry, "/leg_odom", self._leg_odom_callback, 20)
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

    def _sim_odom_callback(self, msg):
        if self._odom_is_finite(msg):
            self.sim_history.append(
                (
                    stamp_to_seconds(msg.header.stamp),
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    yaw_from_quaternion(msg.pose.pose.orientation),
                )
            )

    def _leg_odom_callback(self, msg):
        if self._odom_is_finite(msg):
            self.leg_history.append(
                (
                    stamp_to_seconds(msg.header.stamp),
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    yaw_from_quaternion(msg.pose.pose.orientation),
                )
            )

    @staticmethod
    def _odom_is_finite(msg):
        pose = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        twist = msg.twist.twist

        values = [
            pose.x,
            pose.y,
            quat.x,
            quat.y,
            quat.z,
            quat.w,
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
        ]

        return all(math.isfinite(v) for v in values)

    def wait_for_odometry(self, timeout):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            if len(self.sim_history) >= 3 and len(self.leg_history) >= 3:
                return

        raise TimeoutError("timed out waiting for /odom and /leg_odom")

    def publish_command(self, linear_x):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = linear_x
        self.cmd_pub.publish(msg)

    def drive_and_collect(self, warmup_sec, duration_sec, cooldown_sec):
        self.wait_for_odometry(timeout=15.0)

        start_time = self.sim_history[-1][0]
        measurement_start = start_time + warmup_sec
        measurement_end = measurement_start + duration_sec
        cooldown_end = measurement_end + cooldown_sec
        deadline = time.monotonic() + MAX_WALL_TIMEOUT_SEC

        while time.monotonic() < deadline:
            current_sim_time = self.sim_history[-1][0] if self.sim_history else start_time
            command_x = CMD_LINEAR_X if current_sim_time < measurement_end else 0.0
            self.publish_command(command_x)
            rclpy.spin_once(self, timeout_sec=0.03)
            time.sleep(0.02)

            if not self.sim_history:
                continue

            if self.sim_history[-1][0] >= cooldown_end:
                return measurement_start, measurement_end, cooldown_end

        raise TimeoutError(
            f"simulation did not advance {warmup_sec + duration_sec + cooldown_sec:.1f}s within "
            f"{MAX_WALL_TIMEOUT_SEC:.1f}s wall time"
        )


def filter_window(history, begin, end):
    window = [sample for sample in history if begin <= sample[0] <= end]

    if len(window) < 20:
        raise ValueError(
            f"insufficient samples in analysis window [{begin:.3f}, {end:.3f}]: {len(window)}"
        )

    return window


def main():
    launches = []
    logs = []

    rclpy.init()
    node = None

    try:
        simulator = start_process(
            "simulation_leg_odometry_simulator",
            [
                "ros2",
                "launch",
                "antsy_simulation",
                "simulator.launch.xml",
                "run_headless:=true",
                "use_sim_time:=true",
                "realtime_factor:=1.0",
                "start_controller:=false",
            ],
        )

        description = start_process(
            "simulation_leg_odometry_description",
            [
                "ros2",
                "launch",
                "antsy_description",
                "description.launch.py",
                "use_sim_time:=true",
            ],
        )

        controller = start_process(
            "simulation_leg_odometry_controller",
            [
                "ros2",
                "run",
                "antsy_control",
                "follow_velocity_rectangle",
                "--ros-args",
                "-p",
                "use_sim_time:=true",
                "-p",
                "heading_hold.enabled:=false",
                "-p",
                f"leg_odometry.translation_scale:={LEG_ODOM_TRANSLATION_SCALE}",
                "-p",
                f"leg_odometry.propagate_on_invalid_update:={str(LEG_ODOM_PROPAGATE).lower()}",
                "-p",
                f"leg_odometry.max_prediction_time:={LEG_ODOM_MAX_PREDICTION_TIME}",
            ],
        )

        launches.extend([simulator, description, controller])

        logs.extend(
            [
                ("simulation", simulator[2]),
                ("description", description[2]),
                ("controller", controller[2]),
            ]
        )

        node = SimulationOdometrySmokeNode()

        measurement_start, measurement_end, cooldown_end = node.drive_and_collect(
            WARMUP_DURATION_SEC,
            RUN_DURATION_SEC,
            COOLDOWN_DURATION_SEC,
        )

        sim_window = filter_window(node.sim_history, measurement_start, measurement_end)
        leg_window = filter_window(node.leg_history, measurement_start, measurement_end)

        sim_vx = fit_slope(sim_window, 1)
        sim_vy = fit_slope(sim_window, 2)
        sim_wz = fit_slope(sim_window, 3)

        leg_vx = fit_slope(leg_window, 1)
        leg_vy = fit_slope(leg_window, 2)
        leg_wz = fit_slope(leg_window, 3)

        sim_forward = sim_vx * RUN_DURATION_SEC
        sim_lateral = sim_vy * RUN_DURATION_SEC

        leg_forward = leg_vx * RUN_DURATION_SEC
        leg_lateral = leg_vy * RUN_DURATION_SEC

        forward_relative_error = abs(leg_vx - sim_vx) / max(abs(sim_vx), 1e-6)
        heading_error = abs(leg_wz - sim_wz) * RUN_DURATION_SEC
        sim_start_pose = sample_nearest(node.sim_history, measurement_start)
        sim_end_pose = sample_nearest(node.sim_history, cooldown_end)
        leg_start_pose = sample_nearest(node.leg_history, measurement_start)
        leg_end_pose = sample_nearest(node.leg_history, cooldown_end)

        sim_cooldown_dx = sim_end_pose[1] - sim_start_pose[1]
        sim_cooldown_dy = sim_end_pose[2] - sim_start_pose[2]
        leg_cooldown_dx = leg_end_pose[1] - leg_start_pose[1]
        leg_cooldown_dy = leg_end_pose[2] - leg_start_pose[2]
        sim_cooldown_dyaw = normalize_angle(sim_end_pose[3] - sim_start_pose[3])
        leg_cooldown_dyaw = normalize_angle(leg_end_pose[3] - leg_start_pose[3])
        cooldown_x_error = leg_cooldown_dx - sim_cooldown_dx
        cooldown_y_error = leg_cooldown_dy - sim_cooldown_dy
        cooldown_position_error = math.hypot(
            cooldown_x_error,
            cooldown_y_error,
        )
        cooldown_heading_error = abs(normalize_angle(leg_cooldown_dyaw - sim_cooldown_dyaw))

        result_summary = (
            f"{RUN_DURATION_SEC:.2f}s after a {WARMUP_DURATION_SEC:.2f}s warmup at "
            f"cmd_vel.x={CMD_LINEAR_X:.2f} m/s, then cooled down for {COOLDOWN_DURATION_SEC:.2f}s; "
            f"sim trend=({sim_forward:.3f} m, {sim_lateral:.3f} m, {sim_wz * RUN_DURATION_SEC:.3f} rad), "
            f"leg trend=({leg_forward:.3f} m, {leg_lateral:.3f} m, {leg_wz * RUN_DURATION_SEC:.3f} rad), "
            f"sim cooldown=({sim_cooldown_dx:.3f} m, {sim_cooldown_dy:.3f} m, {sim_cooldown_dyaw:.3f} rad), "
            f"leg cooldown=({leg_cooldown_dx:.3f} m, {leg_cooldown_dy:.3f} m, {leg_cooldown_dyaw:.3f} rad), "
            f"forward relative error={forward_relative_error:.4f}, "
            f"heading trend error={heading_error:.4f} rad, "
            f"cooldown x error={cooldown_x_error:.4f} m, "
            f"cooldown y error={cooldown_y_error:.4f} m, "
            f"cooldown pose error={cooldown_position_error:.4f} m, "
            f"cooldown heading error={cooldown_heading_error:.4f} rad."
        )
        failures = []

        if sim_forward < MIN_SIM_FORWARD_M:
            failures.append(
                f"simulator forward trend was too small after warmup; distance={sim_forward:.3f} m"
            )

        if abs(sim_lateral) > MAX_SIM_LATERAL_M:
            failures.append(
                f"simulator lateral drift trend was too large; distance={sim_lateral:.3f} m"
            )

        if abs(leg_lateral) > MAX_LEG_LATERAL_M:
            failures.append(
                f"leg-odometry lateral drift trend was too large; distance={leg_lateral:.3f} m"
            )

        if forward_relative_error > MAX_FORWARD_RELATIVE_ERROR:
            failures.append(
                "leg-odometry forward trend diverged from simulator trend; "
                f"relative_error={forward_relative_error:.4f}, "
                f"sim_vx={sim_vx:.4f}, leg_vx={leg_vx:.4f}"
            )

        if heading_error > MAX_HEADING_ERROR_RAD:
            failures.append(
                "leg-odometry heading trend diverged from simulator trend; "
                f"heading_error={heading_error:.4f} rad over {RUN_DURATION_SEC:.1f}s, "
                f"sim_wz={sim_wz:.5f}, leg_wz={leg_wz:.5f}"
            )

        if cooldown_position_error > MAX_COOLDOWN_POSITION_ERROR_M:
            failures.append(
                "leg-odometry cooled-down pose diverged from simulator pose; "
                f"position_error={cooldown_position_error:.4f} m, "
                f"x_error={cooldown_x_error:.4f} m, "
                f"y_error={cooldown_y_error:.4f} m"
            )

        if cooldown_heading_error > MAX_COOLDOWN_HEADING_ERROR_RAD:
            failures.append(
                "leg-odometry cooled-down heading diverged from simulator heading; "
                f"heading_error={cooldown_heading_error:.4f} rad, "
                f"sim={sim_cooldown_dyaw:.4f}, leg={leg_cooldown_dyaw:.4f}"
            )

        if failures:
            fail("; ".join(failures) + ". Full result: " + result_summary, logs)

        print("PASS: headless simulation ran " + result_summary)

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
