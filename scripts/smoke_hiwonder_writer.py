#!/usr/bin/env python3

import fcntl
import os
import pty
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from actuator_msgs.msg import Actuators
from rclpy.node import Node
from std_srvs.srv import Trigger


LOG_DIR = Path(os.environ.get("ANTSY_TEST_LOG_DIR", "/tmp/antsy_smoke_tests"))
SERVO_COUNT = 18
MOVE_COMMAND = 0x03
UNLOAD_COMMAND = 0x14


def start_writer(device):
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / "hiwonder_writer.log"
    log_file = log_path.open("w")
    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "hiwonder_ros2",
            "write_only",
            "--ros-args",
            "-p",
            f"device:={device}",
            "-p",
            "motor_read_rate:=20",
        ],
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


def fail(message, log_path):
    print(f"FAIL: {message}", file=sys.stderr)
    if log_path.exists():
        print(f"\n--- hiwonder writer log: {log_path} ---", file=sys.stderr)
        print("".join(log_path.read_text(errors="replace").splitlines(True)[-80:]), file=sys.stderr)
    raise SystemExit(1)


def set_nonblocking(fd):
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)


def read_available(fd):
    chunks = []
    while True:
        try:
            chunks.append(os.read(fd, 4096))
        except BlockingIOError:
            break
    return b"".join(chunks)


def has_command_packet(data, command):
    return any(
        data[i] == 0x55 and data[i + 1] == 0x55 and data[i + 3] == command
        for i in range(0, max(0, len(data) - 3))
    )


def wait_for_command_packet(fd, command, timeout):
    deadline = time.monotonic() + timeout
    data = b""
    while time.monotonic() < deadline:
        data += read_available(fd)
        if has_command_packet(data, command):
            return data
        time.sleep(0.02)
    raise TimeoutError(f"timed out waiting for serial command 0x{command:02x}")


class HiwonderSmokeNode(Node):
    def __init__(self):
        super().__init__("antsy_hiwonder_writer_smoke")
        self.actuator_pub = self.create_publisher(Actuators, "/actuators", 10)
        self.disable_client = self.create_client(Trigger, "/motors/disable")
        self.enable_client = self.create_client(Trigger, "/motors/enable")

    def publish_actuators(self, offset):
        msg = Actuators()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [offset + 0.01 * i for i in range(SERVO_COUNT)]
        self.actuator_pub.publish(msg)

    def wait_for_services(self):
        if not self.disable_client.wait_for_service(timeout_sec=8.0):
            raise TimeoutError("/motors/disable service did not appear")
        if not self.enable_client.wait_for_service(timeout_sec=8.0):
            raise TimeoutError("/motors/enable service did not appear")

    def call_trigger(self, client, name):
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                response = future.result()
                if not response.success:
                    raise RuntimeError(f"{name} failed: {response.message}")
                return response.message
        raise TimeoutError(f"{name} timed out")


def main():
    master_fd, slave_fd = pty.openpty()
    set_nonblocking(master_fd)
    device = os.ttyname(slave_fd)
    writer = start_writer(device)
    process, log_file, log_path = writer

    rclpy.init()
    node = None
    try:
        node = HiwonderSmokeNode()
        node.wait_for_services()

        for _ in range(8):
            node.publish_actuators(offset=0.0)
            rclpy.spin_once(node, timeout_sec=0.03)
            time.sleep(0.05)
        wait_for_command_packet(master_fd, MOVE_COMMAND, timeout=4.0)

        node.call_trigger(node.disable_client, "/motors/disable")
        wait_for_command_packet(master_fd, UNLOAD_COMMAND, timeout=4.0)
        time.sleep(0.2)
        read_available(master_fd)

        for _ in range(8):
            node.publish_actuators(offset=0.5)
            rclpy.spin_once(node, timeout_sec=0.03)
            time.sleep(0.05)
        disabled_data = read_available(master_fd)
        if has_command_packet(disabled_data, MOVE_COMMAND):
            fail("writer sent move commands while motors were disabled", log_path)

        node.call_trigger(node.enable_client, "/motors/enable")
        wait_for_command_packet(master_fd, MOVE_COMMAND, timeout=4.0)
        print("PASS: Hiwonder writer sent move packets, unloaded servos, paused writes, and resumed on enable.")
    except Exception as exc:
        fail(str(exc), log_path)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        stop_process(process, log_file)
        os.close(master_fd)
        os.close(slave_fd)


if __name__ == "__main__":
    main()
