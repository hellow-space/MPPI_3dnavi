#!/usr/bin/env python3
"""Exercise the actual guard and launch remaps, using isolated test topics only."""

import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

import rclpy
from geometry_msgs.msg import Twist
from launch import LaunchContext
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from lifecycle_msgs.srv import ChangeState
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter


ROOT = Path(__file__).resolve().parents[1]
PREFIX = f"/velocity_guard_test_{os.getpid()}"


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


guard_module = load_module("velocity_guard_under_test", ROOT / "src/keep_zero.py")


def launch_remaps(filename):
    module = load_module(filename, ROOT / "src/fishbot_navigation2/launch" / filename)
    module.get_package_share_directory = lambda name: str(
        ROOT / "src" / name if name in ("fishbot_navigation2", "custom_path_navigation")
        else Path("/opt/ros/humble/share") / name)
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, GroupAction):
            for action in entity.get_sub_entities():
                if isinstance(action, SetRemap):
                    action.execute(context)
    return context.launch_configurations["ros_remaps"]


class VelocityOutputGuardTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Absolute topic remaps prevent test commands from reaching the robot,
        # even when this test is invoked without an isolated ROS_DOMAIN_ID.
        rclpy.init(args=["--ros-args", "-r", f"/cmd_vel:={PREFIX}/final",
                         "-r", f"/command/cmd_twist:={PREFIX}/raw",
                         "-r", f"/command/cmd_twist_smoothed:={PREFIX}/smoothed"])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.guard = guard_module.KeepAliveZero()
        self.peer = rclpy.create_node("velocity_guard_test_peer")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.guard)
        self.executor.add_node(self.peer)
        self.received = []
        self.reader = self.peer.create_subscription(
            Twist, PREFIX + "/final",
            lambda msg: self.received.append((time.monotonic(), msg)), 10)
        self.smooth = self.peer.create_publisher(Twist, PREFIX + "/smoothed", 1)
        self.raw = self.peer.create_publisher(Twist, PREFIX + "/raw", 1)
        self.spin_until(lambda: self.guard.pub.get_subscription_count() > 0)

    def tearDown(self):
        self.executor.shutdown()
        self.guard.destroy_node()
        self.peer.destroy_node()

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.005)

    def spin_until(self, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        self.assertTrue(predicate(), "ROS discovery or message delivery timed out")

    def test_only_smoothed_commands_reach_final_output_and_resume_after_idle(self):
        self.spin_for(0.4)
        self.assertTrue(self.received)
        self.assertTrue(all(msg == Twist() for _, msg in self.received))
        raw = Twist()
        raw.linear.x = 0.8
        self.raw.publish(raw)
        self.spin_for(0.1)
        self.assertFalse(any(msg.linear.x == 0.8 for _, msg in self.received))
        self.spin_until(lambda: self.smooth.get_subscription_count() > 0)
        smooth = Twist()
        smooth.linear.x = 0.12
        smooth.angular.z = -0.06
        self.received.clear()
        self.smooth.publish(smooth)
        self.spin_until(lambda: any(msg == smooth for _, msg in self.received))
        self.received.clear()
        self.spin_for(0.15)
        self.assertEqual(self.received, [], "guard must not insert zeros during valid input")
        self.spin_for(0.4)
        self.assertGreaterEqual(len(self.received), 3)
        self.assertTrue(all(msg == Twist() for _, msg in self.received))

    def test_frozen_ros_clock_does_not_stop_timeout_or_zero_timer(self):
        self.guard.set_parameters([Parameter("use_sim_time", value=True)])
        self.assertEqual(self.guard.get_clock().now().nanoseconds, 0)
        command = Twist()
        command.linear.x = 0.2
        self.guard.cmd_callback(command)
        self.spin_until(lambda: any(msg == command for _, msg in self.received))
        self.received.clear()
        self.spin_for(0.6)
        self.assertGreaterEqual(len(self.received), 3)
        self.assertTrue(all(msg == Twist() for _, msg in self.received))

    def test_nonfinite_components_neither_forward_nor_renew_watchdog(self):
        command = Twist()
        command.linear.x = 0.2
        self.guard.cmd_callback(command)
        self.spin_until(lambda: any(msg == command for _, msg in self.received))
        self.received.clear()
        self.spin_for(0.2)
        for group in ("linear", "angular"):
            for axis in ("x", "y", "z"):
                for value in (math.nan, math.inf, -math.inf):
                    invalid = Twist()
                    setattr(getattr(invalid, group), axis, value)
                    self.guard.cmd_callback(invalid)
        self.spin_for(0.23)
        self.assertTrue(self.received, "invalid inputs must not extend the timeout")
        self.assertTrue(all(msg == Twist() for _, msg in self.received))

    def test_both_launches_route_smoother_output_before_nav2_default(self):
        for filename in ("navigation2.launch.py", "navigation_draw_path.launch.py"):
            with self.subTest(launch=filename):
                remaps = launch_remaps(filename)
                # These are the native topic names consumed by Nav2's smoother.
                # User launch remaps precede Nav2's defaults in both Node and
                # LoadComposableNodes; rcl resolves the first matching rule.
                remaps += [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]
                arguments = ["--ros-args"]
                for src, dst in remaps:
                    arguments += ["-r", f"{src}:={dst}"]
                probe = rclpy.create_node("velocity_remap_probe", cli_args=arguments,
                                          use_global_arguments=False)
                try:
                    output = probe.create_publisher(Twist, "cmd_vel_smoothed", 1)
                    input_sub = probe.create_subscription(Twist, "cmd_vel", lambda msg: None, 1)
                    self.assertEqual(output.topic_name, "/command/cmd_twist_smoothed")
                    self.assertEqual(input_sub.topic_name, "/command/cmd_twist")
                finally:
                    probe.destroy_node()

    def test_real_smoother_timeout_and_process_exit_leave_only_zero_output(self):
        executable = Path("/opt/ros/humble/lib/nav2_velocity_smoother/velocity_smoother")
        if not executable.is_file():
            self.skipTest("Nav2 velocity smoother binary is unavailable")
        name = "isolated_guard_test_smoother"
        args = [str(executable), "--ros-args", "-r", f"__node:={name}",
                "-r", f"cmd_vel:={PREFIX}/raw",
                "-r", f"cmd_vel_smoothed:={PREFIX}/smoothed",
                "-p", "velocity_timeout:=0.3", "-p", "smoothing_frequency:=20.0",
                "-p", "max_accel:=[1.2, 0.0, 1.2]",
                "-p", "max_decel:=[-1.2, 0.0, -1.2]"]
        with open(os.devnull, "w") as log:
            process = subprocess.Popen(args, stdout=log, stderr=log)
            try:
                client = self.peer.create_client(ChangeState, f"/{name}/change_state")
                self.assertTrue(client.wait_for_service(timeout_sec=5))
                for transition_id in (1, 3):
                    request = ChangeState.Request()
                    request.transition.id = transition_id
                    future = client.call_async(request)
                    self.spin_until(future.done)
                    self.assertTrue(future.result().success)
                self.spin_until(lambda: self.smooth.get_subscription_count() > 0)
                command = Twist()
                command.linear.x = 0.4
                for _ in range(15):
                    self.raw.publish(command)
                    self.spin_for(0.05)
                self.assertTrue(any(msg.linear.x > 0 for _, msg in self.received))
                stopped_input = time.monotonic()
                self.spin_for(1.5)
                late = [msg for stamp, msg in self.received if stamp > stopped_input + 1]
                self.assertGreaterEqual(len(late), 3)
                self.assertTrue(all(msg == Twist() for msg in late))
                for _ in range(12):
                    self.raw.publish(command)
                    self.spin_for(0.05)
                self.assertGreater(self.received[-1][1].linear.x, 0)
                process.send_signal(signal.SIGINT)
                exited = time.monotonic()
                self.spin_for(0.9)
                late = [msg for stamp, msg in self.received if stamp > exited + 0.4]
                self.assertGreaterEqual(len(late), 3)
                self.assertTrue(all(msg == Twist() for msg in late))
                publishers = self.peer.get_publishers_info_by_topic(PREFIX + "/final")
                self.assertEqual([pub.node_name for pub in publishers], ["keep_alive_zero"])
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


if __name__ == "__main__":
    unittest.main()
