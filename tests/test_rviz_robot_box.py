#!/usr/bin/env python3

import importlib.util
import os
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROBOT_URDF = (
    PROJECT_ROOT
    / "src"
    / "fishbot_navigation2"
    / "urdf"
    / "navigation_footprint.urdf"
)
REPLAY_RVIZ = PROJECT_ROOT / "nav_experiment_toolkit" / "config" / "nav_replay.rviz"
NAVIGATION_RVIZ = (
    PROJECT_ROOT
    / "src"
    / "fishbot_navigation2"
    / "rviz"
    / "navigation.rviz"
)
NAVIGATION_LAUNCH = (
    PROJECT_ROOT
    / "src"
    / "fishbot_navigation2"
    / "launch"
    / "navigation2.launch.py"
)
REPLAY_SCRIPT = (
    PROJECT_ROOT / "nav_experiment_toolkit" / "scripts" / "replay_nav_rviz.sh"
)
ROBOT_DESCRIPTION_TOPIC = "/navigation_footprint/robot_description"


def robot_box_display(config_path):
    with config_path.open(encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    displays = config["Visualization Manager"]["Displays"]
    return next(display for display in displays if display["Name"] == "Robot Footprint Box")


def run_fake_replay(*, rviz_exits_early=False, publisher_exits_early=False):
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_path = Path(temp_dir)
        bag_path = temp_path / "bag"
        bin_path = temp_path / "bin"
        rsp_prefix = temp_path / "rsp_prefix"
        call_log = temp_path / "calls.log"
        bag_path.mkdir()
        bin_path.mkdir()
        rsp_executable = (
            rsp_prefix
            / "lib"
            / "robot_state_publisher"
            / "robot_state_publisher"
        )
        rsp_executable.parent.mkdir(parents=True)
        (bag_path / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")

        rviz = bin_path / "rviz2"
        rviz.write_text(
            "#!/usr/bin/env bash\n"
            "printf -v call 'rviz2'\n"
            "for arg in \"$@\"; do printf -v call '%s %q' \"$call\" \"$arg\"; done\n"
            "printf '%s\\n' \"$call\" >> \"$FAKE_CALL_LOG\"\n"
            "if [[ \"$FAKE_RVIZ_EARLY_EXIT\" == true ]]; then exit 7; fi\n"
            "exec /bin/sleep 30\n",
            encoding="utf-8",
        )
        ros2 = bin_path / "ros2"
        ros2.write_text(
            "#!/usr/bin/env bash\n"
            "printf -v call 'ros2'\n"
            "for arg in \"$@\"; do printf -v call '%s %q' \"$call\" \"$arg\"; done\n"
            "printf '%s\\n' \"$call\" >> \"$FAKE_CALL_LOG\"\n"
            "if [[ \"${1:-}\" == pkg && \"${2:-}\" == prefix ]]; then\n"
            "  printf '%s\\n' \"$FAKE_RSP_PREFIX\"\n"
            "fi\n",
            encoding="utf-8",
        )
        rsp_executable.write_text(
            "#!/usr/bin/env bash\n"
            "printf -v call 'robot_state_publisher'\n"
            "for arg in \"$@\"; do printf -v call '%s %q' \"$call\" \"$arg\"; done\n"
            "printf '%s\\n' \"$call\" >> \"$FAKE_CALL_LOG\"\n"
            "if [[ \"$FAKE_RSP_EARLY_EXIT\" == true ]]; then exit 8; fi\n"
            "exec /bin/sleep 30\n",
            encoding="utf-8",
        )
        rviz.chmod(0o755)
        ros2.chmod(0o755)
        rsp_executable.chmod(0o755)

        env = os.environ.copy()
        env["PATH"] = f"{bin_path}:{env['PATH']}"
        env["FAKE_CALL_LOG"] = str(call_log)
        env["FAKE_RSP_PREFIX"] = str(rsp_prefix)
        env["FAKE_RVIZ_EARLY_EXIT"] = str(rviz_exits_early).lower()
        env["FAKE_RSP_EARLY_EXIT"] = str(publisher_exits_early).lower()
        result = subprocess.run(
            [str(REPLAY_SCRIPT), str(bag_path)],
            check=False,
            env=env,
            capture_output=True,
            text=True,
        )
        return result, call_log.read_text(encoding="utf-8")


def launch_substitution_text(parts):
    return "".join(part.perform(None) for part in parts)


class RvizRobotBoxTest(unittest.TestCase):
    def test_robot_box_matches_the_nav2_footprint(self):
        root = ET.parse(ROBOT_URDF).getroot()
        box = root.find("./link[@name='base_link']/visual/geometry/box")

        self.assertIsNotNone(box)
        self.assertEqual(box.attrib["size"], "1.2 0.9 0.05")

    def test_replay_rviz_starts_with_the_robot_box_enabled(self):
        display = robot_box_display(REPLAY_RVIZ)

        self.assertEqual(display["Class"], "rviz_default_plugins/RobotModel")
        self.assertEqual(display["Description Source"], "Topic")
        self.assertEqual(
            display["Description Topic"]["Value"],
            ROBOT_DESCRIPTION_TOPIC,
        )
        self.assertIs(display["Enabled"], True)
        self.assertIs(display["Value"], True)

    def test_navigation_rviz_starts_with_the_robot_box_enabled(self):
        display = robot_box_display(NAVIGATION_RVIZ)

        self.assertEqual(display["Class"], "rviz_default_plugins/RobotModel")
        self.assertEqual(
            display["Description Topic"]["Value"],
            ROBOT_DESCRIPTION_TOPIC,
        )
        self.assertIs(display["Enabled"], True)

    def test_navigation_launch_provides_the_box_model_and_project_rviz(self):
        with tempfile.TemporaryDirectory() as log_dir:
            previous_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = log_dir
            try:
                spec = importlib.util.spec_from_file_location(
                    "navigation2_launch_test",
                    NAVIGATION_LAUNCH,
                )
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                package_root = NAVIGATION_LAUNCH.parents[1]
                module.get_package_share_directory = lambda package: str(
                    package_root
                    if package == "fishbot_navigation2"
                    else Path("/opt/ros/humble/share") / package
                )
                entities = module.generate_launch_description().entities
            finally:
                if previous_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_log_dir

        nodes = {
            entity.node_package: entity
            for entity in entities
            if hasattr(entity, "node_package")
        }
        publisher = nodes["robot_state_publisher"]
        rviz = nodes["rviz2"]
        remappings = {
            launch_substitution_text(source): launch_substitution_text(destination)
            for source, destination in publisher._Node__remappings
        }

        self.assertEqual(
            remappings["robot_description"],
            ROBOT_DESCRIPTION_TOPIC,
        )
        self.assertEqual(
            rviz._Node__arguments,
            ["-d", str(NAVIGATION_RVIZ)],
        )

    def test_existing_replay_command_starts_the_robot_description_publisher(self):
        result, calls = run_fake_replay()

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("ros2 pkg prefix robot_state_publisher", calls)
        self.assertIn("robot_state_publisher --ros-args", calls)
        self.assertIn(
            "robot_description:=/navigation_footprint/robot_description",
            calls,
        )
        self.assertIn("use_sim_time:=true", calls)
        self.assertIn("navigation_footprint", calls)
        self.assertIn("ros2 bag play", calls)
        self.assertIn("--clock", calls)
        self.assertIn("rviz2 -d", calls)

    def test_replay_does_not_start_the_bag_when_rviz_startup_fails(self):
        result, calls = run_fake_replay(rviz_exits_early=True)

        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("ros2 bag play", calls)
        self.assertIn("RViz exited during startup", result.stderr)

    def test_replay_does_not_start_the_bag_when_robot_publisher_fails(self):
        result, calls = run_fake_replay(publisher_exits_early=True)

        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("ros2 bag play", calls)
        self.assertIn("robot_state_publisher exited during startup", result.stderr)


if __name__ == "__main__":
    unittest.main()
