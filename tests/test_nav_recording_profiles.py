#!/usr/bin/env python3

import os
import subprocess
import tempfile
import unittest
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
TOOLKIT_ROOT = PROJECT_ROOT / "nav_experiment_toolkit"
BASELINE_RECORD = TOOLKIT_ROOT / "scripts" / "record_nav_baseline.sh"
BASELINE_CHECK = TOOLKIT_ROOT / "scripts" / "check_before_record.sh"
ELEVATION_RECORD = TOOLKIT_ROOT / "scripts" / "record_nav_elevation.sh"
ELEVATION_REPLAY = TOOLKIT_ROOT / "scripts" / "replay_nav_elevation_rviz.sh"
BASELINE_RVIZ = TOOLKIT_ROOT / "config" / "nav_replay.rviz"
ELEVATION_RVIZ = TOOLKIT_ROOT / "config" / "nav_elevation_replay.rviz"
ELEVATION_TOPIC = "/elevation_mapping_node/elevation_map"
LOCAL_COSTMAP = "/local_costmap/costmap"
GLOBAL_COSTMAP = "/global_costmap/costmap"
ELEVATION_REQUIRED = {
    ELEVATION_TOPIC, "/cloud_registered",
    "/height_obstacle_layer/height_difference_map",
    "/tf", "/tf_static", "/aft_mapped_to_init", "/map",
    LOCAL_COSTMAP, GLOBAL_COSTMAP,
}

BASE_NAV_TOPICS = {
    "/map",
    "/tf",
    "/tf_static",
    "/aft_mapped_to_init",
    "/path",
    "/amcl_pose",
    "/command/cmd_twist",
    "/cmd_vel",
    "/goal_pose",
    "/plan",
    "/transformed_global_plan",
    "/trajectories",
}


def run_record_script(script, published_topics):
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_path = Path(temp_dir)
        bin_path = temp_path / "bin"
        call_log = temp_path / "record-args.log"
        bin_path.mkdir()

        ros2 = bin_path / "ros2"
        ros2.write_text(
            "#!/usr/bin/env bash\n"
            "set -euo pipefail\n"
            "if [[ \"${1:-}\" == topic && \"${2:-}\" == list ]]; then\n"
            "  printf '%s\\n' \"$FAKE_PUBLISHED_TOPICS\"\n"
            "elif [[ \"${1:-}\" == bag && \"${2:-}\" == record ]]; then\n"
            "  shift 2\n"
            "  printf '%s\\n' \"$@\" > \"$FAKE_CALL_LOG\"\n"
            "else\n"
            "  exit 2\n"
            "fi\n",
            encoding="utf-8",
        )
        ros2.chmod(0o755)

        env = os.environ.copy()
        env["PATH"] = f"{bin_path}:{env['PATH']}"
        env["FAKE_CALL_LOG"] = str(call_log)
        env["FAKE_PUBLISHED_TOPICS"] = "\n".join(published_topics)
        result = subprocess.run(
            [str(script)],
            check=False,
            env=env,
            capture_output=True,
            text=True,
        )
        args = call_log.read_text(encoding="utf-8").splitlines() if call_log.exists() else []
        return result, args


def run_baseline_check(published_topics, profile="baseline"):
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_path = Path(temp_dir)
        bin_path = temp_path / "bin"
        bin_path.mkdir()
        ros2 = bin_path / "ros2"
        ros2.write_text(
            "#!/usr/bin/env bash\n"
            "if [[ \"${1:-}\" == topic && \"${2:-}\" == list ]]; then\n"
            "  printf '%s\\n' \"$FAKE_PUBLISHED_TOPICS\"\n"
            "  exit 0\n"
            "fi\n"
            "exit 2\n",
            encoding="utf-8",
        )
        ros2.chmod(0o755)
        env = os.environ.copy()
        env["PATH"] = f"{bin_path}:{env['PATH']}"
        env["FAKE_PUBLISHED_TOPICS"] = "\n".join(published_topics)
        return subprocess.run(
            [str(BASELINE_CHECK), profile],
            check=False,
            env=env,
            capture_output=True,
            text=True,
        )


def load_rviz(path):
    with path.open(encoding="utf-8") as config_file:
        return yaml.safe_load(config_file)


def display_topics(config):
    topics = {}
    for display in config["Visualization Manager"]["Displays"]:
        topic = display.get("Topic")
        if isinstance(topic, dict) and "Value" in topic:
            topics[topic["Value"]] = display
    return topics


def run_elevation_replay():
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_path = Path(temp_dir)
        bag_path = temp_path / "bag"
        bin_path = temp_path / "bin"
        rsp_prefix = temp_path / "rsp-prefix"
        rviz_log = temp_path / "rviz.log"
        rsp_log = temp_path / "rsp.log"
        ros2_log = temp_path / "ros2.log"
        bag_path.mkdir()
        bin_path.mkdir()
        (bag_path / "metadata.yaml").write_text(
            "rosbag2_bagfile_information: {}\n", encoding="utf-8"
        )

        rsp = rsp_prefix / "lib" / "robot_state_publisher" / "robot_state_publisher"
        rsp.parent.mkdir(parents=True)
        rsp.write_text(
            "#!/usr/bin/env bash\n"
            "printf -v call 'robot_state_publisher'\n"
            "for arg in \"$@\"; do printf -v call '%s %q' \"$call\" \"$arg\"; done\n"
            "printf '%s\\n' \"$call\" > \"$FAKE_RSP_LOG\"\n"
            "exec /bin/sleep 30\n",
            encoding="utf-8",
        )

        (bin_path / "rviz2").write_text(
            "#!/usr/bin/env bash\n"
            "printf -v call 'rviz2'\n"
            "for arg in \"$@\"; do printf -v call '%s %q' \"$call\" \"$arg\"; done\n"
            "printf '%s\\n' \"$call\" > \"$FAKE_RVIZ_LOG\"\n"
            "exec /bin/sleep 30\n",
            encoding="utf-8",
        )
        (bin_path / "ros2").write_text(
            "#!/usr/bin/env bash\n"
            "if [[ \"${1:-}\" == pkg && \"${2:-}\" == prefix ]]; then\n"
            "  printf '%s\\n' \"$FAKE_RSP_PREFIX\"\n"
            "  exit 0\n"
            "fi\n"
            "printf -v call 'ros2'\n"
            "for arg in \"$@\"; do printf -v call '%s %q' \"$call\" \"$arg\"; done\n"
            "printf '%s\\n' \"$call\" > \"$FAKE_ROS2_LOG\"\n",
            encoding="utf-8",
        )
        (bin_path / "sleep").write_text(
            "#!/usr/bin/env bash\n"
            "for unused in {1..100}; do\n"
            "  if [[ -s \"$FAKE_RVIZ_LOG\" && -s \"$FAKE_RSP_LOG\" ]]; then exit 0; fi\n"
            "  /bin/sleep 0.01\n"
            "done\n"
            "exit 1\n",
            encoding="utf-8",
        )
        for executable in (rsp, bin_path / "rviz2", bin_path / "ros2", bin_path / "sleep"):
            executable.chmod(0o755)

        env = os.environ.copy()
        env["PATH"] = f"{bin_path}:{env['PATH']}"
        env["FAKE_RSP_PREFIX"] = str(rsp_prefix)
        env["FAKE_RVIZ_LOG"] = str(rviz_log)
        env["FAKE_RSP_LOG"] = str(rsp_log)
        env["FAKE_ROS2_LOG"] = str(ros2_log)
        result = subprocess.run(
            [str(ELEVATION_REPLAY), str(bag_path)],
            check=False,
            env=env,
            capture_output=True,
            text=True,
        )
        calls = "".join(
            log.read_text(encoding="utf-8")
            for log in (rviz_log, rsp_log, ros2_log)
            if log.exists()
        )
        return result, calls


class NavigationRecordingProfilesTest(unittest.TestCase):
    def test_2d_recording_captures_complete_local_and_global_costmaps(self):
        result, args = run_record_script(BASELINE_RECORD, [])

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(LOCAL_COSTMAP, args)
        self.assertIn(GLOBAL_COSTMAP, args)
        self.assertNotIn("/local_costmap/costmap_updates", args)
        self.assertNotIn("/global_costmap/costmap_updates", args)

    def test_2d_rviz_starts_with_both_costmaps_visible(self):
        topics = display_topics(load_rviz(BASELINE_RVIZ))

        self.assertIs(topics[LOCAL_COSTMAP]["Enabled"], True)
        self.assertIs(topics[GLOBAL_COSTMAP]["Enabled"], True)

    def test_2d_precheck_requires_both_complete_costmaps(self):
        without_costmaps = run_baseline_check(BASE_NAV_TOPICS)
        with_costmaps = run_baseline_check(
            BASE_NAV_TOPICS | {LOCAL_COSTMAP, GLOBAL_COSTMAP}
        )

        self.assertNotEqual(without_costmaps.returncode, 0)
        self.assertIn(f"[MISSING] {LOCAL_COSTMAP}", without_costmaps.stdout)
        self.assertIn(f"[MISSING] {GLOBAL_COSTMAP}", without_costmaps.stdout)
        self.assertEqual(with_costmaps.returncode, 0, with_costmaps.stdout)

    def test_elevation_records_core_chain_and_waits_for_later_task_topics(self):
        result, args = run_record_script(ELEVATION_RECORD, ELEVATION_REQUIRED)
        self.assertEqual(result.returncode, 0, result.stderr)
        recorded_topics = set(args[args.index("--output") + 2 :])
        self.assertTrue(ELEVATION_REQUIRED <= recorded_topics)
        self.assertIn("--include-hidden-topics", args)
        for topic in ("/navigate_to_pose/_action/status", "/follow_path/_action/status",
                      "/custom_path/raw_path", "/custom_path/preview_path",
                      "/custom_path/status", "/command/cmd_twist", "/cmd_vel",
                      "/goal_pose", "/plan", "/transformed_global_plan"):
            self.assertIn(topic, recorded_topics)
        for topic in ("/trajectories", "/path", "/amcl_pose"):
            self.assertNotIn(topic, recorded_topics)

    def test_elevation_required_inputs_are_shared_by_precheck_and_recorder(self):
        result = run_baseline_check(ELEVATION_REQUIRED, "elevation")
        self.assertEqual(result.returncode, 0, result.stdout)
        for missing in ELEVATION_REQUIRED:
            with self.subTest(missing=missing):
                published = ELEVATION_REQUIRED - {missing}
                result, args = run_record_script(ELEVATION_RECORD, published)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(args, [])
                self.assertIn(missing, result.stderr)
                check = run_baseline_check(published, "elevation")
                self.assertNotEqual(check.returncode, 0)
                self.assertIn(f"[MISSING] {missing}", check.stdout)

    def test_elevation_recording_refuses_to_start_without_the_elevation_topic(self):
        result, args = run_record_script(ELEVATION_RECORD, [])

        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(args, [])
        self.assertIn(ELEVATION_TOPIC, result.stderr)

    def test_elevation_rviz_emphasizes_grid_map_without_costmaps(self):
        config = load_rviz(ELEVATION_RVIZ)
        topics = display_topics(config)
        elevation = topics[ELEVATION_TOPIC]
        display_names = {
            display["Name"]
            for display in config["Visualization Manager"]["Displays"]
        }

        self.assertEqual(elevation["Class"], "grid_map_rviz_plugin/GridMap")
        self.assertEqual(elevation["Height Layer"], "elevation")
        self.assertEqual(elevation["Color Layer"], "elevation")
        self.assertIs(elevation["Enabled"], True)
        self.assertNotIn(LOCAL_COSTMAP, topics)
        self.assertNotIn(GLOBAL_COSTMAP, topics)
        self.assertIn("Robot Footprint Box", display_names)
        self.assertEqual(
            config["Visualization Manager"]["Views"]["Current"]["Class"],
            "rviz_default_plugins/Orbit",
        )

    def test_elevation_replay_uses_only_the_elevation_rviz_template(self):
        result, calls = run_elevation_replay()

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f"rviz2 -d {ELEVATION_RVIZ}", calls)
        self.assertIn("ros2 bag play", calls)
        self.assertIn("--clock", calls)
        self.assertIn("robot_state_publisher --ros-args", calls)


if __name__ == "__main__":
    unittest.main()
