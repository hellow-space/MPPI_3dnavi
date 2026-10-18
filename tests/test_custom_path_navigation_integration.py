#!/usr/bin/env python3
"""Check the installed-launch contract without launching Nav2 or RViz."""

import importlib.util
import os
import tempfile
import unittest
from pathlib import Path

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node, SetRemap
from launch_ros.utilities import evaluate_parameters


ROOT = Path(__file__).resolve().parents[1]
NAV = ROOT / "src/fishbot_navigation2"
CUSTOM = ROOT / "src/custom_path_navigation"
NEW_LAUNCH = NAV / "launch/navigation_draw_path.launch.py"
NEW_RVIZ = NAV / "rviz/navigation_draw_path.rviz"


def resolve(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


class CustomPathIntegrationTest(unittest.TestCase):
    def test_package_has_no_elevation_dependencies(self):
        for filename in ("package.xml", "CMakeLists.txt"):
            text = (CUSTOM / filename).read_text(encoding="utf-8")
            for forbidden in ("grid_map", "height_obstacle", "height_jump"):
                self.assertNotIn(forbidden, text, filename)

    def test_executor_config_matches_approved_defaults(self):
        params = yaml.safe_load((CUSTOM / "config/custom_path.yaml").read_text())
        self.assertEqual(params["custom_path_executor"]["ros__parameters"], {
            "fixed_frame": "map", "robot_frame": "base_link", "start_tolerance": 0.5,
            "min_raw_spacing": 0.02, "min_path_length": 0.20, "sample_spacing": 0.05,
            "smoothing_passes": 1, "follow_path_action": "/follow_path",
            "controller_id": "FollowPath", "goal_checker_id": "precise_goal_checker",
            "action_server_timeout": 2.0, "cancel_timeout": 2.0,
        })

    def test_rviz_keeps_original_view_and_adds_preview_and_tool(self):
        self.assertTrue(NEW_RVIZ.is_file(), "new RViz template is missing")
        old = yaml.safe_load((NAV / "rviz/navigation.rviz").read_text())
        new = yaml.safe_load(NEW_RVIZ.read_text())
        original = old["Visualization Manager"]
        manager = new["Visualization Manager"]
        for key in ("Panels", "Window Geometry"):
            self.assertEqual(new[key], old[key])
        for key in ("Views", "Global Options"):
            self.assertEqual(manager[key], original[key])
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
        for display in original["Displays"]:
            self.assertIn(display, manager["Displays"], display["Name"])
        for tool in original["Tools"]:
            self.assertIn(tool, manager["Tools"])
        self.assertIn({"Class": "custom_path_navigation/DrawFollowPath"}, manager["Tools"])
        for kind, topic in (("Path", "/custom_path/preview_path"),
                            ("Marker", "/custom_path/preview_marker")):
            display = next(d for d in manager["Displays"]
                           if d.get("Topic", {}).get("Value") == topic)
            self.assertEqual(display["Class"], f"rviz_default_plugins/{kind}")
            self.assertTrue(display["Enabled"])
            self.assertTrue(display["Value"])
            self.assertEqual(display["Topic"]["Reliability Policy"], "Reliable")
            self.assertEqual(display["Topic"]["Durability Policy"], "Volatile")

    def test_launch_preserves_nav2_and_robot_and_forwards_overrides(self):
        self.assertTrue(NEW_LAUNCH.is_file(), "new navigation launch is missing")
        with tempfile.TemporaryDirectory() as log_dir:
            previous = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = log_dir
            try:
                spec = importlib.util.spec_from_file_location("draw_path_launch_test", NEW_LAUNCH)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                module.get_package_share_directory = lambda package: str({
                    "fishbot_navigation2": NAV, "custom_path_navigation": CUSTOM,
                }.get(package, Path("/opt/ros/humble/share") / package))
                entities = module.generate_launch_description().entities
            finally:
                if previous is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous

        declarations = {e.name: e for e in entities if isinstance(e, DeclareLaunchArgument)}
        self.assertEqual(set(declarations), {"map", "use_sim_time", "params_file",
                                            "custom_path_params_file"})
        defaults = LaunchContext()
        for declaration in declarations.values():
            declaration.execute(defaults)
        self.assertEqual(defaults.launch_configurations["params_file"],
                         str(NAV / "config/nav2_params_height_obstacle.yaml"))
        self.assertEqual(defaults.launch_configurations["custom_path_params_file"],
                         str(CUSTOM / "config/custom_path.yaml"))

        context = LaunchContext()
        context.launch_configurations.update({
            "map": "/tmp/custom map.yaml", "use_sim_time": "true",
            "params_file": "/tmp/alternate nav2.yaml",
            "custom_path_params_file": "/tmp/alternate custom.yaml",
        })
        nodes = [e for e in entities if isinstance(e, Node)]
        self.assertEqual(len(nodes), 3)
        by_package = {n.node_package: n for n in nodes}
        executor = by_package["custom_path_navigation"]
        self.assertEqual(resolve(context, executor.node_executable), "custom_path_executor")
        params = evaluate_parameters(context, executor._Node__parameters)
        self.assertEqual(str(params[0]), "/tmp/alternate custom.yaml")
        self.assertTrue(params[1]["use_sim_time"])
        rviz = by_package["rviz2"]
        self.assertEqual(rviz._Node__arguments, ["-d", str(NEW_RVIZ)])
        self.assertTrue(evaluate_parameters(context, rviz._Node__parameters)[0]["use_sim_time"])
        publisher = by_package["robot_state_publisher"]
        self.assertEqual(resolve(context, publisher.node_executable), "robot_state_publisher")
        robot_params = evaluate_parameters(context, publisher._Node__parameters)[0]
        self.assertEqual(robot_params["robot_description"],
                         (NAV / "urdf/navigation_footprint.urdf").read_text())
        self.assertTrue(robot_params["use_sim_time"])
        remaps = {resolve(context, src): resolve(context, dst)
                  for src, dst in publisher._Node__remappings}
        self.assertEqual(remaps["robot_description"], "/navigation_footprint/robot_description")

        groups = [e for e in entities if isinstance(e, GroupAction)]
        self.assertEqual(len(groups), 1)
        actions = groups[0].get_sub_entities()
        remap = next(e for e in actions if isinstance(e, SetRemap))
        self.assertEqual(resolve(context, remap.src), "/cmd_vel")
        self.assertEqual(resolve(context, remap.dst), "/command/cmd_twist")
        includes = [e for e in actions if isinstance(e, IncludeLaunchDescription)]
        self.assertEqual(len(includes), 1)
        include = includes[0]
        # Humble exposes a stringified substitution until the source is loaded.
        # Loading the description checks the real include without executing its nodes.
        include.launch_description_source.get_launch_description(context)
        self.assertEqual(resolve(context, include.launch_description_source.location),
                         "/opt/ros/humble/share/nav2_bringup/launch/bringup_launch.py")
        forwarded = {resolve(context, k): resolve(context, v)
                     for k, v in include.launch_arguments}
        self.assertEqual(forwarded, {"map": "/tmp/custom map.yaml", "use_sim_time": "true",
                                    "params_file": "/tmp/alternate nav2.yaml"})

    def test_existing_profiles_have_no_custom_path_references(self):
        files = [NAV / "launch/navigation2.launch.py", NAV / "rviz/navigation.rviz"]
        files.extend(NAV.glob("config/*.yaml"))
        files.extend(p for p in (ROOT / "scripts").glob("*navigation*.sh")
                     if "draw_path" not in p.name)
        for path in files:
            text = path.read_text(encoding="utf-8")
            for forbidden in ("custom_path", "DrawFollowPath", "navigation_draw_path"):
                self.assertNotIn(forbidden, text, str(path))


if __name__ == "__main__":
    unittest.main()
