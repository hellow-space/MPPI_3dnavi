#!/usr/bin/env python3

import unittest
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = (
    PROJECT_ROOT
    / "src"
    / "fishbot_navigation2"
    / "config"
    / "nav2_params_height_jump.yaml"
)


class HeightJumpNavigationConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with CONFIG_PATH.open(encoding="utf-8") as config_file:
            cls.config = yaml.safe_load(config_file)

    def test_local_costmap_has_only_an_inert_inflation_layer(self):
        params = self.config["local_costmap"]["local_costmap"]["ros__parameters"]

        self.assertEqual(params["plugins"], ["inflation_layer"])
        self.assertEqual(
            params["inflation_layer"]["plugin"],
            "nav2_costmap_2d::InflationLayer",
        )
        self.assertFalse(
            {"obstacle_layer", "voxel_layer", "static_layer"} & params.keys()
        )

    def test_height_jump_is_the_only_mppi_terrain_feasibility_critic(self):
        controller = self.config["controller_server"]["ros__parameters"]["FollowPath"]
        height_jump = controller["HeightJumpCritic"]

        self.assertIn("HeightJumpCritic", controller["critics"])
        self.assertNotIn("ObstaclesCritic", controller["critics"])
        self.assertNotIn("CostCritic", controller["critics"])
        self.assertNotIn("ObstaclesCritic", controller)
        self.assertNotIn("CostCritic", controller)
        self.assertIs(height_jump.get("unknown_is_traversable", False), True)
        self.assertIs(height_jump["startup_unknown_enabled"], True)
        self.assertEqual(height_jump["unknown_ignore_radius"], 1.0)

    def test_global_costmap_uses_only_the_static_map(self):
        params = self.config["global_costmap"]["global_costmap"]["ros__parameters"]

        self.assertEqual(params["plugins"], ["static_layer", "inflation_layer"])
        self.assertNotIn("obstacle_layer", params)
        self.assertNotIn("voxel_layer", params)

    def test_recovery_behaviors_support_the_default_behavior_tree(self):
        params = self.config["behavior_server"]["ros__parameters"]

        self.assertEqual(
            params["behavior_plugins"],
            ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"],
        )
        self.assertEqual(params["spin"]["plugin"], "nav2_behaviors/Spin")
        self.assertEqual(params["backup"]["plugin"], "nav2_behaviors/BackUp")
        self.assertEqual(
            params["drive_on_heading"]["plugin"],
            "nav2_behaviors/DriveOnHeading",
        )
        self.assertEqual(
            params["assisted_teleop"]["plugin"],
            "nav2_behaviors/AssistedTeleop",
        )
        self.assertEqual(params["wait"]["plugin"], "nav2_behaviors/Wait")


if __name__ == "__main__":
    unittest.main()
