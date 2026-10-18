#!/usr/bin/env python3

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src/fishbot_navigation2/config/nav2_params_height_obstacle.yaml"


def main() -> None:
    params = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))

    controller = params["controller_server"]["ros__parameters"]["FollowPath"]
    critics = controller["critics"]
    assert "CostCritic" in critics
    assert "HeightJumpCritic" in critics
    assert critics.index("CostCritic") < critics.index("HeightJumpCritic")
    assert controller["CostCritic"]["consider_footprint"] is True
    assert controller["CostCritic"]["collision_cost"] == 1_000_000.0
    assert controller["CostCritic"]["inflation_layer_name"] == "inflation_layer"
    assert controller["HeightJumpCritic"]["enabled"] is True

    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = params["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["plugins"] == ["height_obstacle_layer", "inflation_layer"]
    assert global_["plugins"] == ["static_layer", "height_obstacle_layer", "inflation_layer"]

    for costmap in (local, global_):
        layer = costmap["height_obstacle_layer"]
        assert layer["plugin"] == "height_obstacle_layer::HeightObstacleLayer"
        assert layer["map_topic"] == "/elevation_mapping_node/elevation_map"
        assert layer["elevation_layer"] == "elevation"
        assert layer["jump_radius"] == 0.10
        assert layer["max_height_jump"] == 0.30
        assert layer["map_timeout"] == 1.0
        assert layer["unknown_as_obstacle"] is False

    assert local["height_obstacle_layer"]["publish_height_difference"] is True
    assert global_["height_obstacle_layer"]["publish_height_difference"] is False

    start_script = (ROOT / "scripts/start_navigation_height_obstacle.sh").read_text(
        encoding="utf-8"
    )
    stop_script = (ROOT / "scripts/stop_navigation_height_obstacle.sh").read_text(
        encoding="utf-8"
    )
    assert "mppi_nav_height_obstacle" in start_script
    assert "nav2_params_height_obstacle.yaml" in start_script
    assert "mppi_nav_height_obstacle" in stop_script


if __name__ == "__main__":
    main()
