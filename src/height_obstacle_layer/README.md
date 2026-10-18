# HeightObstacleLayer

This Nav2 costmap plugin samples a 3 x 3 neighborhood around every elevation-map
cell. If the known local maximum minus minimum is greater than or equal to
`max_height_jump`, the cell is written as `LETHAL_OBSTACLE` (254). Unknown
samples are ignored by default. Put Nav2's `InflationLayer` immediately after
this plugin to inflate the binary obstacles.

Each valid map replaces the previous snapshot, so disappeared obstacles are
cleared on the next costmap update. Missing, invalid, or stale input never
turns the old snapshot into free space: the layer keeps the last valid marks
and reports itself non-current. The independent `HeightJumpCritic` can remain
enabled as the controller-side stale/out-of-map fail-safe.

The optional `height_difference_map` publisher exposes a single GridMap layer
named `height_difference` for RViz and rosbag verification.
