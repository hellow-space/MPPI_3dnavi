# Velocity output guard implementation plan

> **For agentic workers:** Execute inline using superpowers:executing-plans.

**Goal:** Give all existing navigation scripts a single smoothed velocity output and continuous idle zeros.

**Architecture:** MPPI/recovery → `/command/cmd_twist` → smoother →
`/command/cmd_twist_smoothed` → keep_zero → `/cmd_vel`.

**Tech Stack:** ROS 2 Humble, Python/rclpy, Nav2 1.1.20, launch_ros.

**Spec:** `docs/superpowers/specs/2026-11-08-velocity-output-guard-design.md`

## Global Constraints

- Preserve startup scripts, MPPI parameters, 20 Hz zero timer and 0.3 s timeout.
- Change both shared launch files and `src/keep_zero.py`; update affected docs.
- Work in the existing checkout so the approved shared-script fix is available
  there; no unrelated branch changes or restart of the active navigation stack.
- Exercise ROS traffic only on explicitly remapped test topics in an isolated domain.

## Review Focus

- Both launch entry points override Nav2's default smoother output remap.
- Raw MPPI messages never bypass smoothing or enter the final guard directly.
- A stopped ROS clock cannot prevent fallback zeros.
- Invalid values on any Twist component cannot renew the watchdog.
- Stopping raw input and exiting the smoother both leave final output at zero.

### Task 1: Route and guard velocity output

**Files:** two `src/fishbot_navigation2/launch/navigation*.launch.py` files,
`src/keep_zero.py`, `tests/test_velocity_output_guard.py`,
`src/custom_path_navigation/README.md`, `README.md`.

**Interfaces:** preserve raw `/command/cmd_twist`; introduce
`/command/cmd_twist_smoothed`; final `/cmd_vel` remains Twist.

- [x] Add tests exercising actual guard output, invalid input, steady timer,
      both launch remap contexts, and a real isolated smoother/guard chain.
- [x] Run tests against existing code; confirm failures identify old routing,
      ROS-clock dependency and missing validation.
- [x] Add the explicit smoother-output override to both launch scopes, change
      the guard input, use depth one and steady time, reject nonfinite input.
- [x] Run targeted tests, existing Python and shell tests; build fishbot_navigation2.
- [x] Update navigation documentation; inspect the complete diff and verify
      no active navigation process was restarted. Report the next-restart boundary.

## Verification results

- Targeted regression tests failed on the original code: six failures across
  five tests, including both launch entry points.
- `python3 tests/test_velocity_output_guard.py -v`: five passed after the fix,
  in isolated ROS domain 193 with explicitly remapped test topics.
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q`: 57 passed.
- All four `tests/test_navigation_*_scripts.sh` suites passed.
- `colcon build --packages-select fishbot_navigation2 --symlink-install
  --executor sequential`: one package built successfully.
- Independent review found no Critical/Important issues and reran the five
  targeted tests successfully in isolated domain 217. Composition was reviewed
  against the installed launch implementation and actual ROS topic resolution;
  the full composed navigation stack was not started by the tests.
- Existing navigation PID 52072 and guard PID 52040 retained their original
  startup times. Restart navigation and guard together to activate this change.
