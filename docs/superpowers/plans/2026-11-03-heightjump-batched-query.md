# HeightJump Batched Query Implementation Plan

> **For agentic workers:** Implement inline with the executing-plans workflow. No implementation delegation; request an independent review after verification.

**Goal:** Reduce HeightJump scoring latency while preserving every valid-input sample, decision, height range, unknown count and safety check.
**Architecture:** Prepare immutable per-map geometry in TerrainQuery, resolve the three x and three y coordinates independently using GridMap 2.0.1 arithmetic, then read the same nine cells in the original order. Feed samples into the existing evaluator; skip startup checks only when unknown is explicitly traversable.
**Tech Stack:** C++17, GridMap 2.0.1, ROS Humble, GTest.
**Spec:** docs/analysis/heightjump_optimization_2026-11-03/report.md (items 1 and 3, approved by user).

## Global Constraints

- Keep 1000 trajectories, 56 steps, dt 0.05, 20Hz target, thresholds, radius and unknown policies.
- Keep finite-coordinate prechecks, storage bounds, circular-buffer semantics, map validity/freshness and startup-region logic.
- Preserve floating-point subtraction order and division; do not replace division with reciprocal multiplication.
- No neighborhood memoization, threading, GPU, or Eigen migration this iteration.
- Work in the current workspace and preserve existing untracked analysis; no live robot motion or Nav2 launch.

## Review Focus

- Floating-point internal/map boundaries: test exact and adjacent doubles for both axes and nonintegral radius/resolution.
- Circular buffer: test both zero, either axis zero, and both nonzero start indices.
- Huge/nonfinite coordinates: no invalid integer conversions or out-of-bounds reads.
- Unknown permission: skip only redundant exemption computation, preserve known danger/outside precedence and full statistics.
- Immutable snapshots: prepared old queries remain valid across replacements.

### Task 1: Tests and implementation

- [x] Add and run failing evaluator test for redundant startup callback.
- [x] Add batch-neighborhood/differential/snapshot/fenv tests and confirm missing API fails to build.
- [x] Add TerrainQuery::neighborhood(x,y,radius), cached geometry and guarded axis conversion.
- [x] Add evaluateNeighborhood helper using the original evaluate ordering and switch HeightJumpCritic to this path.
- [x] Run scalar/batched differential correctness with native GridMap as independent reference.

### Task 2: Performance and integration

- [x] Extend paired benchmark with scalar prepared vs batch paths and full result equivalence.
- [x] Run standalone tests and paired performance before installing.
- [x] Back up installed plugin, build Release in the workspace, run package tests and Python suite.
- [x] Re-run the isolated full MPPI diagnostic and review the actual diff independently.
- [x] Report measured gains and limits; do not claim field update rate or circling fixed without a new field run.

## Execution ledger

- Ruling: work in the shared workspace authorized for this targeted fix; no branch operations or commits, preserve existing analysis.
- RED: unknown-pass exemption callback called 9 times instead of 0; batch tests failed to compile because neighborhood API was absent.
- GREEN: native 30 GTests, standalone 2 CTests, 24 Python tests; 242304 full-result differential cases matched.
- Paired flat score: prepared scalar 25.8001ms -> batch 3.94792ms P50.
- Installed Release plugin after backing up old .so in /tmp/mppi_heightjump_before_batch_20261103/.
- Isolated full optimizer+display: 40.3308ms -> 15.0005ms P50. Paced run retains scheduling/DVFS variability; field20Hz/circling not claimed.
- Independent reviewer found no material issues and reran standalone CTest 2/2.
