# Elevation Sensor Origin Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline and request an independent code review before completion.

**Goal:** Make registered-cloud fusion use the current physical LiDAR origin while preserving map geometry.

**Architecture:** Add a validated CPU affine geometry adapter and subscriber sensor_frame; reuse the existing sensor-frame GPU kernels. Use nonblocking bounded-age TF lookups and only advance accepted observation timestamps.

**Tech Stack:** ROS2 Humble, NumPy, tf2, CuPy/CUDA, Docker, pytest/colcon.

**Spec:** `docs/superpowers/specs/2026-11-04-elevation-sensor-origin-design.md`

## Global Constraints

- MPPI checkpoint 3c59a62 must precede external edits.
- Preserve unrelated external working changes and make only own-increment commits.
- No changes to radar point rate, FAST-LIVO, MPPI, CUDA interfaces, or new aggressive filtering.
- Use isolated ROS domains and report all test failures and actual field limits.

## Review Focus

- World-registered input must not use the world-frame origin as sensor position.
- Sensor rotation and additional data channels must survive re-expression.
- Latest TF fallback must be bounded and must never substitute a guessed origin.
- Rejected cloud data must not advance published observation age.
- Raw/synthetic launch defaults must retain their correct physical sensor frames.

## Task 1: Source origin and transport fix

- [x] Write real callback regressions before modification and confirm failure.
- [x] Add geometry helper, explicit sensor-frame lookup and bounded-age TF behavior; preserve GPU API.
- [x] Wire real and synthetic raw launch/config and add focused geometry, launch and GPU tests.
- [x] Build external package, run focused/full package checks and isolated topic validation; benchmark overhead.
- [x] Independent review, update external status/experience and commit only this task's increments.

## Ledger

- Main checkpoint 3c59a62 created after 52/52 Python tests and staged whitespace checks.
- External initial HEAD 4ea09a2; branch fix/cloud-sensor-origin created with user working edits preserved. Container initially stopped, command bash, bind-mounted workspace.
- RED:8 callback failures/2 passes confirmed fixed-world-origin error. Independent review exposed zero-stamped dynamic fallback bypass and missing accepted-fallback warning; two additional RED cases added then fixed.
- 18 new feature tests pass, real GPU reference/gate checks and isolated ROS domain144 smoke pass. Adapter 18000-point CPU timing P50 1.336/P95 1.373 ms, excludes rest of pipeline.
- Full package checks ran:64 cases with10 errors/10 failures across6/15 groups, all neural-mode startup/import failures due missing libcudss.so.0. Independent import torch reproduces it. Raw mode does not load this backend. Existing MCAP dependency absent too; no unrelated dependency changes made.
- External commit4190bb9 includes only this task's code/config increments, docs, state and experience. Original user file changes remain unstaged and their net line delta was checked unchanged. Main integration report:docs/analysis/elevation_sensor_origin_2026-11-04/report.md.
- Follow-up explicit authorization to commit remaining changes:external3b1ebbf saves the remaining raw map-frame/7m configuration,tests and documents. Fresh build succeeds;23 focused/raw/default-frame tests pass;domain144 ROS recheck passes;main52 Python tests pass. Read-only bag comparison and synthetic GPU numeric evidence are in comparison.md alongside reproducible scripts/CSV/JSON. These do not validate field navigation or total latency.
