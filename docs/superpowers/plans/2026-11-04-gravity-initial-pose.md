# Gravity Initial Pose Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Initialize map position/yaw without converting transient body tilt into map tilt.

**Architecture:** A ROS-independent calibration helper validates a stationary IMU window and computes gravity-constrained alignment; the existing ROS script gathers data and publishes the resulting fixed transform.

**Tech Stack:** Python, NumPy, ROS 2 Humble, unittest, tf2.

**Spec:** `docs/superpowers/specs/2026-11-04-gravity-initial-pose-design.md`

## Global Constraints

- Work on the user's `feat/location` branch; no external FAST-LIVO changes or real motion commands.
- Preserve full 3D LIO pose and the existing TF frame/topic contracts.
- No fallback to zero body roll/pitch on invalid or unavailable gravity.
- Record actual validation and field limitations; no unrequested commit.

## Review Focus

- Tilted camera_init and tilted body must not be confused.
- Accelerometer direction must be transformed at its measurement time.
- Manual reinitialization must preserve map up and the established height zero.
- Stationary noise, out-of-order data and missing transforms must not silently initialize.
- ROS callbacks must keep memory bounded and handle failures without killing the node.

## Task 1: Gravity-constrained alignment and ROS adapter

Files: `scripts/initial_pose_alignment.py`, `scripts/initial_pose_tf.py`, `tests/test_initial_pose_alignment.py`, `tests/test_initial_pose_tf.py`, `scripts/README_initial_pose.md`.

- [x] Write behavior tests for tilted body/camera, reinitialization, invalid data and stationary window checks; run and confirm failure.
- [x] Implement validated gravity/alignment math and retain current TF helper APIs where possible.
- [x] Integrate bounded IMU buffer, timestamp-matched TF, reset and actionable errors into the existing node.
- [x] Run focused tests, complete Python suite and isolated ROS transport smoke test.
- [x] Review complete diff, document startup/use/limitations, update ledger and report results.

## Execution ledger

- Branch `feat/location` was clean at start. User explicitly requested implementation after discussing scope; proceed inline without repeating approval.
- Executed inline on user's existing branch; no extra worktree, external source changes or commits.
- Initial callback RED: seven behavioral assertion failures plus an old-interface fixture error. Extra stationary gap/diagnostic RED observed and fixed.
- Fresh reviewer found two fail-open cases involving newer-than-TF motion and invalid recent samples. Both received failing regression tests then fixes; reviewer reran 28 focused tests and found no remaining critical/important issues.
- Full Python suite 52/52; isolated ROS domain143 transport smoke passed all five checks; syntax compilation and whitespace check passed. Evidence: `docs/analysis/initial_pose_tf_fix_2026-11-04/`.
