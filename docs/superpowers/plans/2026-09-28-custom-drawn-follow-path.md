# RViz Drawn FollowPath Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an independent RViz freehand path tool and executor that sends the confirmed two-dimensional curve directly to Nav2 `FollowPath`, with preview, undo, cancellation, start-pose validation, and no elevation-map coupling.

**Architecture:** A new `custom_path_navigation` package contains a deterministic path-processing library, a ROS2 `custom_path_executor` action client, and an RViz Tool plugin. The Tool publishes raw `nav_msgs/Path` candidates and calls Trigger services; the executor owns validation, preview/status publication, and the single active FollowPath goal. New fishbot launch/RViz files and new scripts integrate the feature without modifying existing navigation profiles.

**Tech Stack:** ROS2 Humble, C++17, ament_cmake, rclcpp/rclcpp_action, Nav2 `nav2_msgs/action/FollowPath`, TF2, RViz2 pluginlib/Qt5/Ogre, GTest, Python/YAML and Bash integration tests.

**Spec:** `docs/superpowers/specs/2026-09-28-custom-drawn-follow-path-design.md`

## Global Constraints

- All path poses are two-dimensional in `map`: `z=0`; do not depend on GridMap, elevation topics, HeightObstacleLayer, or HeightJumpCritic.
- Do not modify existing navigation scripts, existing RViz templates, existing Nav2 YAML files, HeightObstacleLayer, HeightJumpCritic, or the reinforcement-learning controller.
- The Tool never publishes NavigateToPose goals, velocity commands, or zero-velocity commands.
- Mouse release only previews; only `Enter` may request FollowPath execution.
- The path's final Pose is the FollowPath terminal Pose; no global planner or waypoint conversion is involved.
- Default start tolerance is exactly `0.5 m`; never insert a hidden connector from the robot to the drawn start.
- A new path may replace an active goal only after cancellation is confirmed; never allow two active FollowPath goals.
- Preserve the current `/cmd_vel` to `/command/cmd_twist` remap in the new integration launch.
- Target the installed ROS2 Humble and Nav2 1.1.20 APIs; compile with `-Wall -Wextra -Wpedantic` and C++17.

## Review Focus

- A high-rate mouse stroke containing duplicate or degenerate points must produce a finite path or a clear rejection, never NaN yaw; Task 1 pins this with degenerate-input tests.
- A Fixed Frame change during a stroke must discard that stroke rather than publish mixed-frame coordinates; Task 3 pins this in the interaction-model tests.
- Repeated `Enter` while cancellation/replacement is pending must not send duplicate goals; Task 2 pins this with fake-server goal counts.
- An invalid raw candidate arriving after a valid one must invalidate execution instead of silently executing the previous path; Task 2 pins this in candidate-replacement tests.
- Late callbacks from an old goal after replacement must not overwrite the current goal's status; Task 2 pins this with generation-tagged fake-server results.

---

## File Structure

### New generic package

- `src/custom_path_navigation/package.xml` — package metadata and runtime/test dependencies.
- `src/custom_path_navigation/CMakeLists.txt` — core, executor, RViz plugin, installation and tests.
- `src/custom_path_navigation/rviz_plugins.xml` — `rviz_common::Tool` plugin export.
- `src/custom_path_navigation/include/custom_path_navigation/path_processor.hpp` — deterministic path processing API.
- `src/custom_path_navigation/src/path_processor.cpp` — filtering, smoothing, resampling and yaw generation.
- `src/custom_path_navigation/include/custom_path_navigation/execution_model.hpp` — executor states, generation tracking and replacement decisions.
- `src/custom_path_navigation/src/execution_model.cpp` — ROS-independent execution state transitions.
- `src/custom_path_navigation/include/custom_path_navigation/path_executor.hpp` — ROS node interface.
- `src/custom_path_navigation/src/path_executor.cpp` — topics, services, TF validation and FollowPath action client.
- `src/custom_path_navigation/src/path_executor_main.cpp` — node executable entry point.
- `src/custom_path_navigation/include/custom_path_navigation/interaction_model.hpp` — drawing/history/key-command state.
- `src/custom_path_navigation/src/interaction_model.cpp` — one-level undo and stroke lifecycle.
- `src/custom_path_navigation/include/custom_path_navigation/draw_follow_path_tool.hpp` — RViz Tool declaration.
- `src/custom_path_navigation/src/draw_follow_path_tool.cpp` — mouse/key handling and ROS interface calls.
- `src/custom_path_navigation/config/custom_path.yaml` — executor defaults fixed by the spec.
- `src/custom_path_navigation/README.md` — standalone interfaces and usage.
- `src/custom_path_navigation/test/test_path_processor.cpp` — pure algorithm tests.
- `src/custom_path_navigation/test/test_execution_model.cpp` — pure goal-state tests.
- `src/custom_path_navigation/test/test_path_executor.cpp` — fake TF/action-server integration tests.
- `src/custom_path_navigation/test/test_interaction_model.cpp` — stroke, undo and frame-change tests.
- `src/custom_path_navigation/test/test_rviz_plugin.cpp` — pluginlib load test.

### New project integration files

- `src/fishbot_navigation2/launch/navigation_draw_path.launch.py` — independent Nav2/RViz/executor launch.
- `src/fishbot_navigation2/rviz/navigation_draw_path.rviz` — copy of the current project view with the new Tool, Path and Marker entries.
- `scripts/start_navigation_draw_path.sh` — independent tmux startup with optional YAML filename.
- `scripts/stop_navigation_draw_path.sh` — independent clean shutdown.
- `tests/test_custom_path_navigation_integration.py` — config, dependencies, RViz and launch assertions.
- `tests/test_navigation_draw_path_scripts.sh` — executable script behavior.

### Existing files that must remain unchanged

- `scripts/start_navigation_height_obstacle.sh`
- `scripts/stop_navigation_height_obstacle.sh`
- `src/fishbot_navigation2/launch/navigation2.launch.py`
- `src/fishbot_navigation2/rviz/navigation.rviz`
- `src/fishbot_navigation2/config/nav2_params_height_obstacle.yaml`
- all files under `src/height_obstacle_layer/` and `src/height_jump_mppi_critic/`

---

### Task 1: Deterministic 2D Path Processor

**Files:**
- Create: `src/custom_path_navigation/package.xml`
- Create: `src/custom_path_navigation/CMakeLists.txt`
- Create: `src/custom_path_navigation/include/custom_path_navigation/path_processor.hpp`
- Create: `src/custom_path_navigation/src/path_processor.cpp`
- Create: `src/custom_path_navigation/test/test_path_processor.cpp`

**Interfaces:**
- Consumes: `nav_msgs::msg::Path` containing raw poses in one frame.
- Produces: `ProcessingConfig`, `ProcessingResult`, and `processPath(const nav_msgs::msg::Path &, const ProcessingConfig &) -> ProcessingResult` for Tasks 2 and 3.
- `ProcessingConfig` fields: `double min_raw_spacing`, `double min_path_length`, `double sample_spacing`, `int smoothing_passes`.
- `ProcessingResult` fields: `bool valid`, `std::string error`, `nav_msgs::msg::Path path`.

- [ ] **Step 1: Write failing path-processing tests**

Add named GTests covering:

```cpp
TEST(PathProcessor, RejectsEmptySinglePointNonFiniteAndTooShortPaths);
TEST(PathProcessor, RemovesPointsCloserThanTwoCentimeters);
TEST(PathProcessor, MovingAveragePreservesFirstAndLastPosition);
TEST(PathProcessor, ResamplesStraightLineAtFiveCentimeterArcSpacing);
TEST(PathProcessor, AssignsFiniteTangentYawAndZeroZToEveryPose);
TEST(PathProcessor, UsesFinalSegmentYawForTheTerminalPose);
TEST(PathProcessor, DegenerateDuplicateStrokeNeverProducesNanYaw);
TEST(PathProcessor, RejectsInvalidConfiguration);
```

Assert the exact defaults `0.02`, `0.20`, `0.05`, and one smoothing pass; exact first/last positions; `z == 0`; normalized finite quaternions; and the final Pose equal to the processed preview terminal Pose.

- [ ] **Step 2: Run the test target and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_path_navigation --cmake-args -DBUILD_TESTING=ON
```

Expected: FAIL because the package/API or implementation does not exist; the failure must name `path_processor` rather than an unrelated dependency.

- [ ] **Step 3: Add the package skeleton and public processing types**

Declare the exact interfaces above in `path_processor.hpp`. Add only dependencies needed by this task: `ament_cmake`, `geometry_msgs`, `nav_msgs`, `tf2`, `tf2_geometry_msgs`, and `ament_cmake_gtest`. Export the include directory and compile as C++17 with the required warnings.

- [ ] **Step 4: Implement filtering, bounded smoothing, arc-length resampling and yaw**

Implement `processPath` in this fixed order: validate config/frame/finite input; distance-filter raw points; verify length; run three-point moving-average passes while preserving endpoints; resample by cumulative arc length; calculate tangent yaw; force every `z=0`; preserve the input header frame and assign one coherent timestamp to all output poses.

Do not use splines or extrapolate beyond either endpoint. If filtering leaves a zero-length segment, skip it; if no nonzero segment remains, return invalid.

- [ ] **Step 5: Build and run the Task 1 tests**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_path_navigation --event-handlers console_direct+
colcon test --packages-select custom_path_navigation --ctest-args -R test_path_processor --event-handlers console_direct+
colcon test-result --test-result-base build/custom_path_navigation --verbose
```

Expected: `test_path_processor` passes with zero failures and compiler output contains no warnings from package sources.

- [ ] **Step 6: Commit Task 1**

```bash
git add src/custom_path_navigation
git commit -m "feat: add deterministic drawn path processing"
```

---

### Task 2: FollowPath Executor and Single-Goal State Machine

**Files:**
- Create: `src/custom_path_navigation/include/custom_path_navigation/execution_model.hpp`
- Create: `src/custom_path_navigation/src/execution_model.cpp`
- Create: `src/custom_path_navigation/include/custom_path_navigation/path_executor.hpp`
- Create: `src/custom_path_navigation/src/path_executor.cpp`
- Create: `src/custom_path_navigation/src/path_executor_main.cpp`
- Create: `src/custom_path_navigation/config/custom_path.yaml`
- Create: `src/custom_path_navigation/test/test_execution_model.cpp`
- Create: `src/custom_path_navigation/test/test_path_executor.cpp`
- Modify: `src/custom_path_navigation/CMakeLists.txt`
- Modify: `src/custom_path_navigation/package.xml`

**Interfaces:**
- Consumes from Task 1: `processPath(...) -> ProcessingResult`.
- Consumes ROS: `/custom_path/raw_path`, `/custom_path/execute`, `/custom_path/cancel`, `fixed_frame -> robot_frame` TF, and `/follow_path` action.
- Produces ROS: `/custom_path/preview_path`, `/custom_path/preview_marker`, `/custom_path/status`.
- Produces for Task 3: the exact topic/service contract from the spec.
- `ExecutionModel` exposes `setCandidate(bool valid)`, `requestExecute()`, `goalAccepted(uint64_t generation)`, `cancelConfirmed(uint64_t generation)`, `goalFinished(uint64_t generation, TerminalResult)`, `requestCancel()`, `state()`, `generation()`, and `hasPendingReplacement()`.

- [ ] **Step 1: Write failing execution-state tests**

Add GTests for:

```cpp
TEST(ExecutionModel, ValidCandidateTransitionsIdleToReady);
TEST(ExecutionModel, InvalidCandidateClearsExecutableCandidate);
TEST(ExecutionModel, ExecuteWithoutCandidateIsRejected);
TEST(ExecutionModel, ReplacementWaitsForConfirmedCancellation);
TEST(ExecutionModel, RepeatedExecuteWhileCancelPendingDoesNotDuplicateReplacement);
TEST(ExecutionModel, LateOldGoalResultCannotOverwriteNewGeneration);
TEST(ExecutionModel, RepeatedCancelIsIdempotent);
```

Assert the public state names `IDLE`, `READY`, `SENDING`, `ACTIVE`, `SUCCEEDED`, `FAILED`, and `CANCELED`, and strictly increasing goal generations.

- [ ] **Step 2: Run execution-model tests and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_path_navigation
colcon test --packages-select custom_path_navigation --ctest-args -R test_execution_model --event-handlers console_direct+
```

Expected: FAIL because `ExecutionModel` is missing.

- [ ] **Step 3: Implement the ROS-independent execution model**

Implement the interface above so action callbacks must carry the generation they belong to. Ignore callbacks from any generation older than `generation()`. A second execute request while a replacement is already pending returns a rejected decision and does not increment generation.

- [ ] **Step 4: Run execution-model tests and verify GREEN**

Run the Task 2 Step 2 command again.

Expected: all `test_execution_model` cases pass.

- [ ] **Step 5: Write failing executor integration tests with a fake FollowPath server**

Create fixtures using an isolated ROS domain and a fake `nav2_msgs::action::FollowPath` server. Cover:

```cpp
TEST_F(PathExecutorTest, InvalidRawPathReplacesAndInvalidatesPreviousCandidate);
TEST_F(PathExecutorTest, RejectsWrongFrameAndMissingTransform);
TEST_F(PathExecutorTest, AcceptsExactlyHalfMeterStartAndRejectsBeyondBoundary);
TEST_F(PathExecutorTest, ExecuteFailsWhenActionServerIsUnavailable);
TEST_F(PathExecutorTest, SendsProcessedPathControllerAndGoalCheckerIds);
TEST_F(PathExecutorTest, FollowPathTerminalPoseEqualsPreviewTerminalPose);
TEST_F(PathExecutorTest, CancelServiceCancelsActiveGoalAndClearsCandidate);
TEST_F(PathExecutorTest, ReplacementCancelsOldGoalBeforeSendingOneNewGoal);
TEST_F(PathExecutorTest, CancelFailureDoesNotSendReplacementGoal);
TEST_F(PathExecutorTest, LateOldResultDoesNotChangeReplacementStatus);
```

Assert that no test publishes Twist messages and that the fake server never observes two simultaneous accepted goals.

- [ ] **Step 6: Run executor integration tests and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/custom_path_executor_test_logs
colcon test --packages-select custom_path_navigation --ctest-args -R test_path_executor --event-handlers console_direct+
```

Expected: FAIL because `PathExecutor` and executable are missing.

- [ ] **Step 7: Implement PathExecutor topics, services, TF checks and asynchronous action flow**

Implement `PathExecutor : public rclcpp::Node`. Use reliable keep-last-one QoS for candidate/preview/status. Publish preview markers with green for valid READY, blue for ACTIVE, red for invalid or FAILED, and gray for terminal success/cancellation states.

Validate and declare all parameters from `config/custom_path.yaml`. On every raw message, process it and replace both candidate validity and preview; an invalid message must not leave the previous candidate executable. At execute time, re-check TF and the inclusive `distance <= start_tolerance` boundary, check action-server readiness, and then drive `ExecutionModel` through asynchronous goal/cancel callbacks tagged with the current generation.

Do not block an executor callback waiting for another callback. Use async action callbacks and pending replacement state; send a replacement only from confirmed cancellation completion. Publish status after every state transition.

- [ ] **Step 8: Add exact executor configuration and executable installation**

Create `config/custom_path.yaml` with the spec's exact defaults. Add runtime dependencies: `nav2_msgs`, `rclcpp`, `rclcpp_action`, `std_msgs`, `std_srvs`, `visualization_msgs`, `tf2`, and `tf2_ros`. Install `custom_path_executor` and the config directory.

- [ ] **Step 9: Build and run all Task 1-2 tests**

Run:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/custom_path_executor_test_logs
colcon build --packages-select custom_path_navigation --event-handlers console_direct+
colcon test --packages-select custom_path_navigation --event-handlers console_direct+
colcon test-result --test-result-base build/custom_path_navigation --verbose
```

Expected: path processor, execution model and executor integration suites all pass with zero failures.

- [ ] **Step 10: Commit Task 2**

```bash
git add src/custom_path_navigation
git commit -m "feat: execute drawn paths through Nav2 FollowPath"
```

---

### Task 3: RViz Freehand Tool and Undo Interaction

**Files:**
- Create: `src/custom_path_navigation/rviz_plugins.xml`
- Create: `src/custom_path_navigation/include/custom_path_navigation/interaction_model.hpp`
- Create: `src/custom_path_navigation/src/interaction_model.cpp`
- Create: `src/custom_path_navigation/include/custom_path_navigation/draw_follow_path_tool.hpp`
- Create: `src/custom_path_navigation/src/draw_follow_path_tool.cpp`
- Create: `src/custom_path_navigation/test/test_interaction_model.cpp`
- Create: `src/custom_path_navigation/test/test_rviz_plugin.cpp`
- Modify: `src/custom_path_navigation/CMakeLists.txt`
- Modify: `src/custom_path_navigation/package.xml`

**Interfaces:**
- Consumes from Task 2: `/custom_path/raw_path`, `/custom_path/execute`, `/custom_path/cancel`.
- Produces: plugin class ID `custom_path_navigation/DrawFollowPath`, type `custom_path_navigation::DrawFollowPathTool`, base `rviz_common::Tool`.
- `InteractionModel` exposes `beginStroke(std::string frame)`, `append(Point2D)`, `finishStroke()`, `cancelStroke()`, `undo()`, `clear()`, `frameChanged(std::string)`, `currentPath()`, and `previousPath()`.

- [ ] **Step 1: Write failing interaction-model tests**

Add GTests:

```cpp
TEST(InteractionModel, MouseReleaseCreatesReadyCandidateWithoutExecuteCommand);
TEST(InteractionModel, NewStrokeReplacesCandidateAndKeepsOneUndoLevel);
TEST(InteractionModel, CtrlZRestoresPreviousCandidateOnlyOnce);
TEST(InteractionModel, EscapeDuringDrawingDiscardsOnlyCurrentStroke);
TEST(InteractionModel, EscapeWhenReadyClearsCandidate);
TEST(InteractionModel, FixedFrameChangeDuringStrokeDiscardsMixedFrameStroke);
TEST(InteractionModel, EmptyOrOnePointStrokeDoesNotReplaceValidHistory);
```

- [ ] **Step 2: Run interaction tests and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select custom_path_navigation --ctest-args -R test_interaction_model --event-handlers console_direct+
```

Expected: FAIL because the interaction API is absent.

- [ ] **Step 3: Implement one-level candidate history and frame-safe stroke lifecycle**

Implement the exact `InteractionModel` interface. It stores only current, previous and in-progress strokes. `finishStroke()` changes candidates but never emits an execute decision. A frame change invalidates only the in-progress stroke; completed paths keep their original frame until replaced or cleared.

- [ ] **Step 4: Run interaction tests and verify GREEN**

Run the Task 3 Step 2 command again.

Expected: all interaction tests pass.

- [ ] **Step 5: Write a failing pluginlib load test**

Use `pluginlib::ClassLoader<rviz_common::Tool>` to assert the declared class list contains `custom_path_navigation/DrawFollowPath` and that `createSharedInstance` returns a `DrawFollowPathTool`. The test must not require an X display or initialize a RenderPanel.

- [ ] **Step 6: Run plugin test and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select custom_path_navigation --ctest-args -R test_rviz_plugin --event-handlers console_direct+
```

Expected: FAIL because the plugin XML/shared library does not exist.

- [ ] **Step 7: Implement DrawFollowPathTool mouse and keyboard behavior**

Override `onInitialize`, `activate`, `deactivate`, `processMouseEvent`, and `processKeyEvent`. Obtain the RViz ROS node from `DisplayContext`. Convert window coordinates to the fixed-frame `z=0` plane using the Ogre camera viewport ray intersecting `Ogre::Plane(Ogre::Vector3::UNIT_Z, 0)`; discard events with no forward intersection.

On left-down call `beginStroke(current_fixed_frame)`; while left is held append world points; on left-up finish and publish the current raw path. Map `Enter` to `/custom_path/execute`, `Esc` to local clear plus `/custom_path/cancel`, and `Ctrl+Z` to undo and republish. Do not auto-deactivate the Tool and do not publish any goal or Twist message.

- [ ] **Step 8: Export and install the RViz plugin**

Add `rviz_common`, `rviz_rendering`, `pluginlib`, Qt5 Widgets, Ogre, and `nav_msgs` dependencies. Export `rviz_plugins.xml` for `rviz_common`, link the shared library, install headers/library/XML, and ensure AUTOMOC is enabled for `Q_OBJECT`.

- [ ] **Step 9: Build and run all package tests**

Run:

```bash
source /opt/ros/humble/setup.bash
export QT_QPA_PLATFORM=offscreen ROS_LOG_DIR=/tmp/custom_path_rviz_test_logs
colcon build --packages-select custom_path_navigation --event-handlers console_direct+
colcon test --packages-select custom_path_navigation --event-handlers console_direct+
colcon test-result --test-result-base build/custom_path_navigation --verbose
```

Expected: all package tests pass; pluginlib finds the Tool; no display server is required.

- [ ] **Step 10: Commit Task 3**

```bash
git add src/custom_path_navigation
git commit -m "feat: draw FollowPath routes in RViz"
```

---

### Task 4: Independent Project Launch, RViz Template and Scripts

**Files:**
- Create: `src/custom_path_navigation/README.md`
- Create: `src/fishbot_navigation2/launch/navigation_draw_path.launch.py`
- Create: `src/fishbot_navigation2/rviz/navigation_draw_path.rviz`
- Create: `scripts/start_navigation_draw_path.sh`
- Create: `scripts/stop_navigation_draw_path.sh`
- Create: `tests/test_custom_path_navigation_integration.py`
- Create: `tests/test_navigation_draw_path_scripts.sh`

**Interfaces:**
- Consumes from Task 3: installed executable, config, plugin class ID and ROS interfaces.
- Produces: `navigation_draw_path.launch.py` arguments `map`, `use_sim_time`, `params_file`, and `custom_path_params_file`; scripts with optional YAML basename; tmux session `mppi_nav_draw_path`.

- [ ] **Step 1: Write failing integration/config tests**

In `test_custom_path_navigation_integration.py`, assert:

- `package.xml` and CMake contain no `grid_map`, `height_obstacle`, or `height_jump` dependency.
- `custom_path.yaml` contains every exact default from the spec.
- new RViz config keeps the robot box and existing navigation displays, loads Tool class `custom_path_navigation/DrawFollowPath`, subscribes Path to `/custom_path/preview_path`, and Marker to `/custom_path/preview_marker`.
- new launch uses the new RViz file, starts `custom_path_executor`, forwards the Nav2 params file, preserves `/cmd_vel -> /command/cmd_twist`, and starts the same robot description publisher.
- the existing launch, RViz, scripts and Nav2 YAML contain no new custom-path references.

- [ ] **Step 2: Write failing script tests**

Use fake `tmux`, `gnome-terminal`, and `sleep` executables. Cover:

```text
default argument selects nav2_params_height_obstacle.yaml
an existing alternate basename is passed unchanged
missing YAML exits nonzero before creating tmux
absolute paths, names containing '/', '..', or non-YAML suffixes are rejected
repeat start interrupts and closes the old session before creating a new one
failed old-session shutdown prevents new startup
stop is idempotent and still kills the session if pane Ctrl-C fails
project paths containing spaces remain correctly shell-quoted
```

- [ ] **Step 3: Run integration tests and verify RED**

Run:

```bash
python3 tests/test_custom_path_navigation_integration.py
bash tests/test_navigation_draw_path_scripts.sh
```

Expected: FAIL because the new launch, RViz and scripts do not exist.

- [ ] **Step 4: Add the independent RViz template**

Copy the current visual layout into the new file without modifying the source template. Add the custom Tool and enabled preview Path/Marker displays with reliable volatile QoS. Keep Fixed Frame `map`, the robot description topic, costmaps, plans and MPPI visualization already present.

- [ ] **Step 5: Add the independent integration launch**

Implement the same Nav2 include, map/params/use_sim_time arguments, robot box publisher and command remap as the current launch, but point RViz to `navigation_draw_path.rviz` and start `custom_path_executor` with `custom_path_params_file`. Do not import or alter the existing launch in a way that launches a second RViz.

- [ ] **Step 6: Add validated start/stop scripts**

Derive the project root from the script location and shell-quote it. Accept zero or one YAML basename. Validate with a basename-only regex ending in `.yaml`, then require the resolved file under `src/fishbot_navigation2/config/` to exist before touching tmux. Use three panes matching the current HeightObstacle startup: new navigation launch, `keep_zero.py`, and `initial_pose_tf.py`.

- [ ] **Step 7: Document operator controls and no-hardware behavior**

In the new package README, document Add/selected Tool behavior, drag/release/Enter/Esc/Ctrl+Z, start proximity, script syntax, status colors, FollowPath prerequisites, and the fact that absent TF/action prevents execution without publishing motion.

- [ ] **Step 8: Run integration tests and old-profile regressions**

Run:

```bash
python3 tests/test_custom_path_navigation_integration.py
bash tests/test_navigation_draw_path_scripts.sh
bash tests/test_navigation_height_obstacle_scripts.sh
bash tests/test_navigation_height_jump_scripts.sh
python3 tests/test_height_obstacle_config.py
python3 tests/test_nav2_height_jump_config.py
```

Expected: all commands exit zero; both old script suites still print PASS.

- [ ] **Step 9: Commit Task 4**

```bash
git add src/custom_path_navigation/README.md \
  src/fishbot_navigation2/launch/navigation_draw_path.launch.py \
  src/fishbot_navigation2/rviz/navigation_draw_path.rviz \
  scripts/start_navigation_draw_path.sh scripts/stop_navigation_draw_path.sh \
  tests/test_custom_path_navigation_integration.py \
  tests/test_navigation_draw_path_scripts.sh
git commit -m "feat: launch drawn path navigation profile"
```

---

### Task 5: Whole-Feature Verification

**Files:**
- Modify only if verification exposes a defect in files created by Tasks 1-4.

**Interfaces:**
- Consumes: every deliverable from Tasks 1-4.
- Produces: verified build/install/plugin/launch evidence and a clean final review package.

- [ ] **Step 1: Run formatting and static configuration checks**

Run:

```bash
ament_uncrustify \
  src/custom_path_navigation/include/custom_path_navigation/*.hpp \
  src/custom_path_navigation/src/*.cpp \
  src/custom_path_navigation/test/*.cpp
bash -n scripts/start_navigation_draw_path.sh scripts/stop_navigation_draw_path.sh
xmllint --noout src/custom_path_navigation/package.xml src/custom_path_navigation/rviz_plugins.xml
python3 -m py_compile src/fishbot_navigation2/launch/navigation_draw_path.launch.py
```

Expected: zero style divergence, valid XML, valid shell and valid Python syntax.

- [ ] **Step 2: Build all affected packages from the workspace**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  custom_path_navigation fishbot_navigation2 \
  height_obstacle_layer height_jump_mppi_critic \
  --event-handlers console_direct+
```

Expected: four packages finish successfully with zero compiler errors.

- [ ] **Step 3: Run package and repository-level targeted tests**

Run:

```bash
source install/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/custom_path_final_test_logs
export QT_QPA_PLATFORM=offscreen
colcon test --packages-select custom_path_navigation --event-handlers console_direct+
colcon test-result --test-result-base build/custom_path_navigation --verbose
python3 tests/test_custom_path_navigation_integration.py
bash tests/test_navigation_draw_path_scripts.sh
bash tests/test_navigation_height_obstacle_scripts.sh
bash tests/test_navigation_height_jump_scripts.sh
```

Expected: zero failed tests; targeted old-profile regressions pass.

- [ ] **Step 4: Verify plugin discovery and launch construction**

Run:

```bash
source install/setup.bash
ros2 pkg prefix custom_path_navigation
rg "custom_path_navigation/DrawFollowPath" \
  install/custom_path_navigation/share/custom_path_navigation/rviz_plugins.xml
ros2 launch fishbot_navigation2 navigation_draw_path.launch.py --show-args
```

Expected: package prefix resolves inside this workspace, the installed plugin XML contains the class ID, and launch argument construction exits zero. Set `ROS_LOG_DIR=/tmp/custom_path_launch_logs` if the default ROS log directory is not writable.

- [ ] **Step 5: Review the final diff against the spec's non-coupling constraints**

Run:

```bash
git status --short
git diff --check
rg -n "grid_map|elevation|HeightObstacle|HeightJump" src/custom_path_navigation || true
git diff -- \
  scripts/start_navigation_height_obstacle.sh \
  scripts/stop_navigation_height_obstacle.sh \
  src/fishbot_navigation2/launch/navigation2.launch.py \
  src/fishbot_navigation2/rviz/navigation.rviz \
  src/fishbot_navigation2/config/nav2_params_height_obstacle.yaml \
  src/height_obstacle_layer src/height_jump_mppi_critic
```

Expected: no whitespace errors; no elevation coupling in the new package; no diff in the protected existing files.

- [ ] **Step 6: Request a fresh whole-feature code review**

Review specifically: goal replacement races, service/action callback threading, stale action callbacks, RViz fixed-frame changes, script path validation, absence of direct velocity publication, and no modification to protected files. Fix Critical/Important findings with a new RED→GREEN test before re-running Steps 1-5.

- [ ] **Step 7: Commit verification fixes if any**

```bash
git add src/custom_path_navigation \
  src/fishbot_navigation2/launch/navigation_draw_path.launch.py \
  src/fishbot_navigation2/rviz/navigation_draw_path.rviz \
  scripts/start_navigation_draw_path.sh scripts/stop_navigation_draw_path.sh \
  tests/test_custom_path_navigation_integration.py \
  tests/test_navigation_draw_path_scripts.sh
git commit -m "test: verify drawn FollowPath workflow"
```

Skip this commit only when verification required no file changes.
