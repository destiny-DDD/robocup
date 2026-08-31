# RViz SLAM Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enable RViz-specified goals while SLAM Toolbox builds the map online and the existing Nav2 stack drives to each goal.

**Architecture:** Pass `slam=True` and `use_localization=True` to Nav2 bringup from the project launch file. Nav2 will select its SLAM branch (SLAM Toolbox publishes `/map` and `map -> odom`) while the existing NavFn A*, MPPI, smoother, and collision monitor remain unchanged.

**Tech Stack:** ROS 2 Jazzy, Python launch, Nav2 bringup, SLAM Toolbox, YAML.

**Spec:** `docs/superpowers/specs/2026-08-31-rviz-slam-navigation-design.md`

## Global Constraints

- Only SLAM Toolbox publishes `map -> odom` in online mapping mode.
- Keep `/scan`, `/odom`, and `base_footprint` interfaces unchanged.
- Do not modify controller, planner, costmap, or motor-control behavior in this change.
- This supports manual RViz goals; autonomous frontier exploration is out of scope.

---

### Task 1: Select SLAM Bringup Mode

**Files:**
- Modify: `src/mynav2/launch/mynav.launch.py:32-36`
- Test: launch-file syntax and static argument inspection

**Interfaces:**
- Consumes: existing `nav_config` path and Nav2 `bringup_launch.py` launch arguments.
- Produces: a launch description that selects Nav2's SLAM branch and keeps localization enabled.

- [x] **Step 1: Update launch arguments**

Replace the launch argument mapping with:

```python
launch_arguments={
    'map': amcl_config,
    'slam': 'True',
    'use_localization': 'True',
    'params_file': nav_config,
}.items()
```

`slam=True` makes Nav2 include `slam_launch.py`; with `use_localization=True`, that launch starts SLAM Toolbox instead of the AMCL/map-server localization branch. The `map` argument is retained for compatibility but is ignored by the SLAM branch.

- [x] **Step 2: Run static validation**

Run:

```bash
python3 -m py_compile src/mynav2/launch/mynav.launch.py
python3 - <<'PY'
from pathlib import Path
text = Path('src/mynav2/launch/mynav.launch.py').read_text()
assert "'slam': 'True'" in text
assert "'use_localization': 'True'" in text
assert "'params_file': nav_config" in text
print('launch configuration checks passed')
PY
git diff --check
```

Expected: both checks print success and `git diff --check` exits 0.

- [ ] **Step 3: Verify runtime after rebuild/source**

Local build verification completed; ROS graph checks must be run on the robot
because this sandbox cannot access the running DDS graph.

Run on the ROS machine:

```bash
colcon build --packages-select mynav2
source install/setup.bash
ros2 launch mynav2 mynav.launch.py
```

In another shell, verify:

```bash
source install/setup.bash
ros2 node list | grep -E 'slam_toolbox|amcl|map_server|planner_server|controller_server'
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
```

Expected: `slam_toolbox`, `planner_server`, and `controller_server` are present; `amcl` and static `map_server` are absent; `/map` and exactly one `map -> odom` transform are available. Use RViz's Nav2 Goal tool to send a goal and observe `/cmd_vel`.

- [ ] **Step 4: Commit the implementation**

```bash
git add src/mynav2/launch/mynav.launch.py
git commit -m "feat: enable online SLAM navigation goals"
```
