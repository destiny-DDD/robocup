# RViz Goal Navigation With Online SLAM

## Goal

Allow an operator to set a goal in RViz while the robot builds or updates a
map with SLAM Toolbox and drives to that goal through the existing Nav2 stack.
This is manual goal-directed navigation, not autonomous frontier exploration.

## Architecture

`slam_toolbox` runs in mapping mode and consumes `/scan` plus the existing
`odom -> base_footprint` transform. It publishes the live `/map` and the
`map -> odom` transform. Nav2's global costmap consumes that map, the existing
NavFn planner uses A* (`use_astar: true`), and the existing MPPI controller
tracks the resulting path and sends velocity commands through the current
smoother and collision monitor.

Only one node may publish `map -> odom`: SLAM Toolbox. AMCL and the static
`map_server` must therefore be disabled in this mode. The `map_saver_server`
provided by the SLAM bringup remains available for saving the completed map.

## Files And Changes

- Update `src/mynav2/launch/mynav.launch.py` to pass `slam=True` while keeping
  `use_localization=True`, selecting Nav2's SLAM branch instead of AMCL.
- Keep `src/mynav2/config/nav2_params.yaml` as the shared Nav2 configuration,
  including the existing A* and controller settings. Its global static layer
  will consume the live `/map` published by SLAM Toolbox.
- Use the installed Jazzy online-sync SLAM Toolbox defaults, which match the
  project's `/scan`, `odom`, and `base_footprint` interfaces. A project-local
  SLAM parameter block can be added later if tuning is required.

## Runtime Checks

After launch, verify that `slam_toolbox` and Nav2 are active, `amcl` is absent,
`/map` is being published, and exactly one `map -> odom` transform exists.
Send a goal from RViz and confirm the action reaches `bt_navigator` and the
velocity pipeline publishes `/cmd_vel`.

## Constraints

The robot must provide `/scan`, a valid laser-to-base transform, and a usable
`odom -> base_footprint` transform. The current repository odometry is command
integration rather than encoder feedback, so map drift is an expected risk
until real wheel odometry is supplied.
