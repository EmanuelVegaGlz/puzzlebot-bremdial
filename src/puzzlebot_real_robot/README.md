# puzzlebot_real_robot

Bringup package for the physical Puzzlebot. It replaces the Gazebo bringup in
real-robot runs and provides the hardware core plus SLAM/Nav2 launch wrappers.

## Main Launches

- `real_robot_core.launch.xml`: robot description, optional micro-ROS agent,
  LiDAR, encoder odometry, `odom -> base_footprint`, and wheel joint states.
- `slam_real.launch.xml`: integrated physical SLAM bringup; starts the robot
  core by default, then SLAM Toolbox and RViz.
- `nav2_real.launch.xml`: integrated physical navigation bringup; starts the
  robot core by default, then Nav2, AMCL, map server, and RViz.
- `laptop_slam.launch.xml` and `laptop_nav2.launch.xml`: laptop-side wrappers
  for the two-machine workflow where the robot core is already running on the
  Jetson.

## Saving SLAM Maps

Create the package map directory before saving, especially after a fresh clone:

```bash
mkdir -p ~/puzzlebot_ros2/puzzlebot_real_robot/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/puzzlebot_ros2/puzzlebot_real_robot/maps/map_maze_real
```

The saver writes both `map_maze_real.pgm` and `map_maze_real.yaml`. If it logs
`Magick: Unable to open file`, the output directory is missing or not writable.

## TF Ownership

- `puzzlebot_localization.py` publishes `/odom` and `odom -> base_footprint`.
- `puzzlebot_joint_state_publisher.py` publishes only `/joint_states`.
- `robot_state_publisher` publishes the structural robot TF from
  `base_footprint` through `base_link`, wheels, and `laser_frame`.
- SLAM Toolbox or AMCL/Nav2 publishes `map -> odom`; never run both at the same
  time.

Expected TF chain:

```text
map -> odom -> base_footprint -> base_link -> laser_frame
```

If no encoder data has arrived yet, localization still publishes a stationary
`/odom` and `odom -> base_footprint` so RViz, SLAM, and Nav2 can start with a
valid odom frame.
