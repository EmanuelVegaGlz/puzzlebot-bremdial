# World-Origin Localization, ArUco EKF, and Bug0 Guide

## Architecture

The real-robot stack separates continuous wheel odometry from globally
corrected localization:

```text
world_origin -> odom -> base_footprint -> base_link
```

- `/odom` is encoder-only and expressed in `odom`.
- `/localization/odom` is the fused robot pose in `world_origin`.
- `odom -> base_footprint` remains continuous.
- ArUco corrections change `world_origin -> odom`.
- The Bug0 controller consumes `/localization/odom`.

`world_origin` is the lower-left maze corner. `+X` points toward marker 70,
`+Y` points left, and `-Y` points right. Marker map positions, initial pose,
goals, and paths all use this frame.

Live marker observations use `base_footprint`: `+X` is robot-forward, `+Y` is
robot-left, and `-Y` is robot-right.

## Build and Source

```bash
cd /home/atad/puzzlebot-bremdial
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

If hardware packages come from another workspace, source that workspace before
this one.

## Robot Bringup

### Hardware

```bash
ros2 launch puzzlebot_real_robot real_robot_core.launch.xml \
  launch_micro_ros:=true \
  micro_ros_device:=/dev/ttyUSB0 \
  launch_lidar:=true \
  lidar_serial_port:=/dev/ttyUSB1 \
  launch_localization:=false \
  launch_joint_states:=true
```

### Camera and ArUco

```bash
ros2 launch puzzlebot_ros aruco_jetson.launch.py \
  width:=160 \
  height:=90 \
  camera_calibration_file:=file:///home/puzzlebot/.ros/cam_calib.yaml \
  camera_frame:=camera_link_optical \
  reference_frame:=base_footprint \
  marker_size:=0.094
```

OpenCV pose estimation starts with optical axes: `+X` image-right, `+Y` down,
and `+Z` forward. `aruco_ros` must identify that source as
`camera_link_optical` so TF can convert detections into `base_footprint`.

### Localization and EKF

```bash
ros2 launch puzzlebot_sim localization_aruco_robot.launch.py \
  use_sim_time:=false \
  enable_aruco:=false \
  enable_localization:=true \
  enable_aruco_ekf:=true \
  enable_controller:=false \
  enable_path_generator:=false \
  world_frame:=world_origin \
  odom_frame:=odom \
  base_frame:=base_footprint \
  initial_x:=0.3 \
  initial_y:=-0.3 \
  initial_theta:=0.0
```

`enable_aruco:=false` prevents a second detector from competing with the
external `puzzlebot_ros` launch.

### Workstation Visualization

```bash
ros2 launch puzzlebot_sim localization_aruco_computer.launch.py \
  use_sim_time:=false \
  enable_rviz:=true \
  enable_image_view:=true \
  image_topic:=/video_source/raw \
  world_frame:=world_origin
```

RViz uses `world_origin` as its fixed frame. The global odometry display uses
`/localization/odom`.

## Static Checks

```bash
ros2 topic info /odom -v
ros2 topic info /localization/odom -v
ros2 node info /controller
ros2 param get /aruco_ekf_localization marker_measurement_frame
ros2 param get /aruco_ekf_localization marker_max_age
ros2 param get /aruco_ekf_localization innovation_gate
ros2 param get /marker_publisher camera_frame
ros2 param get /marker_publisher reference_frame
ros2 run tf2_ros tf2_echo world_origin odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint camera_link_optical
ros2 run tf2_ros tf2_echo base_link laser_frame
```

Expected ownership:

- One publisher for `/odom`.
- One broadcaster for `odom -> base_footprint`.
- `localization` publishes `world_origin -> odom`.
- `robot_state_publisher` publishes the fixed sensor and body transforms.

## Marker-Axis Test

Inspect a raw detection:

```bash
ros2 topic echo /marker_publisher/markers --once
```

After the detector-side optical transform is correct:

- Marker ahead: `pose.pose.position.x > 0`, with `y` near zero.
- Marker right: `pose.pose.position.y < 0`.
- Marker left: `pose.pose.position.y > 0`.
- Marker header frame: `base_footprint`.

If a marker ahead remains positive `z`, the detector is still publishing
optical coordinates. Do not enable EKF corrections until this is fixed.

## Covariance and Gating

The default initial covariance represents 20 cm position and 15 degree heading
standard deviation:

```yaml
initial_covariance: [
  0.04, 0.0, 0.0,
  0.0, 0.04, 0.0,
  0.0, 0.0, 0.06854
]
```

Wheel uncertainty is motion-dependent. Each wheel uses a default distance
variance density of `0.0004 m^2/m`; covariance does not grow while stationary.

Initial marker measurement assumptions:

- Range variance: `0.01 m^2` (10 cm standard deviation).
- Bearing variance: `0.02 rad^2` (approximately 8.1 degrees).
- Two-dimensional normalized-innovation gate: `9.21`.
- Maximum marker age: `0.5 s`.

Tune these from recorded data rather than setting covariance to zero.

## EKF Acceptance Test

1. Measure the robot center in `world_origin` and launch with the closest
   practical initial pose.
2. Keep the robot stationary with no visible marker. Neither pose nor
   covariance should drift from timer activity.
3. Show one known marker and verify `/aruco_ekf/odom_correction` publishes.
4. Confirm `odom -> base_footprint` stays continuous.
5. Confirm `world_origin -> odom` changes as the global estimate is corrected.
6. Hide markers and drive straight. Both odometry topics should advance from
   encoder motion.
7. Repeat with multiple non-collinear markers before trusting heading.
8. Test an intentionally offset initial pose and verify global error decreases.

Useful commands:

```bash
ros2 topic echo /odom --once
ros2 topic echo /localization/odom --once
ros2 topic echo /aruco_ekf/odom_correction --once
ros2 topic echo /aruco_ekf/detected_markers --once
```

## Bug0 Integration

Enable the controller only after localization passes:

```bash
ros2 launch puzzlebot_sim localization_aruco_robot.launch.py \
  use_sim_time:=false \
  enable_aruco:=false \
  enable_localization:=true \
  enable_aruco_ekf:=true \
  enable_controller:=true \
  enable_path_generator:=false
```

Publish a short goal in `world_origin`:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/Pose2D \
  "{x: 0.6, y: 0.0, theta: 0.0}"
```

Verify the controller subscription:

```bash
ros2 node info /controller | grep /localization/odom
```

Increase difficulty from open floor to a single obstacle, turn, corridor, and
then the full maze.

## Laser Check

```bash
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo base_footprint laser_frame
```

Laser angle zero must point robot-forward, and positive angles must point left.
Do not tune Bug0 until this convention is physically verified.

## Record a Debug Bag

```bash
ros2 bag record \
  /VelocityEncR /VelocityEncL \
  /marker_publisher/markers \
  /aruco_ekf/odom_correction \
  /aruco_ekf/map_markers \
  /aruco_ekf/detected_markers \
  /odom /localization/odom \
  /scan /cmd_vel /tf /tf_static
```

Record the measured start pose, camera resolution, calibration file, marker
size, visible marker IDs, and measured checkpoint errors with every bag.
