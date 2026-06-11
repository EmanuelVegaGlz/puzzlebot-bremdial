# Four-Command Real Robot Guide

The single-robot deployment uses four processes across the robot and
workstation. Keep the device-selection arguments identical in commands 3 and 4.

## Coordinate Frames

The localization tree is:

```text
world_origin -> odom -> base_footprint -> base_link
```

- `world_origin` is fixed to the lower-left maze corner. `+X` points toward
  marker 70, `+Y` points left, and `-Y` points right.
- `odom` is the continuous wheel-odometry frame. It may drift but never jumps.
- `base_footprint` uses robot axes: `+X` forward and `+Y` left.
- ArUco corrections update `world_origin -> odom`; they do not reset
  `odom -> base_footprint`.

## 1. Robot Hardware Core

Run on the robot/Jetson. Keep hardware localization disabled so
`puzzlebot_sim/localization` is the only owner of `/odom` and
`odom -> base_footprint`.

```bash
ros2 launch puzzlebot_real_robot real_robot_core.launch.xml \
  launch_micro_ros:=true \
  micro_ros_device:=/dev/ttyUSB0 \
  launch_lidar:=true \
  lidar_serial_port:=/dev/ttyUSB1 \
  launch_localization:=false \
  launch_joint_states:=true
```

## 2. Robot Camera and ArUco

Run the detector supplied by `puzzlebot_ros` on the robot:

```bash
ros2 launch puzzlebot_ros aruco_jetson.launch.py \
  width:=320 \
  height:=180 \
  camera_calibration_file:=file:///home/puzzlebot/.ros/cam_calib.yaml \
  camera_frame:=camera_link_optical \
  reference_frame:=base_footprint \
  marker_size:=0.094
```

The calibration must match the configured image resolution. The detector
transforms OpenCV optical coordinates through `camera_link_optical` and
publishes marker poses in `base_footprint`.

## 3. Robot-Side Compute Stack

Run on the robot. `enable_aruco:=false` is required because command 2 owns the
camera and `/marker_publisher/markers`.

```bash
ros2 launch puzzlebot_sim localization_aruco_robot.launch.py \
  use_sim_time:=false \
  enable_aruco:=false \
  enable_localization:=true \
  enable_aruco_ekf:=true \
  enable_controller:=true \
  enable_path_generator:=true \
  world_frame:=world_origin \
  odom_frame:=odom \
  base_frame:=base_footprint \
  initial_x:=0.36 \
  initial_y:=-0.27 \
  initial_theta:=0.0 \
  localization_device:=robot \
  aruco_ekf_device:=robot \
  controller_device:=robot \
  path_generator_device:=robot
```

## 4. Workstation Stack

Run on the computer with the same placement values: 

```bash
ros2 launch puzzlebot_sim localization_aruco_computer.launch.py \
  use_sim_time:=false \
  enable_rviz:=true \
  enable_image_view:=true \
  image_topic:=/marker_publisher/result \
  enable_localization:=true \
  enable_aruco_ekf:=true \
  enable_controller:=true \
  enable_path_generator:=true \
  world_frame:=world_origin \
  odom_frame:=odom \
  base_frame:=base_footprint \
  localization_device:=robot \
  aruco_ekf_device:=robot \
  controller_device:=robot \
  path_generator_device:=robot
```

## Manually Toggle Next Goal

```bash
ros2 topic pub --once /next_goal std_msgs/msg/Empty "{}" 
```

## Placement Presets

All compute on the robot:

```bash
localization_device:=robot aruco_ekf_device:=robot controller_device:=robot path_generator_device:=robot
```

Localization on the robot and navigation compute on the workstation:

```bash
localization_device:=robot aruco_ekf_device:=robot controller_device:=computer path_generator_device:=computer
```

Only camera/ArUco on the robot:

```bash
localization_device:=computer aruco_ekf_device:=computer controller_device:=computer path_generator_device:=computer
```

## Required Checks

```bash
ros2 topic hz /VelocityEncR
ros2 topic hz /VelocityEncL
ros2 topic hz /scan
ros2 topic echo /marker_publisher/markers
ros2 topic hz /odom
ros2 topic hz /localization/odom
ros2 topic hz /aruco_ekf/odom_correction
ros2 topic info /odom -v
ros2 param get /aruco_ekf_localization marker_timestamp_policy
ros2 run tf2_ros tf2_echo world_origin odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint camera_link_optical
timedatectl status
chronyc tracking
```

Exactly one localization and one controller must be active. A marker directly
ahead must have positive `x` after the detector publishes it in
`base_footprint`; right is negative `y`, and left is positive `y`. Run the
clock checks on both machines. Soft timestamp handling tolerates bad metadata,
but synchronized clocks provide more accurate motion compensation.
