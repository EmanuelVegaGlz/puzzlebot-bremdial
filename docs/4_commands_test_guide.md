## Dual-Device Launch Guide

Use the real-robot core for hardware only, then run the selectable
localization/controller stack from `puzzlebot_sim` on whichever device is best.

The selectable launch arguments are shared by the robot and computer launch
files:

| Argument | Values | Default |
| --- | --- | --- |
| `localization_device` | `robot`, `computer` | `robot` |
| `aruco_ekf_device` | `robot`, `computer` | `robot` |
| `controller_device` | `robot`, `computer` | `robot` |
| `path_generator_device` | `robot`, `computer` | `robot` |

Keep the selected device arguments identical on both devices. A node starts only
when its `*_device` value matches the launch file being run.

## 1. Robot Hardware Core

Run this on the robot/Jetson. Keep `launch_localization:=false` so the
selectable `puzzlebot_sim` localization node is the only `/odom` owner.

```bash
ros2 launch puzzlebot_real_robot real_robot_core.launch.xml \
  launch_micro_ros:=true \
  micro_ros_device:=/dev/ttyUSB0 \
  launch_lidar:=true \
  lidar_serial_port:=/dev/ttyUSB1 \
  launch_localization:=false \
  launch_joint_states:=true
```

## 2. Robot-Side Selectable Stack

Run this on the robot/Jetson. This launch owns the camera/ArUco detector and
starts only the selected compute nodes whose `*_device:=robot`.

```bash
ros2 launch puzzlebot_sim localization_aruco_robot.launch.py \
  use_sim_time:=false \
  enable_aruco:=true \
  enable_localization:=true \
  enable_aruco_ekf:=true \
  enable_controller:=true \
  enable_path_generator:=true \
  localization_device:=robot \
  aruco_ekf_device:=robot \
  controller_device:=robot \
  path_generator_device:=robot
```

## 3. Computer-Side Selectable Stack

Run this on the computer. Use the same placement values as the robot-side
launch. This command starts RViz/image viewing plus any selected compute nodes
whose `*_device:=computer`.

```bash
ros2 launch puzzlebot_sim localization_aruco_computer.launch.py \
  use_sim_time:=false \
  enable_rviz:=true \
  enable_image_view:=true \
  image_topic:=/video_source/raw \
  enable_localization:=true \
  enable_aruco_ekf:=true \
  enable_controller:=true \
  enable_path_generator:=true \
  localization_device:=robot \
  aruco_ekf_device:=robot \
  controller_device:=robot \
  path_generator_device:=robot
```

## Placement Presets

All compute on robot, RViz only on computer:

```bash
localization_device:=robot aruco_ekf_device:=robot controller_device:=robot path_generator_device:=robot
```

Vision/localization on robot, controller/path generation on computer:

```bash
localization_device:=robot aruco_ekf_device:=robot controller_device:=computer path_generator_device:=computer
```

Only the camera/ArUco detector on robot, all navigation compute on computer:

```bash
localization_device:=computer aruco_ekf_device:=computer controller_device:=computer path_generator_device:=computer
```

## Notes

- `enable_bug:=true` still works as a compatibility alias for
  `enable_controller:=true`.
- If `enable_aruco:=false`, make sure another ArUco detector is publishing
  `/marker_publisher/markers`.
- Exactly one localization node should be active, because it publishes `/odom`
  and `odom -> base_footprint`.
- Exactly one controller should be active, because it publishes velocity
  commands.
