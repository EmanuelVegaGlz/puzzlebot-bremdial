## Jetson 1 

```bash
ros2 launch puzzlebot_real_robot real_robot_core.launch.xml \
  launch_micro_ros:=true \
  micro_ros_device:=/dev/ttyUSB0 \
  launch_lidar:=true \
  lidar_serial_port:=/dev/ttyUSB1 \
  launch_localization:=false \
  launch_joint_states:=true
```

## Jetson 2

```bash
ros2 launch puzzlebot_ros aruco_jetson.launch.py
```

## Jetson 3

```bash
ros2 launch puzzlebot_sim localization_aruco_computer.launch.py \
  use_sim_time:=false \
  enable_aruco:=false \
  enable_aruco_ekf:=true \
  enable_bug:=true \
  enable_path_generator:=true
```

## Rem
```bash
ros2 launch puzzlebot_sim localization_aruco_computer.launch.py \
  use_sim_time:=false \
  enable_rviz:=true \
  enable_image_view:=true \
  image_topic:=/video_source/raw
```