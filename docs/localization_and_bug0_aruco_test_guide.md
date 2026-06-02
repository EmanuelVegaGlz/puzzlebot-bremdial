# ArUco EKF Localization and Bug0 Execution Guide

This guide tests the current two-node ArUco correction stack on
`papoi-aruco-integrate2`:

- `puzzlebot_sim/localization` owns `/odom` and publishes `odom -> base_footprint`.
- `puzzlebot_sim/aruco_ekf_localization` reads `/odom` and
  `/marker_publisher/markers`, then publishes `/aruco_ekf/odom_correction`.
- `localization` applies each correction and the corrected pose/covariance is
  visible on `/odom` and in RViz.
- The Bug0 controller is tested only after encoder odometry, ArUco detections,
  EKF corrections, lidar, and TF are working.

## 1. Build and Source

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

On every new terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/atad/puzzlebot-bremdial/install/setup.bash
```

If the robot bringup is sourced from another workspace, source it after ROS and
before this workspace:

```bash
source /home/atad/puzzlebot_ros2/install/setup.bash
source /home/atad/puzzlebot-bremdial/install/setup.bash
```

## 2. Robot-Side Bringup

Start hardware drivers, lidar, robot description, and joint states. Disable the
real-robot localization node so there is only one `/odom` publisher.

```bash
ros2 launch puzzlebot_real_robot real_robot_core.launch.xml \
  launch_micro_ros:=true \
  micro_ros_device:=/dev/ttyUSB0 \
  launch_lidar:=true \
  lidar_serial_port:=/dev/ttyUSB1 \
  launch_localization:=false \
  launch_joint_states:=true
```

Check the hardware interfaces:

```bash
ros2 topic hz /VelocityEncR
ros2 topic hz /VelocityEncL
ros2 topic hz /scan
ros2 topic info /cmd_vel
```

Do not continue until encoders and lidar are publishing.

## 3. Localization and ArUco EKF Proof

Launch camera ArUco detection, wheel localization, the ArUco EKF correction
node, and RViz:

```bash
ros2 launch puzzlebot_sim localization_aruco_robot.launch.py \
  use_sim_time:=false \
  enable_aruco:=true \
  enable_aruco_ekf:=true \
  reference_frame:=base_footprint \
  camera_frame:=camera \
  marker_size:=0.094 \
  width:=1280 \
  height:=720 \
  camera_calibration_file:=file:///home/puzzlebot/.ros/jetson_cam.yaml \
  initial_x:=0.0 \
  initial_y:=0.0 \
  initial_theta:=0.0 \
  enable_bug:=false \
  enable_path_generator:=false \
  enable_rviz:=true
```

If an external ArUco detector is already running, launch only localization, EKF,
and RViz:

```bash
ros2 launch puzzlebot_sim localization_aruco_robot.launch.py \
  use_sim_time:=false \
  enable_aruco:=false \
  enable_aruco_ekf:=true \
  enable_bug:=false \
  enable_path_generator:=false \
  enable_rviz:=true
```

Required checks:

```bash
ros2 topic hz /VelocityEncR
ros2 topic hz /VelocityEncL
ros2 topic hz /marker_publisher/markers
ros2 topic hz /aruco_ekf/odom_correction
ros2 topic hz /odom
ros2 topic echo /aruco_ekf/odom_correction --once
ros2 topic echo /aruco_ekf/detected_markers --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint camera
```

Expected RViz result:

- Fixed Frame is `odom`.
- `/odom` shows pose and covariance.
- `/aruco_ekf/map_markers` shows known marker map positions.
- `/aruco_ekf/detected_markers` flashes live detections and rays.
- `/odom` changes smoothly after `/aruco_ekf/odom_correction` messages.

## 4. Manual EKF Test Procedure

Drive manually first:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Test sequence:

1. Place the robot at a measured start pose and launch with matching
   `initial_x`, `initial_y`, and `initial_theta`.
2. Keep the robot still and show one configured marker.
3. Confirm marker sign convention: marker ahead is positive robot-frame `x`,
   marker left is positive `y`, marker right is negative `y`.
4. Hide markers and drive straight; `/odom` should move by encoder odometry.
5. Drive past one marker; `/aruco_ekf/odom_correction` should publish and RViz
   should show correction without a wild jump.
6. Repeat with at least two non-collinear markers before trusting heading.

Record a bag for debugging:

```bash
ros2 bag record \
  /VelocityEncR /VelocityEncL \
  /marker_publisher/markers \
  /aruco_ekf/odom_correction \
  /aruco_ekf/map_markers \
  /aruco_ekf/detected_markers \
  /odom /scan /cmd_vel /tf /tf_static
```

Initial acceptance criteria:

- Only `localization` publishes `/odom`.
- Marker IDs match `src/puzzlebot_sim/config/aruco_ekf_params.yaml`.
- `marker_size` is `0.094` for 9.4 cm markers.
- Pose remains continuous without markers.
- Pose error improves after seeing known markers.
- Heading does not flip on correction.

## 5. Bug0 Integration

Run Bug0 only after the EKF proof passes.

Start localization, ArUco, EKF, RViz, and controller:

```bash
ros2 launch puzzlebot_sim localization_aruco_robot.launch.py \
  use_sim_time:=false \
  enable_aruco:=true \
  enable_aruco_ekf:=true \
  reference_frame:=base_footprint \
  marker_size:=0.094 \
  enable_bug:=true \
  enable_path_generator:=false \
  enable_rviz:=true
```

Publish a short first goal:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/Pose2D "{x: 0.6, y: 0.0, theta: 0.0}"
```

Increase difficulty in this order:

1. Open floor with one short goal.
2. Single wall.
3. One 90 degree turn.
4. Short corridor segment.
5. Real maze segment.
6. Full real maze.

For automatic path goals, enable the path generator:

```bash
ros2 launch puzzlebot_sim localization_aruco_robot.launch.py \
  use_sim_time:=false \
  enable_aruco:=true \
  enable_aruco_ekf:=true \
  reference_frame:=base_footprint \
  marker_size:=0.094 \
  enable_bug:=true \
  enable_path_generator:=true \
  enable_rviz:=true
```

## 6. Simulation/Single-Launch Checks

The single simulation launch can start the correction node without the Jetson
camera stack:

```bash
ros2 launch puzzlebot_sim puzzle_single_launch.py enable_aruco_ekf:=true
```

For normal current Bug0 simulation without ArUco correction:

```bash
ros2 launch puzzlebot_sim puzzle_single_launch.py
```

## 7. Troubleshooting

Missing `tf_transformations` or `transforms3d` on the Jetson:

The launch error below means the installed `puzzlebot_sim` Python entry points
were built from code that still imports external transform helpers:

```text
ModuleNotFoundError: No module named 'tf_transformations'
ModuleNotFoundError: No module named 'transforms3d'
```

First check the actual ROS and Ubuntu release on the Jetson. The reported stack
uses Python 3.8, which normally means Ubuntu 20.04/Focal, not Jammy.

```bash
echo "$ROS_DISTRO"
lsb_release -a
python3 --version
```

If the Jetson has no internet, generate the exact Debian package URL list on
the Jetson so the package versions and CPU architecture match the robot:

```bash
mkdir -p ~/offline_ros_deps
cd ~/offline_ros_deps
apt-get install --print-uris --yes \
  ros-${ROS_DISTRO}-tf-transformations \
  python3-transforms3d \
  | grep "^'" | cut -d"'" -f2 > urls.txt
```

Move `~/offline_ros_deps/urls.txt` to a computer with internet access, download
the packages, then move the downloaded `.deb` files back to the Jetson:

```bash
mkdir -p offline_ros_deps_debs
cd offline_ros_deps_debs
wget -i /path/to/urls.txt
```

On the Jetson, install the transferred packages:

```bash
cd ~/offline_ros_deps_debs
sudo apt install ./*.deb
```

Verify the imports before launching again:

```bash
python3 -c "import tf_transformations, transforms3d; print('transform deps ok')"
ros2 launch puzzlebot_sim puzzle_single_launch.py
```

If `urls.txt` is empty, APT already believes the packages are installed. Check
the Python environment and installed Debian packages:

```bash
dpkg -l | grep -E 'tf-transformations|transforms3d'
which python3
python3 -c "import sys; print(sys.executable); print(sys.path)"
```

The current source tree uses `puzzlebot_sim.transform_utils` instead of directly
importing those external modules. If you transfer the current source to the
Jetson, rebuild, and source the workspace, this specific import error should
also disappear:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select puzzlebot_sim
source install/setup.bash
```

No `/aruco_ekf/odom_correction`:

```bash
ros2 topic hz /marker_publisher/markers
ros2 topic echo /marker_publisher/markers --once
ros2 topic echo /odom --once
ros2 param get /aruco_ekf_localization marker_ids
```

RViz markers appear in the wrong place:

```bash
ros2 param get /aruco_ekf_localization marker_map_scale
ros2 param get /aruco_ekf_localization marker_measurement_frame
ros2 run tf2_ros tf2_echo base_footprint camera
```

Two `/odom` publishers:

```bash
ros2 topic info /odom
```

Stop the extra odometry source. For `puzzlebot_real_robot`, use
`launch_localization:=false` in `real_robot_core.launch.xml`.

Camera scale is wrong:

```bash
ros2 param get /marker_publisher marker_size
```

Use `0.094` for 9.4 cm markers. Recalibrate if `width` or `height` changes.

## 8. Test Log Template

```text
Date:
Robot start pose:
Initial pose parameters:
Initial covariance:
Marker IDs visible:
Camera resolution:
Calibration file:
Marker size parameter:
Localization params file:
Teleop or Bug0:
Path/goal:
/VelocityEncR rate:
/VelocityEncL rate:
/marker_publisher/markers rate:
/aruco_ekf/odom_correction rate:
/odom rate:
/scan rate:
Measured checkpoint error:
Heading error:
RViz correction visible:
Failure notes:
Next parameter change:
```
