# ArUco EKF Localization and Bug0 Test Guide

This guide explains how to launch and test the current Puzzlebot localization stack with ArUco, wheel dead reckoning, and lidar, then how to use the semi-functional Bug0 controller as a later integration test.

The recommended proof path is localization first with manual teleop on the physical robot. Bug0 is useful after the EKF is behaving, but it is not a clean proof of localization quality yet. The Bug system has only worked to a limited degree in a basic wall simulation, has not succeeded in simulated maze testing, and has not been tested in the real maze. From this point forward, ArUco testing should be treated as physical robot testing.

## Current Status

Working pieces in this repo:

- `puzzlebot_sim/localization` subscribes to `/VelocityEncR`, `/VelocityEncL`, and `/marker_publisher/markers`, then publishes `/odom`.
- `src/puzzlebot_sim/config/aruco_ekf_params.yaml` contains the physical marker IDs and map positions. Positions are stored in centimeters and converted to meters with `marker_map_scale: 0.01`.
- `localization` now exposes `initial_x`, `initial_y`, `initial_theta`, and `initial_covariance` so the robot is not locked to one hard-coded start pose.
- `localization` now starts with non-zero covariance, so accepted ArUco detections can correct the EKF immediately.
- `src/puzzlebot_sim/launch/aruco_jetson.launch.py` is a parameterized equivalent of the provided Jetson ArUco launch.
- `src/puzzlebot_sim/launch/localization_aruco_robot.launch.py` can start ArUco detection, EKF localization, RViz, and optionally Bug0/path generation from one command.
- `controller` now starts with one `bug_mode` declaration, uses a front lidar sector for Bug hit detection, applies `max_v` and `max_w`, and exposes wall-following constants as ROS parameters.

Important limitations:

- This repo still does not contain the physical base driver. The robot side must publish encoders, lidar, and accept `/cmd_vel`.
- The ArUco detector package launched by `ros2 launch puzzlebot_ros aruco_jetson.launch.py` is external to this repo. Apply the same parameterization shown here if that external launch remains the source of truth.
- The EKF publishes `/odom` messages, but it does not currently publish an odom TF transform. The controller uses `/odom` directly, but TF consumers need a separate TF source or a future `publish_tf` addition.
- Global relocalization from a completely unknown pose is not mathematically guaranteed from one marker observation. For origin-independent operation, use the initial pose parameters when the start is known; if the start is unknown, use high initial covariance and require multiple known markers from different angles before trusting the pose.

## Required Runtime Interfaces

Localization-only test:

| Topic | Type | Producer | Consumer |
| --- | --- | --- | --- |
| `/VelocityEncR` | `std_msgs/msg/Float32` | Robot base driver | `localization` |
| `/VelocityEncL` | `std_msgs/msg/Float32` | Robot base driver | `localization` |
| `/marker_publisher/markers` | `aruco_msgs/msg/MarkerArray` | `aruco_ros/marker_publisher` | `localization` |
| `/odom` | `nav_msgs/msg/Odometry` | `localization` | RViz, controller |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Teleop or controller | Robot base driver |

Bug0 integration test adds:

| Topic | Type | Producer | Consumer |
| --- | --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Lidar driver | `controller` |
| `/goal` | `geometry_msgs/msg/Pose2D` | `path_generator` or manual command | `controller` |
| `/next_goal` | `std_msgs/msg/Empty` | `controller` | `path_generator` |

ArUco detection adds:

| Topic | Type | Producer | Consumer |
| --- | --- | --- | --- |
| `/video_source/raw` | `sensor_msgs/msg/Image` | `ros_deep_learning/video_source` | `aruco_ros/marker_publisher` |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_info_publisher` | `aruco_ros/marker_publisher` |

Frame requirements:

- The provided ArUco launch uses `reference_frame: base` and `camera_frame: camera` by default.
- `aruco_ekf_params.yaml` uses `marker_measurement_frame: base_xy`, meaning accepted marker poses must already be expressed as robot-frame `+x` forward and `+y` left.
- If the physical robot uses `base_link` or `base_footprint`, launch ArUco with `reference_frame:=base_link` or the correct available frame.
- Verify the TF from the reference frame to the camera frame before trusting marker positions.

## Physical Launch Sequence: EKF Proof With Teleop

Use this sequence before involving Bug0.

### 1. Build and Source

From this workspace:

```bash
colcon build
source install/setup.bash
```

On every new terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/atad/puzzlebot-bremdial/install/setup.bash
```

If your physical robot bringup lives in a second workspace, source that too:

```bash
source /home/atad/puzzlebot_ros2/install/setup.bash
```

### 2. Start the Physical Robot Base

Start the robot-side launch that provides encoders, lidar, and `/cmd_vel`. If using the sibling `puzzlebot_real_robot` package found on this machine, use this pattern and disable its odometry publisher to avoid two `/odom` sources:

```bash
ros2 launch puzzlebot_real_robot real_robot_core.launch.xml \
  launch_micro_ros:=true \
  micro_ros_device:=/dev/ttyUSB0 \
  launch_lidar:=true \
  lidar_serial_port:=/dev/ttyUSB1 \
  launch_localization:=false \
  launch_joint_states:=true
```

If using another robot bringup package, confirm it provides:

```bash
ros2 topic hz /VelocityEncR
ros2 topic hz /VelocityEncL
ros2 topic hz /scan
ros2 topic info /cmd_vel
```

Do not run another node that publishes `/odom` while testing `puzzlebot_sim/localization`.

### 3. Start ArUco, EKF, and RViz

Recommended single command from this repo:

```bash
ros2 launch puzzlebot_sim localization_aruco_robot.launch.py \
  marker_size:=0.094 \
  width:=1280 \
  height:=720 \
  camera_calibration_file:=file:///home/puzzlebot/.ros/cam_calib.yaml \
  reference_frame:=base \
  camera_frame:=camera \
  initial_x:=0.0 \
  initial_y:=0.0 \
  initial_theta:=0.0 \
  enable_bug:=false \
  enable_path_generator:=false \
  enable_rviz:=true
```

If the external `puzzlebot_ros` ArUco launch is already running, disable the local ArUco include and start only EKF/RViz:

```bash
ros2 launch puzzlebot_sim localization_aruco_robot.launch.py \
  enable_aruco:=false \
  enable_bug:=false \
  enable_rviz:=true
```

### 4. Drive Manually

Run teleop in a separate terminal so the keyboard has a real TTY:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive slowly for the first tests. Stay below about `0.15 m/s` in the 60 cm maze corridors.

## External ArUco Launch Parameterization

The provided `puzzlebot_ros aruco_jetson.launch.py` must expose these launch arguments:

| Argument | Default | Reason |
| --- | --- | --- |
| `camera_resource` | `csi://0` | Jetson CSI camera input |
| `width` | `1280` | Must match the calibration file |
| `height` | `720` | Must match the calibration file |
| `camera_calibration_file` | `file:///home/puzzlebot/.ros/jetson_cam.yaml` | Camera intrinsics |
| `marker_size` | `0.094` | Physical marker side length in meters |
| `reference_frame` | `base` | Robot frame used by ArUco output |
| `camera_frame` | `camera` | Camera frame used by ArUco |
| `image_topic` | `/video_source/raw` | Image input to `aruco_ros` |
| `image_is_rectified` | `true` | Only true if the image is actually rectified |

The old value `marker_size: 0.165` is not compatible with 9.4 cm markers. It scales marker geometry by about `0.165 / 0.094 = 1.76`, which can make the EKF correction look plausible while being badly wrong.

Equivalent command:

```bash
ros2 launch puzzlebot_ros aruco_jetson.launch.py \
  marker_size:=0.094 \
  width:=1280 \
  height:=720 \
  camera_calibration_file:=file:///home/puzzlebot/.ros/jetson_cam.yaml
```

If a different image size is selected, recalibrate or select a calibration file for that exact resolution.

## Localization Verification Checklist

Before driving:

```bash
ros2 topic hz /VelocityEncR
ros2 topic hz /VelocityEncL
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 topic hz /scan
```

With a marker visible:

```bash
ros2 topic hz /marker_publisher/markers
ros2 topic echo /marker_publisher/markers --once
ros2 topic echo /aruco_ekf/detected_markers --once
```

Frame checks:

```bash
ros2 run tf2_ros tf2_echo base camera
```

Use `base_link camera` or `base_footprint camera` if those are the actual robot frames.

Parameter checks:

```bash
ros2 param get /localization initial_x
ros2 param get /localization initial_y
ros2 param get /localization initial_theta
ros2 param get /localization initial_covariance
ros2 param get /marker_publisher marker_size
```

## EKF Test Procedure

1. Place the robot at a measured start pose. If the pose is known, pass it with `initial_x`, `initial_y`, and `initial_theta`.
2. Keep the robot still and show one known marker. The marker ID must be listed in `aruco_ekf_params.yaml`.
3. Confirm marker sign convention:
   - marker in front of robot: positive robot-frame x
   - marker to robot left: positive robot-frame y
   - marker to robot right: negative robot-frame y
4. Cover or turn away from markers and drive forward/backward. `/odom` should move smoothly from wheel dead reckoning.
5. Drive past one known marker. `/aruco_ekf/detected_markers` should appear, and `/odom` should correct smoothly rather than jump wildly.
6. Repeat with at least two non-collinear markers. This is the minimum useful test for map geometry and heading consistency.
7. Repeat from at least three different start poses before claiming origin-independent behavior.
8. Compare `/odom` against tape-measured floor checkpoints.
9. Record a bag for repeatable analysis:

```bash
ros2 bag record \
  /VelocityEncR /VelocityEncL \
  /marker_publisher/markers \
  /aruco_ekf/detected_markers \
  /odom /scan /cmd_vel /tf /tf_static
```

Suggested first acceptance criteria:

- Correct marker IDs are reported.
- Marker range is plausible after `marker_size:=0.094`.
- Odometry remains continuous when no markers are visible.
- EKF corrections reduce measured drift after marker detections.
- Pose error is approximately within `0.10 m` to `0.15 m` after seeing two known markers.
- Heading does not flip or rotate abruptly on marker updates.
- Lidar readings in a centered corridor are plausible, with side walls near `0.30 m` from the lidar if the lidar is close to the robot center.

## Bug0 Integration Sequence

Do not use Bug0 as the first proof that localization works. Use it only after the localization checklist passes.

Start ArUco, EKF, RViz, controller, and path generator:

```bash
ros2 launch puzzlebot_sim localization_aruco_robot.launch.py \
  marker_size:=0.094 \
  enable_bug:=true \
  enable_path_generator:=true \
  enable_rviz:=true
```

For a manual one-cell goal, launch the controller without `path_generator`:

```bash
ros2 launch puzzlebot_sim localization_aruco_robot.launch.py \
  marker_size:=0.094 \
  enable_bug:=true \
  enable_path_generator:=false \
  enable_rviz:=true
```

Then publish a short goal:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/Pose2D "{x: 0.6, y: 0.0, theta: 0.0}"
```

Test in this order:

1. Open floor, no obstacles.
2. Single straight wall.
3. One 90 degree corner.
4. Short grid corridor.
5. Real maze segment.
6. Full real maze.

Keep goals to one grid cell or less at first. In this maze, `0.6 m` is already one full cell.

## Maze Geometry and Parameter Analysis

Physical maze:

- Total size: `3.30 m x 3.30 m`
- Hall width: `0.60 m`
- Marker size: `0.094 m x 0.094 m`
- Robot coverage radius: about `0.19 m`
- Robot coverage diameter: about `0.38 m`
- Centered side clearance: `(0.60 - 0.38) / 2 = 0.11 m`

Important parameter implications:

- `marker_size` must be `0.094`. The previous `0.165` value is a major scaling error.
- A centered robot can see side walls around `0.30 m`; global closest-obstacle logic with `bug_hit_dist: 0.50` treats normal corridor driving as a collision event.
- Bug hit detection should use a front sector, not the global closest lidar return. The controller now uses `front_sector` for this.
- `front_d_safety` near `0.28 m` is a better first physical value than `0.50 m` for 60 cm halls, but it must be tuned carefully because lidar position on the robot changes the true front clearance.
- `d_wall` should be near the expected centered side-wall distance, about `0.27 m` to `0.32 m`.
- `v_wall` should be slow for first physical tests. Start around `0.08 m/s` to `0.16 m/s`.
- `max_v` should stay near `0.10 m/s` to `0.18 m/s` until the real robot is stable in corridors.
- The maze is grid-based with 90 degree corners, so a future controller should include an explicit corner/turn state rather than relying only on continuous wall following.

Current first-pass physical values in `path_params.yaml`:

| Parameter | Value |
| --- | ---: |
| `bug_hit_dist` | `0.34` |
| `front_d_safety` | `0.28` |
| `front_sector` | `0.35 rad` |
| `d_safety` | `0.18` |
| `d_wall` | `0.30` |
| `v_wall` | `0.12` |
| `max_v` | `0.15` |
| `max_w` | `0.8` |
| `bug0_clear_shot_dist` | `0.60` |
| `bug0_clear_shot_sector` | `0.30 rad` |

## Recommended Future Improvements

Localization:

- Add optional TF publishing from the EKF if this stack should own `world -> base_link`.
- Add marker innovation logging and an innovation gate to reject large false corrections.
- Add an explicit relocalization mode that waits for two or more known markers before declaring the pose trustworthy.
- Add a launch argument or config profile for high-covariance unknown-start tests.

ArUco:

- Verify whether `/video_source/raw` is rectified. If not, set `image_is_rectified:=false` or add image rectification.
- Standardize `base`, `base_link`, and `base_footprint` naming across the robot, ArUco, EKF, and RViz.
- Keep camera calibration tied to the selected `width` and `height`.

Bug controller:

- Use side-sector lidar data for wall following instead of the global closest return.
- Add corridor-centering behavior for 60 cm halls.
- Add an explicit 90 degree corner state for the grid maze.
- Add an e-stop or watchdog that publishes zero velocity when required topics time out.
- Add repeatable tests around obstacle hit, leave condition, and timeout behavior.

Simulation:

- Treat basic-wall simulation as a component test only.
- Add a simulated maze that matches the `3.30 m x 3.30 m` physical maze.
- Add simulated ArUco markers with the same IDs and map coordinates as the physical markers.
- Do not claim maze readiness until the system passes both simulated maze and physical maze segment tests.

## Test Log Template

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
Teleop or Bug:
Path/goal:
Observed /VelocityEncR and /VelocityEncL rate:
Observed /marker_publisher/markers rate:
Observed /odom rate:
Observed /scan rate:
Measured checkpoint error:
Heading error:
Lidar hallway readings:
Failure notes:
Next parameter change:
```

## Bottom Line

Use teleop to prove the ArUco EKF first. Once marker scale, camera calibration, frame convention, encoder dead reckoning, lidar health, and EKF corrections are verified together, use Bug0 only as an integration test and start with one-cell goals in simple physical maze segments.
