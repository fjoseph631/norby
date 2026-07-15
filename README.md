# Norby

ROS 2 + Webots simulation workspace for a TurtleBot3 Burger with SLAM, Nav2 navigation, camera perception, and optional radar fusion.

## Prerequisites

- [ROS 2](https://docs.ros.org/) Humble or Jazzy (Linux or WSL recommended)
- [Webots](https://cyberbotics.com/) R2025a
- `webots_ros2` for your ROS distro, for example:

```bash
sudo apt install ros-${ROS_DISTRO}-webots-ros2 \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-slam-toolbox \
  python3-colcon-common-extensions
```

## Quick start

```bash
source /opt/ros/<distro>/setup.bash
./install.sh
source install/setup.bash
ros2 launch norby_webots_drivers controllers.launch.py
```

`install.sh` will:

1. Install ROS dependencies with `rosdep`
2. Download YOLO weights for object detection
3. Install Python packages from `requirements.txt`
4. Patch world files for GPS/IMU/camera/lidar alignment
5. Build the workspace with `colcon`

## Launch files

| Launch | Purpose |
|--------|---------|
| `controllers.launch.py` | Webots driver, diff-drive, camera, object tracker |
| `webots_only.launch.py` | Start Webots with `apartment.wbt` only |
| `slam.launch.py` | Point cloud to laser scan + `slam_toolbox` |
| `navigation.launch.py` | Nav2 bringup |
| `robot.launch.py` | Full stack: Webots + controllers + SLAM + Nav2 |

## Sensor configuration

The URDF (`urdf/TurtleBot3Burger.urdf`) and Webots worlds are aligned on these ROS topics:

| Sensor | Webots device | ROS topic |
|--------|---------------|-----------|
| GPS | `gps` | `/gps_sensor`, `/gps_sensor/speed` |
| IMU | `inertial unit`, `gyro(1)`, `accelerometer(1)` | `/imu` |
| Camera | `camera` | `/front_camera/camera_raw` |
| Lidar | `LDS-01` | `/base_cloud` |
| Odometry | diff-drive controller | `/odom` |
| Commands | Nav2 / teleop | `/cmd_vel` |

The TurtleBot `extensionSlot` must include GPS, IMU, camera, and `RobotisLds01`. Reference fragment:

`src/norby_webots_drivers/worlds/turtlebot_extension_slot.inc.wbt`

Run `python3 scripts/update_worlds.py` after editing worlds to re-apply sensor alignment and pin `EXTERNPROTO` URLs to Webots **R2025a**.

## Packages

- `norby_webots_drivers` — launch files, Webots bridge, sensor nodes
- `object_tracking` — YOLO detection and Kalman object tracking library

## Tests

```bash
source install/setup.bash
colcon test --packages-select norby_webots_drivers
colcon test-result --verbose
```

Integration tests verify URDF sensor plugins, world device markers, R2025a `EXTERNPROTO` pins, config files, and launch file generation.

## Supported platforms

| Platform | Support |
|----------|---------|
| Ubuntu + ROS 2 | Primary |
| WSL2 + ROS 2 | Supported |
| Windows native | Experimental (controller spawner uses `python.exe`; full stack not validated) |

## Project layout

```text
norby/
├── install.sh
├── requirements.txt
├── scripts/update_worlds.py
└── src/
    └── norby_webots_drivers/
        ├── config/          # Nav2, SLAM, ros2_control
        ├── launch/
        ├── urdf/
        ├── worlds/          # Installed Webots worlds
        └── object_tracking/
```

## Known limitations

- Object tracking requires YOLO weights and a working `pyyolo` install.
- `apartment.wbt` must be patched by `install.sh` / `update_worlds.py` to include GPS and IMU devices.
- Nav2 ships with `turtlebot3_world.yaml` as a placeholder map name; generate a map with SLAM before autonomous navigation in a new environment.
