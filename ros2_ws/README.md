# Storagy ROS2 workspace

ROS2 Humble packages for rotating fallen-person detection and approach.

## Included

- `fallen_person_safety`: YOLO detection, approach controller, administrator notification
- `storagy`: robot bringup, Navigation2 integration, RViz configuration
- `motor_driver2`: Storagy motor driver
- `aruco_moving1`: existing ArUco docking package
- `best.pt`: trained `Fallen Person` YOLO model

Navigation2 and OrbbecSDK ROS2 are external dependencies and are not vendored
in this repository.

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Terminal 1:

```bash
ros2 launch storagy bringup.launch.py
```

Terminal 2:

```bash
ros2 launch fallen_person_safety fallen_person_safety.launch.py
```

RViz displays the annotated image from `/safety/debug_image`.
