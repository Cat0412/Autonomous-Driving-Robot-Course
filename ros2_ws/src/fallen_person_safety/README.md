# Fallen person safety

## Main topics

- RGB input: `/camera/color/image_raw`
- Depth input: `/camera/depth/image_raw`
- Lidar input: `/scan`
- Safety motion output: `/cmd_vel_safety`
- Nav2 smoothed motion: `/cmd_vel_nav_smoothed`
- Motor motion after priority mux: `/cmd_vel`
- State: `/safety/state`
- Incident JSON: `/safety/incident`
- Alarm latch: `/safety/alarm`
- Buzzer request: `/safety/buzzer`
- Snapshot: `/safety/snapshot`
- Network-friendly JPEG snapshot: `/safety/snapshot/compressed`
- Annotated monitoring image: `/safety/debug_image`

## Run

```bash
cd ~/Desktop/storagy_ws2
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch fallen_person_safety fallen_person_safety.launch.py
```

The hardware camera, lidar, and motor driver must already be running. The
default model path is `<workspace>/best.pt`. Override it with:

```bash
ros2 launch fallen_person_safety fallen_person_safety.launch.py \
  model_path:=/absolute/path/to/best.pt
```

The controller stops at 0.50 m. If detection or ranging data becomes stale,
the robot publishes zero velocity and does not approach.

The Storagy hardware launch starts a velocity mux. Fresh safety commands have
priority over Nav2 commands, and only the mux publishes the final `/cmd_vel`
used by the motor driver. The hardware launch also enables RGB-depth
registration for safer distance measurement.
