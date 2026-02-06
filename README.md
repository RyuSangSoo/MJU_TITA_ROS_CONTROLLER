# Tita UI

ROS2 Qt UI for monitoring TiTa IMU and joint states with live plots.

## Features
- Subscribes to IMU and JointState topics
- Computes roll/pitch/yaw from IMU orientation
- Stores all values in `TitaState`
- Graphs for IMU (roll/pitch/yaw) and joint 1/2/3 positions (Left/Right)
- Tita State table for Left/Right (Pos/Vel/Eff) + RPY values
- ESC to close the window

## Dependencies
Qt5 is required. On Ubuntu:
```bash
sudo apt-get update
sudo apt-get install -y qtbase5-dev
```

## Topics
Defaults:
- IMU: `/imu_sensor_broadcaster/imu`
- Joint States: `/joint_states`

You can override with parameters:
- `imu_topic`
- `joint_states_topic`

## Build
```bash
colcon build --packages-select tita_ui
```

## Run
```bash
source install/setup.bash
ros2 run tita_ui tita_ui_node
```

### Run with custom topics
```bash
ros2 run tita_ui tita_ui_node --ros-args \
  -p imu_topic:=/imu_sensor_broadcaster/imu \
  -p joint_states_topic:=/joint_states
```

## Joint Mapping
The following joint names are mapped to `TitaState`:
- `joint_left_leg_1` -> Left.J1
- `joint_left_leg_2` -> Left.J2
- `joint_left_leg_3` -> Left.J3
- `joint_left_leg_4` -> Left.Wheel
- `joint_right_leg_1` -> Right.J1
- `joint_right_leg_2` -> Right.J2
- `joint_right_leg_3` -> Right.J3
- `joint_right_leg_4` -> Right.Wheel

## Notes
- UI refresh is throttled to ~30Hz for smooth rendering.
- Default window mode is maximized (not fullscreen).
