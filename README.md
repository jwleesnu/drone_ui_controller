# Drone UI Controller

`ui_controller` is a ROS 2 package that provides a simple Tkinter-based GUI to
send command IDs for drone control. The GUI publishes integer commands to a
topic, which can be consumed by your flight control or bridge node.

## Features
- One-click commands for Offboard, Arming, Takeoff, and Emergency Kill.
- Directional movement (Forward/Backward/Left/Right/Up/Down).
- Yaw control (CW/CCW) and Roll/Pitch adjustments.
- Home position set and reference tracking shortcut.

## Requirements
- ROS 2 
- Python 3 with Tkinter (`python3-tk`)

## Build
From your ROS 2 workspace:
```bash
cd ~/ros2_workspace/drone_ws
source /opt/ros/<ros_distro>/setup.bash
colcon build --packages-select ui_controller
source install/setup.bash
```

## Run
Launch the GUI node:
```bash
ros2 launch ui_controller ui_controller.launch.py
```

## Topics
- **Published**: `ui_command` (`std_msgs/Int32`)

## Command Mapping
The GUI publishes the following integer IDs on `ui_command`:

| ID | Command |
|---:|---------|
| 1  | Kill (Emergency) |
| 2  | Offboard |
| 3  | Arming |
| 4  | Takeoff |
| 11 | Forward |
| 12 | Backward |
| 13 | Left |
| 14 | Right |
| 15 | Up |
| 16 | Down |
| 17 | CW (Yaw) |
| 18 | CCW (Yaw) |
| 19 | Roll + |
| 20 | Roll - |
| 21 | Pitch + |
| 22 | Pitch - |
| 23 | Set Home Position |
| 24 | Reference Tracking |

## Notes
- The current launch file runs `src/ui_controller_new.py`.
- Make sure any subscriber node interprets the command IDs consistently.
