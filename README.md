# tasc_drive

`tasc_drive` is a ROS 2 repository that lives inside a larger workspace, typically:

```text
~/asimov_ws/src/tasc_drive
```

This repo contains the drivetrain, Phidgets hardware integration, joystick teleop, RViz model, motor position publisher, and the rover dashboard package used to power motors through relays and monitor telemetry.

## Repository Layout

- [`phidgets_hardware`](phidgets_hardware): `ros2_control` hardware interface, teleop node, controller config
- [`drive_bringup`](drive_bringup): launch files for the drive stack
- [`drive_control`](drive_control): helper nodes such as the motor position publisher
- [`drive_description`](drive_description): URDF/xacro and RViz configuration
- [`rover_dashboard_pkg/rover_dashboard`](rover_dashboard_pkg/rover_dashboard): dashboard GUI and backend
- [`rover_dashboard_pkg/esp32_relay_controller`](rover_dashboard_pkg/esp32_relay_controller): ESP32 micro-ROS relay controller sketch and bring-up notes

## Current Capabilities

- relay-controlled motor power through an ESP32 over micro-ROS Wi-Fi
- motor telemetry publishing for position, velocity, and temperature
- standalone motor position publisher node for autonomy consumers
- joystick driving through `joy_node` and `ps4_teleop`
- safety behavior that blocks movement unless all 4 motors are attached and all 4 motor relays are enabled
- RViz visualization of the rover model from the URDF
- dashboard for motor power toggling and telemetry viewing

## Workspace Setup

This repo is meant to be cloned into a ROS 2 workspace source folder, not used as the workspace root itself.

Example:

```bash
mkdir -p ~/asimov_ws/src
cd ~/asimov_ws/src
git clone git@github.com:Combat-Craft/tasc_drive.git
cd ~/asimov_ws
colcon build
source install/setup.bash
```

If you only want to build the packages from this repo:

```bash
cd ~/asimov_ws
colcon build --packages-select phidgets_hardware drive_bringup drive_control drive_description rover_dashboard
source install/setup.bash
```

## Hardware Overview

The intended high-level data flow is:

1. GUI sends motor power commands
2. backend forwards those commands to the ESP32 relay controller
3. ESP32 toggles motor power relays and publishes relay state
4. Phidgets hardware interface detects attached motors and publishes telemetry
5. dashboard displays relay state plus motor telemetry
6. joystick teleop sends drive commands only when all 4 motors are powered and attached

## rqt Graph Overview

![rqt graph overview](docs/rqt_graph_system.png)

What the graph shows at a high level:

- `joint_state_broadcaster` publishes `/joint_states`, which feed `robot_state_publisher`
- `robot_state_publisher` publishes `/robot_description` and the robot TF tree used by RViz
- `phidgets_motor_telemetry_bridge` publishes `/rover/drive/motor_telemetry`
- `motor_position_publisher` republishes that into `/rover/drive/motor_positions`
- `rover_dashboard_backend` sits in the middle of the relay and GUI flow
- the dashboard backend subscribes to `/rover/relay_board/state`, `/rover/relay_board/heartbeat`, and `/rover/drive/motor_telemetry`
- the dashboard backend publishes `/rover/gui/telemetry` and `/rover/gui/heartbeat` for the GUI
- the GUI publishes `/rover/gui/command`, which the backend translates into `/rover/relay_board/command`
- `diff_drive_controller` consumes joystick-derived velocity commands on `/diff_drive_controller/cmd_vel_unstamped`

This graph is useful because it shows the full end-to-end chain:

1. relay power control through `/rover/relay_board/*`
2. dashboard aggregation through `/rover/gui/*`
3. drivetrain telemetry through `/rover/drive/*`
4. robot-state publication through `/joint_states` and `robot_state_publisher`

## ESP32 Relay Controller

The ESP32 used so far is:

- `ESP32-WROOM-32 DOIT ESP32 DEVKIT V1`

The relay sketch is here:

- [relay_controller_wifi.ino](rover_dashboard_pkg/esp32_relay_controller/relay_controller_wifi/relay_controller_wifi.ino)

The detailed relay-controller guide is here:

- [README.md](rover_dashboard_pkg/esp32_relay_controller/README.md)

## Important VM Networking Note

If you are using an Ubuntu VM:

- set the network adapter to `Bridged Adapter`
- use the `IP address of the VM`, not the host
- make sure the ESP32 and VM are on the same LAN

This matters because the ESP32 connects to the micro-ROS agent using the IP address configured in the sketch.

## First-Time ESP32 Bring-Up Summary

1. Install Arduino IDE or PlatformIO
2. Install ESP32 board support
3. Install `micro_ros_arduino`
4. Edit Wi-Fi credentials and the micro-ROS agent IP in the sketch
5. Upload the sketch to the ESP32
6. Start the micro-ROS agent
7. Start the dashboard backend and GUI
8. Verify relay state topics are publishing

## Run The micro-ROS Agent

If using Docker:

```bash
docker run --rm -it --net=host microros/micro-ros-agent:humble udp4 --port 8888
```

If using a native ROS install:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## Dashboard Bring-Up

From the workspace root:

```bash
cd ~/asimov_ws
source install/setup.bash
ros2 run rover_dashboard hardware_backend
```

In another terminal:

```bash
cd ~/asimov_ws
source install/setup.bash
ros2 run rover_dashboard dashboard
```

The dashboard:

- toggles motor power relays
- shows motor position, velocity, and temperature
- shows overall relay and motor health
- includes a launcher for the RViz URDF view

## Drive Stack Bring-Up

Start the drive stack:

```bash
cd ~/asimov_ws
source install/setup.bash
ros2 launch drive_bringup giskard_bringup.launch.py
```

This starts:

- `ros2_control_node`
- `robot_state_publisher`
- `joint_state_broadcaster`
- `diff_drive_controller`
- `motor_position_publisher`

Start joystick teleop:

```bash
cd ~/asimov_ws
source install/setup.bash
ros2 launch drive_bringup teleop.launch.py
```

This starts:

- `joy_node`
- `ps4_teleop`

## Joystick Behavior

Current joystick mapping:

- left stick `up/down` -> forward/backward
- right stick `left/right` -> turning
- default speed -> `50%`
- hold `R1` -> `100%` boost

Safety behavior:

- if any one motor relay is off, teleop publishes zero command
- if any one motor is not attached, the hardware interface blocks all motor commands

## Drive Geometry

Current diff-drive controller geometry:

- wheel separation: `0.39 m`
- wheel diameter: `0.185 m`
- wheel radius: `0.0925 m`

Configured in:

- [ros2_control_controllers.yaml](phidgets_hardware/config/ros2_control_controllers.yaml)

## RViz Model

The rover URDF/xacro is here:

- [phidgets_giskard.urdf.xacro](drive_description/description/phidgets_giskard.urdf.xacro)

The RViz config is here:

- [drive_model.rviz](drive_description/rviz/drive_model.rviz)

You can open RViz manually with:

```bash
cd ~/asimov_ws
source install/setup.bash
rviz2 -d ~/asimov_ws/install/drive_description/share/drive_description/rviz/drive_model.rviz
```

If using RViz with the robot model:

- a good fixed frame is usually `base_link` or `odom`
- the model should now render as a box chassis with 4 wheels

## Useful Topics

Relay and dashboard topics:

- `/rover/relay_board/command`
- `/rover/relay_board/state`
- `/rover/relay_board/heartbeat`
- `/rover/gui/command`
- `/rover/gui/telemetry`
- `/rover/gui/heartbeat`

Drive topics:

- `/joy`
- `/diff_drive_controller/cmd_vel_unstamped`
- `/joint_states`
- `/rover/drive/motor_telemetry`
- `/rover/drive/motor_positions`

## Useful Debug Commands

Check joystick input:

```bash
ros2 topic echo /joy
```

Check teleop output:

```bash
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped
```

Check relay state:

```bash
ros2 topic echo /rover/relay_board/state
```

Check motor positions:

```bash
ros2 topic echo /rover/drive/motor_positions
```

Check joint states:

```bash
ros2 topic echo /joint_states
```

## Recommended Launch Order

For full-system testing:

1. start the micro-ROS agent
2. start `hardware_backend`
3. start `dashboard`
4. start `giskard_bringup`
5. start `teleop.launch.py`
6. turn on all 4 motor relays in the dashboard
7. verify all 4 motors attach
8. drive using the joystick

## Common Problems

### ESP32 Connects But No ROS Traffic

- wrong agent IP in the sketch
- VM is not in bridged mode
- using host IP instead of VM IP
- firewall blocking UDP

### Motors Do Not Move

- one or more motor relays are off
- one or more motors are not attached
- teleop is not running
- `/joy` is active but `/diff_drive_controller/cmd_vel_unstamped` is not
- drive stack is not fully launched

### Commands Show 0.5 But Motors Still Feel Full Speed

- controller output may still be saturating against the normalized Phidgets command limit
- inspect:

```bash
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped
ros2 topic echo /joint_states
```

### Re-launching Is Inconsistent

- re-source the workspace before retrying:

```bash
cd ~/asimov_ws
source install/setup.bash
```

- make sure both `giskard_bringup` and `teleop.launch.py` are restarted
- verify there are no lingering old processes holding the joystick or hardware

## Wiring Diagram Placeholder

Add the final rover wiring diagram here or link to it from this README.

Suggested content:

- ESP32 to relay board wiring
- relay board to motor power wiring
- Phidgets controller wiring
- motor power path
- common ground strategy
# tasc_drive2
