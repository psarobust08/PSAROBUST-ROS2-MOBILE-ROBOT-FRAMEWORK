# PSA ROBUST — ROS 2 Mobile Robot Framework

A modular mobile robot control framework developed by **PSA ROBUST** using ROS 2 Humble, Python, OpenCV, and Teensy 4.1.

**Current Version:** V1.2  
**Status:** Software and USB Serial Integration Validated  
**Platform:** Ubuntu 22.04 / ROS 2 Humble

## 1. Project Overview

This project provides a modular architecture for vision-based mobile robot development.

The framework integrates:

- Camera image acquisition
- Blue ball detection using OpenCV
- Finite State Machine (FSM) control
- Robot velocity command generation
- Web-based START/STOP control
- Robot status monitoring
- Bidirectional serial communication with Teensy 4.1

The current implementation has been validated up to the microcontroller communication stage. Physical motor control is not yet implemented.

## 2. System Architecture

```text
                    WEB DASHBOARD
                       web_node
                      START/STOP
                          |
                          v
camera_node        detector_node
     |                  |
     +--- Image --------+
                        |
                   /detection
                        |
                        v
                     fsm_node
                        |
                 /fsm/ball_position
                        |
                        v
                robot_controller_node
                        |
                     /cmd_vel
                        |
                        v
                    serial_node
                        |
                     USB Serial
                        |
                        v
                     Teensy 4.1
```

The `status_node` subscribes to `/fsm/state` and publishes `/robot/status`.

The `web_node` also subscribes to `/detection`, `/ball/center`, and `/fsm/state`.

## 3. ROS 2 Nodes

| Node | Description |
|---|---|
| `camera_node` | Captures and publishes camera images |
| `detector_node` | Detects blue balls and determines L/C/R/N positions |
| `fsm_node` | Manages START/STOP and robot operational states |
| `robot_controller_node` | Converts ball position into velocity commands |
| `status_node` | Publishes robot operational status |
| `web_node` | Provides browser-based monitoring and control |
| `serial_node` | Bridges ROS 2 velocity commands to Teensy 4.1 |

## 4. ROS 2 Topics

| Topic | Message Type | Publisher | Subscriber |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | camera_node | detector_node |
| `/detection` | `std_msgs/msg/String` | detector_node | fsm_node, web_node |
| `/ball/center` | `geometry_msgs/msg/Point` | detector_node | web_node |
| `/fsm/state` | `std_msgs/msg/String` | fsm_node | status_node, web_node |
| `/fsm/ball_position` | `std_msgs/msg/String` | fsm_node | robot_controller_node |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | robot_controller_node | serial_node |
| `/robot/status` | `std_msgs/msg/String` | status_node | — |

## 5. ROS 2 Services

| Service | Type | Function |
|---|---|---|
| `/fsm/start` | `std_srvs/srv/Trigger` | Starts robot operation |
| `/fsm/stop` | `std_srvs/srv/Trigger` | Stops robot operation |

## 6. Ball Detection and Motion Mapping

| Detection | vx (m/s) | vy (m/s) | wz (rad/s) |
|---|---:|---:|---:|
| L | 0.0 | +0.2 | 0.0 |
| C | +0.2 | 0.0 | 0.0 |
| R | 0.0 | -0.2 | 0.0 |
| N | 0.0 | 0.0 | 0.0 |

The FSM permits motion commands only in the `RUNNING` state.

When stopped, the FSM publishes `N`, resulting in zero velocity.

These velocity values are preliminary and have not been calibrated for physical robot operation.

## 7. Teensy 4.1 Serial Communication

The `serial_node` provides bidirectional USB serial communication.

| Parameter | Value |
|---|---|
| Controller | Teensy 4.1 |
| Default port | `/dev/ttyACM0` |
| Baud rate | 115200 |
| Protocol | CSV |
| TX format | `vx,vy,wz\n` |
| RX format | `RX \| vx=... \| vy=... \| wz=...` |

Example command:

```text
0.200,0.000,0.000
```

The current Teensy firmware receives and parses velocity commands but does not control physical motors.

**Safety:** ROS 2 STOP is not a physical emergency stop. Hardware motor control must include an independent communication-loss watchdog and appropriate emergency-stop provisions before deployment.

## 8. Installation

### Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- OpenCV
- cv_bridge
- PySerial
- colcon

### Install Dependencies

```bash
sudo apt install python3-opencv python3-serial ros-humble-cv-bridge
```

Install the remaining ROS 2 package dependencies using:

```bash
cd ~/psarobust_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
cd ~/psarobust_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mobile_robot
source install/setup.bash
```

### Launch

```bash
ros2 launch mobile_robot robot.launch.py
```

### Web Dashboard

Open:

```text
http://localhost:8080
```

The dashboard provides ball position monitoring and START/STOP controls.

It uses an unauthenticated HTTP interface and should only be operated on a trusted development network.

## 9. Development Roadmap

| Version | Development Focus | Status |
|---|---|---|
| V1.2 | Basic ROS 2 framework and Teensy serial integration | Completed |
| V1.3 | YOLO-based object detection | Planned |
| V1.4 | X-drive holonomic motion control | Planned |
| V1.5 | Motor driver and encoder integration | Planned |
| V1.6 | Odometry and robot localization | Planned |
| V1.7 | Safety monitoring and diagnostics | Planned |
| V1.8 | Advanced web dashboard | Planned |
| V2.0 | Autonomous robot integration | Planned |

### Potential Node Upgrades

| Current Node | Future Enhancement |
|---|---|
| camera_node | CSI camera, GStreamer, camera calibration |
| detector_node | YOLO detection, object tracking |
| fsm_node | Advanced autonomous behaviour |
| robot_controller_node | Holonomic kinematics, PID control |
| status_node | Battery, CPU, motor and communication monitoring |
| web_node | Live camera streaming, telemetry, manual control |
| serial_node | Heartbeat, watchdog, reconnection and reliable communication |

### Potential Additional Nodes

- `odometry_node`
- `localization_node`
- `navigation_node`
- `safety_node`
- `data_logger_node`

These additional nodes are proposed extensions and are not part of V1.2.

## 10. Project Structure

```text
psarobust_ws/
├── src/
│   └── mobile_robot/
│       ├── mobile_robot/
│       │   ├── camera_node.py
│       │   ├── detector_node.py
│       │   ├── fsm_node.py
│       │   ├── robot_controller_node.py
│       │   ├── status_node.py
│       │   ├── web_node.py
│       │   └── serial_node.py
│       ├── launch/
│       │   └── robot.launch.py
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
└── README.md
```

## 11. Development Team

**PSA ROBUST**  
Politeknik Sultan Salahuddin Abdul Aziz Shah (PSA), Malaysia.

---

**PSA ROBUST — Modular Robotics Development with ROS 2**
