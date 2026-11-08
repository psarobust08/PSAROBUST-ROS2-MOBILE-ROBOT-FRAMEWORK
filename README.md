# PSA ROBUST — ROS 2 Mobile Robot Framework

A modular, vision-based mobile robot control framework developed by **PSA ROBUST**, Politeknik Sultan Salahuddin Abdul Aziz Shah (PSA), Malaysia. The project uses ROS 2 Humble, Python, OpenCV, a browser-based control panel, and a Teensy 4.1 microcontroller.

**Current version:** V1.4  
**Status:** Software integration, bidirectional USB serial communication, FSM heartbeat and heartbeat timeout validated  
**Development platform:** Ubuntu 22.04 / ROS 2 Humble  
**Hardware control:** Teensy serial receiver and heartbeat timeout tested; physical motor and LED control not yet implemented

## 1. Project Overview

The framework separates camera acquisition, object detection, operational state management, motion-command generation, status reporting, web control, and serial communication into independent ROS 2 nodes.

**Implemented through V1.4:**

- USB camera image acquisition with OpenCV.
- HSV-based blue ball detection with L/C/R/N classification and pixel-center coordinates.
- Finite State Machine (FSM) with START and STOP services.
- **Dynamic image-coordinate-based proportional control** generating both `linear.x` (`vx`) and `linear.y` (`vy`).
- Zero-velocity commands when stopped, when detection is unavailable, or when coordinate data is stale.
- Browser dashboard showing ball region, center coordinates, FSM state, and START/STOP buttons.
- Bidirectional CSV communication between ROS 2 and Teensy 4.1 via USB serial.
- **V1.4:** FSM heartbeat forwarded by `serial_node` to Teensy at 1 Hz, only when FSM status messages are recent.
- **V1.4:** Teensy heartbeat timeout detection with fallback to `IDLE`; verified through live serial testing.

**Not yet implemented:** YOLO inference, X-drive inverse kinematics, motor actuation, encoder feedback, physical odometry, physical LED strip control, or a hardware emergency-stop system.

## 2. System Architecture

```text
 USB Camera
     |
     v
 camera_node
     | /camera/image_raw   (sensor_msgs/Image)
     v
 detector_node -------------------------------> web_node
     |             /detection, /ball/center       ^
     |                                            |
     | /detection, /ball/center                   | /fsm/state
     v                                            |
  fsm_node <----------- START / STOP -------------+
     |                     (ROS 2 services)
     | /fsm/state ----------------> status_node
     |       |                        |
     |       |                   /robot/status
     |       +----------------------> serial_node (heartbeat input)
     |
     | /fsm/ball_position (L/C/R/N)
     | /fsm/ball_center   (cx, cy)
     v
 robot_controller_node
     | /cmd_vel (geometry_msgs/Twist)
     v
 serial_node <------ USB Serial RX ------> Teensy 4.1
                 USB Serial TX (velocity + FSM heartbeat)
```

The **FSM controls permission to move**. The **robot controller computes motion commands**. The **serial node owns the hardware communication interface**; the robot controller does not open or configure a serial port.

The Teensy firmware parses and echoes velocity commands, acknowledges FSM heartbeats, and detects heartbeat timeout. It **does not drive motors or physical LED outputs**.

## 3. ROS 2 Nodes

| Node | Current V1.4 responsibility |
|---|---|
| `camera_node` | Captures frames from a USB camera and publishes images. |
| `detector_node` | Detects a blue ball using HSV thresholding, classifies its image region as L/C/R/N, and publishes its pixel center when detected. |
| `fsm_node` | Provides START/STOP services, publishes FSM state, and forwards valid detection/coordinate data only while RUNNING. |
| `robot_controller_node` | Uses valid ball coordinates to compute dynamic `vx` and `vy` using proportional control; publishes `/cmd_vel`. |
| `status_node` | Republishes the latest FSM operational state as robot status. |
| `web_node` | Hosts a local web dashboard, displays ball and FSM data, and invokes START/STOP services. |
| `serial_node` | Converts `/cmd_vel` to CSV over USB serial, forwards fresh FSM status as a 1 Hz heartbeat, and logs Teensy responses. |

**Total application nodes: 7.**

## 4. ROS 2 Topics and Message Types

| Topic | Message type | Publisher | Subscriber(s) | Purpose |
|---|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `camera_node` | `detector_node` | Camera frames |
| `/detection` | `std_msgs/msg/String` | `detector_node` | `fsm_node`, `web_node` | `L`, `C`, `R`, or `N` |
| `/ball/center` | `geometry_msgs/msg/Point` | `detector_node` | `fsm_node`, `web_node` | Detected center in pixel coordinates (`x=cx`, `y=cy`, `z=0`) |
| `/fsm/state` | `std_msgs/msg/String` | `fsm_node` | `status_node`, `web_node`, `serial_node` | `IDLE`, `RUNNING`, or `STOPPED` |
| `/fsm/ball_position` | `std_msgs/msg/String` | `fsm_node` | `robot_controller_node` | Permitted L/C/R/N classification; `N` when movement is not permitted |
| `/fsm/ball_center` | `geometry_msgs/msg/Point` | `fsm_node` | `robot_controller_node` | Valid ball center forwarded during RUNNING |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `robot_controller_node` | `serial_node` | Body-frame velocity commands |
| `/robot/status` | `std_msgs/msg/String` | `status_node` | None in V1.4 | Published operational status |

**Total application topics: 8.** Standard ROS 2 system topics (for example `/rosout`) are not included in this count.

The point coordinates are **pixels, not meters**. When no ball is detected, the detector publishes `N` and does not publish a new ball center.

## 5. ROS 2 Services

| Service | Type | Server | Client | Purpose |
|---|---|---|---|---|
| `/fsm/start` | `std_srvs/srv/Trigger` | `fsm_node` | `web_node` or ROS 2 CLI | Transition to RUNNING |
| `/fsm/stop` | `std_srvs/srv/Trigger` | `fsm_node` | `web_node` or ROS 2 CLI | Transition to STOPPED |

**Total application services: 2.** Both return `success` (`bool`) and `message` (`string`). No custom `.msg` or `.srv` interfaces are used.

Manual examples:

```bash
ros2 service call /fsm/start std_srvs/srv/Trigger "{}"
ros2 service call /fsm/stop std_srvs/srv/Trigger "{}"
```

## 6. Dynamic Coordinate-Based Motion Control (V1.3)

### 6.1 Ball detection and FSM gating

The detector divides each frame into three horizontal image regions:

| Region | Condition |
|---|---|
| `L` | Ball center lies in the left third of the image |
| `C` | Ball center lies in the middle third |
| `R` | Ball center lies in the right third |
| `N` | No valid blue ball detected |

The FSM uses the following rule:

| FSM state | Detection | `/fsm/ball_position` | `/fsm/ball_center` |
|---|---|---|---|
| `IDLE` | Any | `N` | Not forwarded |
| `RUNNING` | `L`, `C`, or `R`, with fresh center | Same region | Forwarded |
| `RUNNING` | `N` or stale data | `N` | Not forwarded |
| `STOPPED` | Any | `N` | Not forwarded |

The FSM checks the freshness of detection and center data, with a configured **0.5 s timeout**.

### 6.2 Normalized image errors

With the current **640 × 480** camera configuration, image center is `(320, 240)`. For a detected ball center `(cx, cy)`:

```text
ex = (cx - 320) / 320
ey = (240 - cy) / 240
```

Here, `ex` describes horizontal displacement from the image center and `ey` describes vertical displacement. These are normalized image errors, not physical position errors.

### 6.3 Proportional velocity controller

The V1.3 controller calculates:

```text
vx = Kx * ey
vy = -Ky * ex
wz = 0
```

The deadband is applied to each normalized error before the velocity calculation, and each velocity component is limited to the configured maximum.

| Parameter | V1.3 value |
|---|---:|
| Image width | 640 px |
| Image height | 480 px |
| `Kx` | 0.3 |
| `Ky` | 0.3 |
| Maximum absolute velocity, per axis | 0.3 m/s |
| Normalized-error deadband | 0.05 |
| Controller coordinate timeout | 0.5 s |
| Control publish frequency | 10 Hz |
| `wz` | 0 rad/s |

Examples (assuming RUNNING, valid fresh detection, and no deadband suppression):

| Ball center `(cx, cy)` | `vx` (m/s) | `vy` (m/s) |
|---|---:|---:|
| (320, 240) | 0.000 | 0.000 |
| (160, 240) | 0.000 | +0.150 |
| (480, 240) | 0.000 | -0.150 |
| (320, 120) | +0.150 | 0.000 |
| (320, 360) | -0.150 | 0.000 |
| (480, 120) | +0.150 | -0.150 |
| No valid detection or STOPPED | 0.000 | 0.000 |

Both `vx` and `vy` can be non-zero simultaneously. This is **two-axis, holonomic-style body-velocity command generation**, **not yet four-wheel X-drive motor control**. `wz` remains zero.

**Important limitation:** The controller assumes a simple mapping from image displacement to body-frame movement. These commands have not been calibrated against camera mounting geometry, perspective, target distance, or actual robot response. A ball appearing low in the image may generate negative `vx`; this is a property of the current formula, not necessarily the desired ball-chasing behavior.

### 6.4 Stop behavior and limitations

The controller publishes a zero `Twist` for `N`, STOPPED/IDLE (via FSM gating), invalid coordinates, or coordinate timeout. The FPS and communication timing of a live system may vary.

This software behavior is **not a safety-rated emergency stop**. The current serial bridge and Teensy receiver do not establish a complete hardware communications-failure stop mechanism. Physical motor integration must include a Teensy-side command watchdog and independent emergency-stop provisions.

## 7. Teensy 4.1 Serial Communication

The `serial_node` subscribes to `/cmd_vel` and `/fsm/state`, serializes velocity commands, transmits FSM heartbeat messages, and reads the Teensy receiver's response. Port and baud configuration are isolated from the robot controller.

| Parameter | Current configuration |
|---|---|
| Microcontroller | Teensy 4.1 |
| Serial device used in testing | `/dev/ttyACM0` |
| Baud rate | 115200 |
| Protocol | UTF-8 CSV, newline-terminated |
| ROS 2 velocity TX format | `vx,vy,wz\n` |
| ROS 2 heartbeat TX format | `HB,<STATE>\n` |
| Teensy heartbeat acknowledgement | `HB_RX,<STATE>` |
| Teensy heartbeat timeout notification | `HB_TIMEOUT,IDLE` |
| Teensy RX acknowledgment/debug line | `RX \| vx=... \| vy=... \| wz=...` |
| Hardware/simulation selection | `simulation` parameter of `serial_node` |
| Current launch configuration | Hardware mode (`simulation: False`) |

Example data sent to Teensy:

```text
0.150,-0.150,0.000
```

Example response logged by `serial_node`:

```text
TEENSY RX: RX | vx=0.150 | vy=-0.150 | wz=0.000
```

The Teensy firmware accepts three finite numeric velocity values, echoes their parsed values, and handles FSM heartbeat messages. It **does not drive motor or LED outputs**. Do not open Arduino Serial Monitor on the same port while `serial_node` owns it.

**Port note:** `/dev/ttyACM0` can change after reconnecting or uploading firmware. Verify the port before using hardware mode.

Firmware sketch:

```text
firmware/teensy/psarobust_teensy_serial/psarobust_teensy_serial.ino
```

### 7.1 FSM heartbeat and timeout (V1.4)

No additional ROS 2 node or application topic is required. The existing `serial_node` directly subscribes to `/fsm/state`, while retaining `/cmd_vel` communication unchanged.

| Item | Implemented behavior |
|---|---|
| FSM state source | `/fsm/state` (`std_msgs/msg/String`, published at 10 Hz by `fsm_node`) |
| Valid states | `IDLE`, `STOPPED`, `RUNNING` |
| ROS 2 heartbeat interval | 1 s (1 Hz) |
| FSM freshness limit in `serial_node` | 1.5 s since the last valid state message; stale/missing state suppresses further heartbeat transmissions |
| Teensy heartbeat timeout | 3000 ms since the last received heartbeat |
| Timeout action | Set Teensy firmware's FSM state to `IDLE` and report `HB_TIMEOUT,IDLE` |
| Heartbeat mechanism | USB serial message; **not** an additional ROS 2 topic |

Serial protocol examples:

```text
ROS 2 -> Teensy: HB,IDLE
Teensy -> ROS 2: HB_RX,IDLE
ROS 2 -> Teensy: HB,RUNNING
Teensy -> ROS 2: HB_RX,RUNNING
ROS 2 -> Teensy: HB,STOPPED
Teensy -> ROS 2: HB_RX,STOPPED
[No heartbeat for > 3000 ms]
Teensy -> ROS 2: HB_TIMEOUT,IDLE
```

**Verification completed:** All three FSM heartbeat states were acknowledged by Teensy 4.1. The timeout was confirmed by stopping only `fsm_node` while keeping `serial_node` active, allowing `TEENSY RX: HB_TIMEOUT,IDLE` to be observed in the ROS 2 terminal. Stopping the entire launch also stops the serial reader, so a later one-shot timeout message may not be visible in that terminal.

**Scope limitation:** The timeout currently updates Teensy's stored FSM state; it is **not** a motor shutdown mechanism or a safety-rated watchdog. Motor actuation and physical LED hardware remain unimplemented.

### 7.2 Planned physical LED status (not implemented)

When an LED strip is installed, the intended Teensy-controlled indications are:

| Condition | Planned LED behavior |
|---|---|
| `IDLE` or startup | Red breathing |
| `STOPPED` | Steady red |
| `RUNNING` | Steady white |
| ROS 2/FSM heartbeat lost (timeout) | Red breathing |

No LED output pin, strip driver, wiring, or LED animation has been implemented or tested in V1.4.

## 8. Installation and Usage

### 8.1 Requirements

- Ubuntu 22.04 with ROS 2 Humble.
- Python 3 and ROS 2 `rclpy`.
- OpenCV, NumPy and `cv_bridge`.
- PySerial (required for hardware serial mode).
- `colcon` and, optionally, `rosdep`.
- USB camera (currently `/dev/video0`).
- Teensy 4.1 running the matching receiver firmware for hardware serial testing.

### 8.2 Install dependencies

On a machine with ROS 2 Humble already installed:

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-serial ros-humble-cv-bridge python3-colcon-common-extensions
```

From the workspace root, install ROS dependencies when available:

```bash
cd ~/psarobust_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 8.3 Clone and build

```bash
mkdir -p ~/psarobust_ws
cd ~/psarobust_ws
git clone https://github.com/psarobust08/PSAROBUST-ROS2-MOBILE-ROBOT-FRAMEWORK.git .
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mobile_robot
source install/setup.bash
```

If the workspace already exists, **do not clone into it**; use its existing source tree.

### 8.4 Launch the system

```bash
cd ~/psarobust_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_robot robot.launch.py
```

The current launch file starts seven nodes. Ensure that the USB camera and configured serial device are available. Keep motors disconnected or disabled during software/serial validation.

### 8.5 Open the web dashboard

```text
http://localhost:8080
```

The browser UI displays the detected L/C/R/N region, ball-center pixels, and FSM state, and provides START/STOP buttons. It does not stream camera video in V1.4.

**Network caution:** The HTTP server listens on port 8080 without authentication or TLS in the current version. Use it only in a trusted development environment; do not expose it to the public internet.

### 8.6 Useful verification commands

```bash
ros2 node list
ros2 topic echo /detection
ros2 topic echo /ball/center
ros2 topic echo /fsm/ball_center
ros2 topic echo /cmd_vel
ros2 topic echo /fsm/state --once
```

## 9. Development Roadmap

Roadmap versions are **proposed targets**; future implementations may be rescheduled based on available hardware and testing results.

| Version | Development focus | Status |
|---|---|---|
| V1.2 | Seven-node ROS 2 framework, web control, fixed L/C/R velocity mapping, bidirectional Teensy serial validation | Completed |
| **V1.3** | **Dynamic coordinate-based `vx`/`vy` proportional control, FSM coordinate forwarding and stale-data handling** | **Completed (software/serial testing)** |
| **V1.4** | **FSM heartbeat via USB serial, Teensy acknowledgement, and 3 s heartbeat timeout with IDLE fallback** | **Completed (hardware serial testing)** |
| Future | Physical LED strip indicators controlled by Teensy | Pending hardware |
| Future | YOLO-based detection and/or object tracking | Planned |
| Future | X-drive inverse kinematics and four-wheel velocity generation | Planned |
| Future | Motor driver and encoder integration | Planned |
| Future | Odometry and robot localization | Planned |
| Future | Hardware safety, watchdogs, diagnostics and reliability improvements | Planned |
| Future | Advanced web dashboard: camera view, telemetry and manual control | Planned |
| Future | Integrated autonomous robot behaviors | Planned |

### 9.1 Potential upgrades to existing nodes

| Existing node | Candidate enhancement |
|---|---|
| `camera_node` | CSI camera support, GStreamer/V4L2 tuning, intrinsic calibration |
| `detector_node` | YOLO detection, custom training, object tracking and confidence handling |
| `fsm_node` | Richer operating states and autonomous behavior management |
| `robot_controller_node` | Camera-calibrated control, X-drive kinematics, motion limits and feedback-based control |
| `status_node` | Battery, motor, CPU, temperature and communication-health reporting |
| `web_node` | Video preview, telemetry, manual motion controls and access control |
| `serial_node` | Command timeout handling, robust reconnection, protocol validation and hardware feedback |

### 9.2 Possible additional nodes (not part of V1.4)

| Proposed node | Possible role |
|---|---|
| `odometry_node` | Estimate pose from wheel encoder data |
| `localization_node` | Combine odometry with external sensing |
| `navigation_node` | Plan or generate higher-level motion toward a goal |
| `safety_node` | Coordinate safety-related monitoring and stop requests (not a replacement for hardware E-stop) |
| `data_logger_node` | Record ROS 2 events and experimental telemetry |

These are **options for later design decisions**, not implemented nodes or required additions.

## 10. Project Structure

```text
psarobust_ws/
├── .gitignore
├── README.md
├── firmware/
│   └── teensy/
│       └── psarobust_teensy_serial/
│           └── psarobust_teensy_serial.ino
└── src/
    └── mobile_robot/
        ├── launch/
        │   └── robot.launch.py
        ├── mobile_robot/
        │   ├── __init__.py
        │   ├── camera_node.py
        │   ├── detector_node.py
        │   ├── fsm_node.py
        │   ├── robot_controller_node.py
        │   ├── serial_node.py
        │   ├── status_node.py
        │   └── web_node.py
        ├── resource/
        │   └── mobile_robot
        ├── test/
        ├── package.xml
        ├── setup.cfg
        └── setup.py
```

Generated workspace directories (`build/`, `install/`, and `log/`) are excluded from version control by `.gitignore`.

## 11. Development Team

**PSA ROBUST**  
Politeknik Sultan Salahuddin Abdul Aziz Shah (PSA), Malaysia.

---

**PSA ROBUST — Modular Robotics Development with ROS 2**
