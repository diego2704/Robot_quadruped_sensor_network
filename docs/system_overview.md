# System Overview — Sensor Network for POCHITA Quadruped

This document provides a technical description of each subsystem in the sensor network implementation. The system runs on a **Raspberry Pi 5** (Ubuntu 24.04 + ROS2 Humble) as the central node, with three **Arduino Nano + MCP2515** modules forming the distributed CAN Bus sensor layer.

---

## Table of Contents

- [Hardware Architecture](#hardware-architecture)
- [PCB Design](#pcb-design)
- [Communication Strategy](#communication-strategy)
- [ROS2 Software Architecture](#ros2-software-architecture)
- [Monitoring GUI](#monitoring-gui)
- [Sensor Subsystems](#sensor-subsystems)
- [ROS2 Topic Map](#ros2-topic-map)

---

## Hardware Architecture

The system uses a **distributed CAN Bus architecture**, chosen for its:
- Resistance to electromagnetic interference (critical inside a moving robot)
- Built-in collision detection and message prioritization
- Scalability: adding new sensor nodes does not require changes to existing nodes

The physical layout has three layers:

1. **Sensor nodes (PCB1, PCB2):** Arduino Nano microcontrollers acquire sensor data and transmit it over CAN via MCP2515 modules.
2. **Distributor PCB:** Physically connects CAN_H and CAN_L lines from all nodes and distributes power to each board.
3. **Central node (Raspberry Pi 5):** An Arduino Nano receiver reads the CAN bus via UART/USB serial and forwards data to the RPi5 running ROS2.

```
Sensors → Arduino Nano (SPI) → MCP2515 → CAN Bus → Arduino Nano (receiver) → UART → RPi5
```

---

## PCB Design

All PCBs were designed with **KiCad** in double-layer configuration to minimize size. Trace width: 0.7 mm throughout.

### PCB 1 — Proprioceptive + Environmental Node
**Dimensions:** 70 × 90 mm

| Component | Qty | Interface |
|-----------|-----|-----------|
| Arduino Nano | 1 | — |
| MCP2515 (CAN module) | 1 | SPI |
| MPU9250 (9-DOF IMU) | 1 | I2C |
| MQ-7 (gas sensor) | 1 | Analog |
| TTP223 (tactile sensors) | 4 | Digital |

This board captures inertial orientation, gas concentration, and foot-contact events. All data is packetized and sent over the CAN bus.

### PCB 2 — Power Monitoring Node
**Dimensions:** 122.5 × 60.428 mm

| Component | Qty | Interface |
|-----------|-----|-----------|
| Arduino Nano | 2 | — |
| MCP2515 (CAN module) | 2 | SPI |
| ACS712 (current sensor) | 1 | Analog |
| FZ0430 (voltage sensor) | 1 | Analog |

One Arduino is dedicated exclusively to power monitoring (ACS712 + FZ0430). The second Arduino acts as a concentrator for additional external sensors.

### Distributor PCB
**Dimensions:** 141 × 121 mm

Connects CAN_H and CAN_L lines from PCB1 and PCB2, and distributes power to both boards via male-female pin connectors. The modular connector design allows each PCB to be detached and tested independently.

---

## Communication Strategy

### Sensor → Arduino (low-level)
Sensors use different electrical interfaces depending on their output type:

| Signal type | Sensors | Arduino interface |
|-------------|---------|------------------|
| Analog | ACS712, FZ0430, MQ-7 | Analog input pins |
| Digital | TTP223 (×4) | Digital input pins |
| I2C | MPU9250 | Wire library (SDA/SCL) |
| — | All above → MCP2515 | SPI → CAN frame |

### Arduino → Raspberry Pi 5 (CAN Bus)
Each Arduino Nano communicates with its MCP2515 module via **SPI**. The MCP2515 handles CAN frame encoding and bus arbitration. A dedicated **receiver Arduino Nano** collects all frames from the bus and forwards them to the Raspberry Pi 5 via **UART (USB-Serial)**.

Key CAN Bus parameters:
- Bitrate: **500,000 bps**
- All nodes operate at the same bitrate to prevent message overlap
- Bus termination: 120 Ω resistors at both ends

### Raspberry Pi 5 → ROS2
The `can_node.py` ROS2 node reads the serial stream from the receiver Arduino and publishes decoded sensor values to the `serial_data` topic. From there, other nodes consume the data independently.

---

## ROS2 Software Architecture

The software follows an **MVC (Model-View-Controller)** pattern:
- **Model:** Sensor data management (`serial_data_processor`, `can_node`)
- **View:** Monitoring GUI and visualization nodes (`interfaz_suscriber`, `stl_node`, `rviz_launcher_node`)
- **Controller:** Command and launch nodes (`comandos_pub`, `comandos_sus`, `audio_publisher`)

The system is organized so that modules are launched **on-demand** from the GUI rather than all at startup, saving resources on the Raspberry Pi.

```
                         ┌───────────────────────────────┐
                         │          ROS2 Topics          │
                         │                               │
 can_node ──────────────►│ serial_data                   │
 gps_node ──────────────►│ gps/fix                       │
 camera_node ───────────►│ video_detections              │
 rplidar_composition ───►│ scan                          │
 audio_publisher ───────►│ audio_text                    │
                         │                               │
                         └───────────────┬───────────────┘
                                         │
                              interfaz_suscriber
                              (main GUI node)
                             ┌─────┬──────┬──────┐
                             │     │      │      │
                          stl_  rviz_ gps_sub camera
                          node  node   node   display
```

---

## Monitoring GUI

Developed with ROS2 and a Python GUI framework. The interface is organized into sections:

**Left panel — IMU:**
- Roll, Pitch, Yaw values (degrees) + rate of change
- "Visualize IMU" button → opens 3D STL model of POCHITA with live orientation

**Right panel — Environmental sensors:**
- Gas concentration (MQ-7)
- Voltage (FZ0430)
- Current (ACS712)
- Tactile sensor status: 4 indicators (one per leg), showing ACTIVE / INACTIVE contact

**Bottom panel — Voice commands:**
- Microphone input → `audio_publisher` node → `/audio_text` topic

**Top menu (hamburger button) — On-demand modules:**
- **Camera:** Opens live camera window with YOLOv5 bounding box overlays
- **Map:** Displays robot's current GPS position on a map
- **Simulation (RViz):** Launches RViz2 with RPLiDAR point cloud

---

## Sensor Subsystems

### IMU — MPU9250
- **Data:** Acceleration (x/y/z), angular velocity (x/y/z), magnetic field (x/y/z)
- **Validation:** Bland-Altman method vs. goniometer → R² = [0.962, 0.966, 0.995] for Roll/Pitch/Yaw
- **ROS2 type:** Custom message via `serial_data`

### Current — ACS712
- **Range:** Depends on model (5A, 20A, or 30A variants)
- **Validation:** Average error margin 5% vs. bench power supply reference
- **Interface:** Analog → Arduino → CAN

### Voltage — FZ0430
- **Purpose:** Battery voltage monitoring
- **Interface:** Analog → Arduino → CAN

### Tactile — TTP223 (×4)
- **Purpose:** Foot-ground contact detection (one sensor per leg)
- **Output:** Digital HIGH/LOW
- **GUI display:** Visual robot diagram with per-leg contact indicators

### Gas — MQ-7
- **Gas detected:** Carbon monoxide (CO)
- **Output:** Analog
- **Interface:** Analog → Arduino → CAN

### GPS
- **Node:** `gps_node` → topic `gps/fix`
- **Protocol:** NMEA 0183 via UART
- **GUI:** Pop-up map window with current coordinates

### Camera + YOLOv5
- **Node:** `camera_node` → topic `video_detections`
- **Model:** YOLOv5-nano (`yolov5nu.pt`, ~5.4 MB)
- **Output:** Live video with bounding boxes and class labels

### LiDAR — RPLiDAR 2D
- **Node:** `rplidar_composition` → topic `scan`
- **Visualization:** RViz2 with 2D point cloud overlay
- **Package:** `rplidar_ros` (ROS2 Humble)

---

## ROS2 Topic Map

| Topic | Message Type | Publisher | Subscriber(s) |
|-------|-------------|-----------|---------------|
| `serial_data` | `std_msgs/String` | `can_node` | `serial_data_processor`, `stl_node` |
| `gps/fix` | `sensor_msgs/NavSatFix` | `gps_node` | `gps_subscriber`, `interfaz_suscriber` |
| `video_detections` | `std_msgs/String` | `camera_node` | `interfaz_suscriber` |
| `scan` | `sensor_msgs/LaserScan` | `rplidar_composition` | RViz2 |
| `audio_text` | `std_msgs/String` | `audio_publisher` | `interfaz_suscriber` |
| `/commands` | `geometry_msgs/Twist` | `comandos_pub` | `comandos_sus` |
| `tf` | `tf2_msgs/TFMessage` | `transform_listener_impl` | visualization nodes |

> Verify actual topic names at runtime with `ros2 topic list` and `rqt_graph`.
