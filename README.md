# 🤖 Sensor Network for Quadruped Platform

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20%7C%2024.04-orange)](https://ubuntu.com/)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://python.org)
[![License](https://img.shields.io/badge/License-Academic-lightgrey)]()

**Authors:** Anderson Sneider Del Castillo Criollo · Diego Alejandro Hernandez Losada  

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Usage](#usage)
- [ROS2 Nodes](#ros2-nodes)
- [Results Summary](#results-summary)
- [Demo](#demo)
- [License & Citation](#license--citation)

---

## Overview

This project implements a distributed sensor network for the quadruped robot platform at the Universidad Autónoma de Occidente (UAO). The system integrates multiple heterogeneous sensors under a unified **ROS2 Humble** architecture running on a **Raspberry Pi 5**, enabling real-time environmental perception, robot state monitoring, and a graphical user interface for data visualization.

The communication backbone is built on the **CAN Bus** protocol (via Arduino Nano + MCP2515 modules), chosen for its robustness against electromagnetic interference, real-time error detection, and ability to support multiple nodes on a shared bus — critical for embedded robotics applications.

**Sensors integrated:**

| Sensor | Type | Interface to MCU | Purpose |
|--------|------|-----------------|---------|
| MPU9250 | 9-DOF IMU | I2C → SPI (MCP2515) → CAN | Orientation (Roll/Pitch/Yaw), acceleration |
| ACS712 | Current | Analog → CAN | Power consumption monitoring |
| FZ0430 | Voltage | Analog → CAN | Battery state monitoring |
| TTP223 × 4 | Capacitive tactile | Digital → CAN | Foot-ground contact (one per leg) |
| MQ-7 | Gas | Analog → CAN | Environmental gas concentration |
| GPS module | GNSS | UART (serial) | Global positioning |
| USB Camera | RGB Vision | USB | Object detection via YOLOv5 |
| RPLiDAR 2D | LiDAR | UART/USB | Point cloud scanning for RViz |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Sensor Network (CAN Bus)                    │
│                                                                 │
│  ┌──────────────────────────┐  ┌─────────────────────────────┐  │
│  │          PCB 1           │  │           PCB 2             │  │
│  │  Arduino Nano + MCP2515  │  │  Arduino Nano (×2)          │  │
│  │  ──────────────────────  │  │  + MCP2515 (×2)             │  │
│  │  MPU9250  (IMU,    I2C)  │  │  ───────────────────────    │  │
│  │  MQ-7     (gas,  analog) │  │  ACS712 (current, analog)  │  │
│  │  TTP223×4 (touch, digit) │  │  FZ0430 (voltage, analog)  │  │
│  └────────────┬─────────────┘  └──────────────┬──────────────┘  │
│               │                               │                 │
│               └───────────────┬───────────────┘                 │
│                          CAN Bus                                │
│                    ┌─────┴──────┐                               │
│                    │ Distributor│  ← Power + CAN_H/CAN_L lines  │
│                    │   PCB      │    (141×121 mm)               │
│                    └─────┬──────┘                               │
└──────────────────────────┼──────────────────────────────────────┘
                           │ UART (USB-Serial)
┌──────────────────────────▼──────────────────────────────────────┐
│         Raspberry Pi 5 — Ubuntu 24.04 — ROS2 Humble             │
│                                                                 │
│  can_node ──────────────► serial_data ──► serial_data_processor │
│  gps_node ──────────────► gps/fix                               │
│  camera_node ───────────► video_detections  (YOLOv5)            │
│  rplidar_composition ───► scan                                  │
│  audio_publisher ───────► audio_text                            │
│                                                                 │
│  interfaz_suscriber ◄── [all topics] ──► Monitoring GUI         │
│  stl_node ◄──────────── serial_data  ──► 3D STL orientation     │
└─────────────────────────────────────────────────────────────────┘
```

See [`docs/system_overview.md`](docs/system_overview.md) for full subsystem documentation.

---

## Hardware Requirements

| Component | Model | Qty | Notes |
|-----------|-------|-----|-------|
| Single-board computer | Raspberry Pi 5 | 1 | Ubuntu 24.04, ROS2 central node |
| Microcontrollers | Arduino Nano | 3 | CAN sensor acquisition nodes |
| CAN transceiver modules | MCP2515 | 3 | SPI-to-CAN, one per Arduino |
| IMU | MPU9250 (9-DOF) | 1 | Accel + gyro + mag, I2C |
| Gas sensor | MQ-7 | 1 | Analog output |
| Capacitive tactile sensors | TTP223 | 4 | Digital, one per leg |
| Current sensor | ACS712 | 1 | Analog, ±5A or ±20A variant |
| Voltage sensor | FZ0430 | 1 | Analog |
| GPS module | NMEA 0183 (e.g., Neo-6M) | 1 | UART |
| Camera | USB RGB camera | 1 | YOLOv5 inference |
| 2D LiDAR | RPLiDAR (A1/A2) | 1 | RViz point cloud |
| Custom PCBs | KiCad (double-layer) | 3 | PCB1: 70×90 mm · PCB2: 122.5×60 mm · Distributor: 141×121 mm |
| Development workstation | PC, Ubuntu 22.04 | 1 | ROS2 Humble, visualization |

See [`hardware/bill_of_materials.md`](hardware/bill_of_materials.md) for full component list.

---

## Software Requirements

| Dependency | Version | Notes |
|------------|---------|-------|
| Ubuntu | 22.04 LTS (PC) / 24.04 (RPi 5) | |
| ROS2 | Humble Hawksbill | [Installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) |
| Python | 3.10+ | |
| OpenCV | 4.x | `pip install opencv-python` |
| Open3D | 0.17+ | `pip install open3d` |
| PyTorch | 2.x | `pip install torch torchvision` |
| python-can | 4.x | `pip install python-can` |
| pyserial | 3.x | `pip install pyserial` |
| rplidar_ros | Humble | `sudo apt install ros-humble-rplidar-ros` |

---

## Repository Structure

```
robot-quadruped-sensor-network/
│
├── README.md                         ← You are here
│
├── docs/
│   ├── architecture.png              ← Full system architecture diagram
│   ├── can_bus_diagram.png           ← CAN Bus wiring (MCP2515 + termination)
│   └── system_overview.md            ← Detailed subsystem documentation
│
├── ros2_ws/
│   ├── src/
│   │   └── tesis_launch/
│   │       ├── package.xml           ← ROS2 package manifest
│   │       ├── setup.py              ← Node entry points
│   │       ├── setup.cfg
│   │       ├── launch/               ← .launch.py files
│   │       ├── tesis_launch/         ← Python nodes & resources
│   │       │   ├── can_node.py            → CAN Bus → serial_data
│   │       │   ├── gps.py                 → GPS → gps/fix
│   │       │   ├── camarafiltro.py        → Camera + YOLOv5
│   │       │   ├── image_publisher.py     → Raw image publisher
│   │       │   ├── image_sus.py           → Image subscriber
│   │       │   ├── interfaz_open3d.py     → Main monitoring GUI
│   │       │   ├── rviz_launcher_node.py  → RViz2 launcher
│   │       │   ├── comandos_pub.py        → Motion commands publisher
│   │       │   ├── comandos_sus.py        → Commands → CAN bridge
│   │       │   ├── prueba_stl.py          → 3D STL viewer (VTK)
│   │       │   ├── prueba_stl_sinvtk.py   → 3D STL viewer (no VTK)
│   │       │   ├── POCHITA_v30.stl        → Robot 3D model
│   │       │   ├── yolov5nu.pt            → YOLOv5-nano weights
│   │       │   └── rviz/                  → RViz2 configuration files
│   │       └── test/
│   ├── README.md                     ← Build & run instructions
│   └── .gitignore
│
├── hardware/
│   ├── schematics/                   ← KiCad PCB files and PDF exports
│   └── bill_of_materials.md          ← Component list with specs
│
└── demo/
    ├── images/                       ← GUI screenshots
    └── video_link.md                 ← Demo video links
```

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/diego2704/robot-quadruped-sensor-network.git
cd robot-quadruped-sensor-network
```

### 2. Install ROS2 Humble

Follow the [official ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), then:

```bash
source /opt/ros/humble/setup.bash
```

### 3. Install Python dependencies

```bash
pip install opencv-python open3d python-can pyserial torch torchvision
```

### 4. Install ROS2 package dependencies

```bash
sudo apt install ros-humble-rplidar-ros
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### 6. Initialize the CAN Bus interface

Before running `can_node`, the SocketCAN interface must be configured:

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
ip link show can0    # Verify the interface is UP
```

> Tip: add this to `/etc/rc.local` or a systemd service to bring up `can0` on boot.

---

## Usage

### Launch the full system

```bash
cd ros2_ws && source install/setup.bash
ros2 launch tesis_launch [main_launch_file.launch.py]
```

### Run individual nodes

```bash
# CAN Bus data publisher (requires can0 initialized)
ros2 run tesis_launch can_node

# GPS publisher
ros2 run tesis_launch gps

# Camera + YOLOv5 object detection
ros2 run tesis_launch camarafiltro

# Main monitoring GUI (Open3D interface)
ros2 run tesis_launch interfaz_open3d

# RViz2 with LiDAR visualization
ros2 run tesis_launch rviz_launcher_node

# 3D STL orientation viewer
ros2 run tesis_launch prueba_stl_sinvtk
```

### Inspect the ROS2 graph

```bash
ros2 topic list
ros2 topic echo /serial_data
ros2 topic echo /gps/fix
rqt_graph    # View the full node/topic graph
```

---

## ROS2 Nodes

| Node | File | Topic | Type | Function |
|------|------|-------|------|----------|
| `can_node` | `can_node.py` | `serial_data` | Publisher | Reads CAN Bus, publishes raw sensor data |
| `serial_data_processor` | — | `serial_data` | Subscriber | Decodes IMU, gas, tactile, current, voltage |
| `gps_node` | `gps.py` | `gps/fix` | Publisher | Parses NMEA, publishes GPS coordinates |
| `gps_subscriber` | — | `gps/fix` | Subscriber | Renders GPS position on map |
| `camera_node` | `camarafiltro.py` | `video_detections` | Publisher | USB camera capture + YOLOv5 inference |
| `image_publisher` | `image_publisher.py` | `/camera/raw` | Publisher | Raw image stream |
| `image_sus` | `image_sus.py` | `/camera/raw` | Subscriber | Image monitoring |
| `audio_publisher` | — | `audio_text` | Publisher | Speech-to-text voice commands |
| `interfaz_suscriber` | `interfaz_open3d.py` | multiple | Subscriber | Central monitoring GUI |
| `rplidar_composition` | — | `scan` | Publisher | 2D LiDAR point cloud for RViz |
| `rviz_launcher_node` | `rviz_launcher_node.py` | — | — | Launches RViz2 with preconfigured layout |
| `stl_node` | `prueba_stl.py` | `serial_data` | Subscriber | Live 3D orientation of STL model |
| `transform_listener_impl` | — | `tf` | Subscriber | Spatial transform support |
| `comandos_pub` | `comandos_pub.py` | `/commands` | Publisher | Motion command publisher |
| `comandos_sus` | `comandos_sus.py` | `/commands` | Subscriber | Commands → CAN Bus bridge |

---

## Results Summary

Key results from the validation campaign:

### IMU (MPU9250) — Bland-Altman method vs. goniometer

| Axis | R² | Assessment |
|------|----|-----------|
| Roll | 0.962 | Within acceptance limits |
| Pitch | 0.966 | Within acceptance limits |
| Yaw | 0.995 | Excellent agreement |

→ The IMU showed small mean errors and high correlation, validating its use for orientation estimation.

### Current sensor (ACS712)

Average error margin: **5%** → validated for sensor network integration.

### CAN Bus communication (3 transmitters → 1 receiver)

| Metric | Transmitters | Receiver |
|--------|-------------|---------|
| Average latency | 491 µs | 498 µs |
| Throughput | 2035.71 msg/s | 2005.86 msg/s |

→ Stable, low-latency communication confirmed across all CAN nodes.

### Usability test (Likert scale, 0–100)

Score > 68 → monitoring interface rated **acceptable** by evaluators.

---

## Demo

The [`demo/`](demo/) folder contains screenshots of the monitoring GUI, which provides:
- Live IMU values (Roll, Pitch, Yaw) with real-time 3D STL model orientation
- Gas (MQ-7), voltage (FZ0430), and current (ACS712) readings
- Foot contact status indicator (4 × TTP223 tactile sensors)
- Voice command transcription panel
- Pop-up windows for Camera (YOLOv5 detections), GPS map, and LiDAR (RViz)

---

## Contact

Feel free to reach out for questions, collaborations, or academic inquiries:

| | Anderson Sneider Del Castillo Criollo | Diego Alejandro Hernandez Losada |
|---|---|---|
| **GitHub** | [@delcri](https://github.com/delcri) | [@diego2704](https://github.com/diego2704) |
| **Email** | delcast2210@gmail.com | hernandezdiegoalejandro35@gmail.com |
| **LinkedIn** | [anderson-sneider-del-castillo](https://www.linkedin.com/in/anderson-sneider-del-castillo-criollo-12b987297) | [diego-hernandez](https://www.linkedin.com/in/diego-hernandez-1827ab256) |

---

## License & Citation

© 2025 Anderson Sneider Del Castillo Criollo & Diego Alejandro Hernandez Losada. All rights reserved.

This project was developed as a thesis at Universidad Autónoma de Occidente. For academic or research use, please cite the authors:

```
A. S. Del Castillo Criollo and D. A. Hernandez Losada,
"Implementación de una Red de Sensores para Labores de Investigación
Aplicada sobre la Plataforma Cuadrúpeda Existente en la UAO,"
B.Eng. thesis, Univ. Autónoma de Occidente, Cali, Colombia, 2025.
```
