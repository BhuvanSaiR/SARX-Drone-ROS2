# Search And Rescue eXpedition (SARX) Autonomous Disaster Management Drone

An autonomous ROS2-based drone software stack designed for **Search And Rescue eXpedition (SARX)**, **human detection** and **payload delivery** in disaster management scenarios.

The system enables the UAV to search a predefined area, detect survivors, deliver emergency supplies, and safely resume its mission.

---

## Project Overview

This project implements a complete onboard autonomy stack for SARX.

During a mission, the drone:

1. Receives a predefined search area.
2. Flies autonomously through the search area.
3. Detects humans using a custom model.
4. Switches between front and downward cameras.
5. Centers above the detected survivor.
6. Delivers an emergency payload.
7. Returns to the mission path.
8. Continues searching until the mission is complete.

The system also incorporates multiple layers of safety including command watchdogs, GPS monitoring, geofencing, kill switch handling, and automatic Return-to-Route (RTR) behavior.

---

# Features

## Autonomous Navigation

- GPS waypoint navigation
- Search pattern execution
- Automatic waypoint progression
- Checkpoint saving
- Resume mission after payload delivery

---

## Computer Vision

- Dual camera support
- Front search camera
- Bottom precision camera
- Custom YOLOv5 human detector
- Largest target selection
- Target localization
- Camera switching

---

## Mission State Machine

Implemented mission states:

```
SEARCHING
    ↓
APPROACHING
    ↓
CENTERING
    ↓
DESCENDING
    ↓
PAYLOAD DROP
    ↓
RETURNING
    ↓
SEARCHING
```

---

## PID-Based Target Centering

- Horizontal correction
- Vertical correction
- Normalized image coordinates
- Smooth positioning

---

## Payload Delivery

- Automatic payload release
- Configurable drop conditions
- Return to checkpoint after delivery

---

## Failsafe System

Multiple independent safety layers:

- Kill switch
- GPS timeout detection
- Command watchdog
- Geofence monitoring
- Automatic hover
- Automatic Return-to-Route
- Nearest waypoint recovery

---

## Geofencing

The operational boundary is automatically generated from the uploaded mission waypoints.

If the UAV exits the geofence:

- Detects breach
- Computes nearest waypoint
- Commands the UAV back into the mission area

---

## Command Watchdog

If command updates stop:

- Hover after 0.5 seconds
- Return to nearest waypoint after 2 seconds

---

## Modular ROS2 Architecture

Each subsystem operates independently.

```
Camera Node
      │
      ▼
Detection Node
      │
      ▼
Perception Node
      │
      ▼
Mission Node
      │
      ▼
Failsafe Node
      │
      ▼
Control Node
      │
      ▼
MAVSDK
      │
      ▼
Pixhawk / ArduPilot
```

---

# ROS2 Nodes

## Camera Node

Responsibilities:

- Capture images
- Publish front camera
- Publish bottom camera

Topics

Published

```
/camera/front
/camera/bottom
```

---

## Detection Node

Responsibilities

- Run YOLO inference
- Detect humans
- Compute image center
- Estimate target area

Published

```
/detection/front
/detection/bottom
```

---

## Perception Node

Responsibilities

- Select active camera
- Refine detections
- Decide approach condition

Published

```
/perception/target
```

Message format

```
[
target_detected,
approach_ready,
cx,
cy,
area
]
```

---

## Mission Node

Responsibilities

- Execute state machine
- Follow waypoints
- Camera switching
- Target approach
- Payload logic
- Mission resume

Published

```
/command_raw
/goto_gps
/drop
/active_camera
```

---

## Failsafe Node

Responsibilities

- Safety supervision
- GPS monitoring
- Command monitoring
- Geofence enforcement
- Return-to-route
- Kill switch

Published

```
/command
/goto_gps
```

---

## Control Node

Responsibilities

- MAVSDK interface
- Offboard control
- Velocity commands
- GPS publishing
- Flight monitoring

Published

```
/gps
```

Subscribed

```
/command
```

---

# Technologies Used

- ROS2
- Python
- MAVSDK
- ArduPilot
- Pixhawk Cube Orange
- YOLOv5
- OpenCV
- PyTorch
- Picamera2
- MAVLink
- Gazebo
- Ubuntu

---

# Hardware

- Pixhawk Cube Orange
- Raspberry Pi 5
- Rsapberry Pi cam 
- GPS Module
- Telemetry Radio
- Servo Payload Mechanism

---

# Simulation

The project is designed to be tested using:

- ArduPilot SITL
- Gazebo
- MAVSDK
- ROS2

Testing includes:

- Waypoint following
- Human detection
- Payload delivery
- Geofence violations
- Command timeout
- GPS failure
- Mission recovery

---
