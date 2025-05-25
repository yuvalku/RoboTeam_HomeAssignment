# ROS2

## Description

ROS 2 infrastructure implementation for robotic platforms. This project integrates ROS 2 with CANOpenNode and includes modular, containerized nodes for control, perception, and operator interfaces.

---

## System Architecture

### Platform Side (Jetson Orin NX)

This device runs two main Docker containers:

* **`drive_hub`**: Handles driving, manipulation, LEDs, tilt cameras, and runs the MPC controller. It interfaces with `ros2can_bridge` for CAN bus communication. (Planned to be renamed to `operation_hub`.)

* **`video_hub`**: Manages IP camera connections using GStreamer, processes frames, and publishes them on the `video_topic`. It dynamically receives the camera IP from the operator via the `ip_service`.

* **`ros2can_bridge`**: Bridges ROS 2 and the CAN bus, translating ROS messages to CAN commands and vice versa. It publishes data on `status_topic`.

### Operator Side

* **`operator_hub`**: Provides the interface between the user (e.g., via AiCU in C#) and the ROS 2 system. It communicates using a message queue internally and ROS 2 topics externally (`operation_cmd_topic`).

---

## Communication Topology

* ROS 2 topics (`vel_cmd_topic`, `manip_cmd_topic`, `status_topic`, `video_topic`, `operation_cmd_topic`)
* ROS 2 service (`ip_service`)
* DDS for intra-ROS communication, Ethernet for inter-machine, and CAN bus for actuator/sensor interface

---

## Environment Setup

### System Requirements

* Ubuntu 22.04 or 24.04 LTS
* Two machines: Jetson Orin NX (platform) and Ubuntu operator PC

### Docker & Workspace Setup

1. **Install Docker and Docker Compose** ([Docker Docs](https://docs.docker.com/engine/install/ubuntu/))

2. **Clone the project on both machines**:

   ```bash
   mkdir /workspace
   cd /workspace
   git clone https://github.com/roboteam-software/ROS2.git
   ```

3. **Build Docker Images**

   On **Jetson**:

   ```bash
   cd /workspace/ROS2
   ./script/docker_build_drive_hub.sh
   ./script/docker_build_video_hub.sh
   ```

   On **Operator PC**:

   ```bash
   cd /workspace/ROS2
   ./script/docker_build_operator.sh
   ```

4. **Run Docker Containers** (in separate terminals):

   On **Jetson**:

   ```bash
   ./script/docker_run_drive_hub.sh
   ./script/docker_run_video_hub.sh
   ```

   On **Operator PC**:

   ```bash
   ./script/docker_run_operator.sh
   ```

5. **Build ROS 2 Packages (inside each container):**

   **drive\_hub:**

   ```bash
   colcon build --symlink-install --packages-select drive_hub interfaces launcher ros2can tester watchdog
   ```

   **video\_hub:**

   ```bash
   colcon build --symlink-install --packages-select video_hub
   ```

   **operator\_hub:**

   ```bash
   colcon build --symlink-install --packages-select operators_hub interfaces
   ```

6. **Alternative: Use Docker Compose (Jetson only)**

   ```bash
   cd /workspace/ROS2
   docker-compose -f docker-compose_drive_hub.yml up
   ```

---

## CANBUS:

### Activate CAN bus on the Advantech Platform

```bash
cd /workspace/ROS2
./script/start_canbus.sh
```



