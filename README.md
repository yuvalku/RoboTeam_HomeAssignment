# Health Monitor System (ROS 2)

## Summary
This project delivers a complete **ROS 2 health monitoring framework** that evaluates the robot’s operational state and classifies it as **HEALTHY**, **WARNING**, or **CRITICAL**.  
It integrates with CAN-bus data via `ros2can` and provides automatic alerting, logging, and a retrievable health history.

---

## 🚀 Key Components

### Health Analyzer Node  
- Subscribes to `/robot_status` (`interfaces/msg/RobotStatus`)  
- Publishes to `/health_status` (`std_msgs/String`)  
- Evaluates robot parameters (RPMs, battery charge, and BIT flags) every 0.2 s  

### Alert Manager Node  
- Listens to `/health_status`  
- Logs messages with proper severity (`INFO`, `WARN`, `ERROR`)  
- Prevents log flooding with a 10-second cooldown between identical alerts  

### Health History Service  
- Stores the most recent 50 health messages in memory  
- Provides a service `/get_health_history` (`interfaces/srv/GetHealthHistory`)  
  to retrieve the latest *N* entries  

### Launch Integration  
- Unified launch file starts all core nodes and the CAN-bus simulation  
- Supports adjustable parameters and clean shutdown  

---

## ⚙️ Classification Logic

| State | Criteria |
|-------|-----------|
| **HEALTHY** | RPMs within limits, battery > 25 %, no BIT errors |
| **WARNING** | 15 % ≤ battery ≤ 25 % OR RPM out of range OR BIT error |
| **CRITICAL** | battery < 15 % OR both motors stopped > 5 s |

All thresholds are configurable through ROS 2 parameters at runtime.

---

## 🧩 Extended Features
- **Configurable Thresholds:** tune battery and RPM limits per robot type  
- **Service API:** request recent health data via `/get_health_history`  
- **CAN Emergency Stop Detection:** monitors CAN IDs 0x103 and 0x104  
- **Real-Time Operation:** runs continuously at 5 Hz  

---

## 🛠️ Build and Run
```bash
# Build
cd RoboTeam_HomeAssignment/
colcon build --packages-select health_monitor
source install/setup.bash

# Launch the system
ros2 launch health_monitor health_monitor.launch.py
