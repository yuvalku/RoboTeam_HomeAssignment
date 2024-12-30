# ROS2
## Description
ROS2 infrastructure implementation 
It implements an integration of ROS2 with CANOpenNode

## Docker build and run
1. **Clone / Navigate**
```bash
cd ~/ROS2
```
2. **Build the ros2canopen Docker Image** (if not already built)
```bash
sudo ./scripts/docker_build_ros2can.sh
```
   
   **Run the ros2canopen Docker Container**
```bash
sudo ./scripts/docker_run_ros2can.sh
```

3. **Build the drive_hub Docker Image** (if not already built): 
```bash
sudo ./scripts/docker_build_drive_hub.sh
```
   **Run the drive_hub Docker Container**
  ```bash
  sudo ./scripts/docker_run_drive_hub.sh
  ```

## CANBUS for ROOK:
1. **Activate CAN bus on the Advantech Platform**
```bash
sudo bash /opt/advantech/tools/enable_can.sh
```
2. **Test the CAN bus**
```bash
cd /workspace/ROS2/scripts
./can_test.sh
```
   If the motors start to run, the CAN bus setup is working correctly.

## CAN bus Commands and Reports
### Drive Commands
1. **Left Motor: Set Speed Command**
```text
   ID: 0x203
   Data: 0x01, <speed> (signed short value)
```
3. **Right Motor: Set Speed Command**
   ```text
   ID: 0x204
   Data: 0x01, <speed> (signed short value)
   ```
Example:
   -Left motor forward command:
   ```text
   ID: 0x203
   Data: 0x01, 0x5E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
   ```
   -Right motor forward command:
   ```text
   ID: 0x204
   Data: 0x01, 0xA2, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00
   ```
**In these examples, 0x5E, 0x00 (for the left motor) and 0xA2, 0xFF (for the right motor) represent signed short speed values in little-endian format**

### Reports
1. **Rotation/Load Report**
   -**Left Motor**
   ```text
   ID: 0x303
   Data: 0x03, <rpm> (short), <load> (short)
   ```
   -**Right Motor**
   ```text
   ID: 0x304
   Data: 0x03, <rpm> (short), <load> (short)
   ```
2. **IMU**
   ```text
   ID: 0x302
   Data: 0x0C, <rollDeg> (ushort), <pitchDeg> (ushort), <yawDeg> (ushort)
   ```
4. **Battery**
   ```text
   ID: 0x309
   Data: 0x07, <batteryID>, <current> (short), <voltage> (short), <charge> (byte)
   ```
