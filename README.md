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
     `sudo ./scripts/docker_run_ros2can.sh`

3. **Build the drive_hub Docker Image** (if not already built): 
     `sudo ./scripts/docker_build_drive_hub.sh`
run:
     sudo ./scripts/docker_run_drive_hub.sh


## CANBUS for ROOK:
     to activate canbus on advantech platform run:
          sudo bash /opt/advantech/tools/enable_can.sh
     test canbus:
          cd /workspace/ROS2/scripts
          ./can_test.sh
          (if the engines start to run all is good)
     Canbus ID's:
          left motor 0x203
          right motor 0x204
     packet structure:
          <ID> <CMD> <Data[0] = speed, Data[1] = direction>
          drive forward example command (0x203,[0x01, 0x5E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]) , 
                                        (0x204,[0x01, 0xA2, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00]) 
