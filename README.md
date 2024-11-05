# ROS2
## Description
ROS2 infrastructure implementation 
It implements an integration of ROS2 with CANOpenNode

## Docker build and run
cd ~/ROS2
build (if not built already) ros2canopen docker: 
     sudo ./scripts/docker_build_ros2can.sh
run:
     sudo ./scripts/docker_run_ros2can.sh

build (if not built already) drive_hub docker: 
     sudo ./scripts/docker_build_drive_hub.sh
run:
     sudo ./scripts/docker_run_drive_hub.sh


CANBUS for ROOK:
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
