#!/bin/bash

# Enable CAN interface (Advantech tool)
sudo bash /opt/advantech/tools/enable_can.sh

# Reconfigure can0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
