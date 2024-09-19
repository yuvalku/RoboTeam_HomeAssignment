#!/bin/bash
# Clone repositories and build the workspace

# Set the workspace directory
WORKSPACE=/workspaces/ros2_workspace

# Ensure the src directory exists
mkdir -p ${WORKSPACE}/src

# Clone repositories
cd ${WORKSPACE}/src
git clone https://github.com/lucasmluza/ros2_moveit_canopen_example.git
git clone https://github.com/CANopenNode/CANopenNode.git
cd CANopenNode
git submodule update --init --recursive

# Source ROS2 setup and build the workspace
source /opt/ros/humble/setup.bash
cd ${WORKSPACE}
colcon build --symlink-install

# Start a bash session after cloning and building
exec bash