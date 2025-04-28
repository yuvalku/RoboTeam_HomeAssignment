#!/usr/bin/env bash

# Color formatting
RED='\033[0;31m'
RED_BOLD='\033[1;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color
CYAN_BOLD='\033[1;36m'

# Docker image name
docker_image=video_hub
container_name=video_hub_container

echo -e "${RED}Launching docker image ${RED_BOLD}$docker_image${NC}"
echo $PWD

# Set X11 permissions for GUI applications
echo -e "${GREEN}Setting X11 permissions...${NC}"
xhost +local:docker

# Docker run command
run_docker_cmd="docker run \
  --name $container_name \
  --net=host \
  --pid=host \
  --ipc=host \
  --rm \
  -it \
  --runtime nvidia \
  -v $PWD:/workspaces/ros2_workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /tmp/argus_socket:/tmp/argus_socket \
  --cap-add=SYS_PTRACE \
  --privileged \
  --cap-add=CAP_SYS_RESOURCE \
  -e ROS_DOMAIN_ID=1 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e PYTHONOPTIMIZE=2 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/xdg \
  $docker_image \
  bash -c ' \
    echo -e \"${GREEN}Installing required Python packages...${NC}\" && \
    pip3 install \"numpy<2\" && \
    sudo apt update && \
    sudo apt install -y python3-gi python3-gi-cairo && \
    echo -e \"${GREEN}Environment is ready!${NC}\" && \
    rm -rf src/rook_description/ src/rook_canopen &&\
    exec bash'"
  
# Print and execute the Docker run command
echo -e "${CYAN_BOLD}$run_docker_cmd${NC}"
eval "$run_docker_cmd"
