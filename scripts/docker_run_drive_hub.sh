#!/usr/bin/env bash

RED='\033[0;31m'
RED_BOLD='\033[1;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color
BROWN='\033[0;33m'
CYAN_BOLD='\033[1;36m'
# ==================================

docker_image=drive_hub
echo -e "${RED}Launching docker image ${RED_BOLD}$docker_image"${NC}
echo $PWD

echo -e "${GREEN}Setting X11 permissions...${NC}"
xhost +local:docker

#  -v /home/nvidia/git/ros2_workspace:/workspaces/ros2_workspace \
run_docker_cmd="docker run \
  --name drive_hub \
  --net=host \
  --pid=host \
  --ipc=host \
  --rm \
  -it \
  -v $PWD/:/workspaces/ros2_workspace \
  -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
  -v /tmp/argus_socket:/tmp/argus_socket \
  --cap-add SYS_PTRACE \
  --privileged \
  --cap-add=CAP_SYS_RESOURCE \
  -e ROS_DOMAIN_ID=1 \
  -e PYTHONOPTIMIZE=2 \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/xdg \
  "
run_docker_cmd+=$docker_image 
echo -e "${CYAN_BOLD}$run_docker_cmd${NC}"
eval "$run_docker_cmd"