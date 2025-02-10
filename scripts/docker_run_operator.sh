#!/usr/bin/env bash

RED='\033[0;31m'
RED_BOLD='\033[1;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color
BROWN='\033[0;33m'
CYAN_BOLD='\033[1;36m'
# ==================================

docker_image=operator
echo -e "${RED}Launching docker image ${RED_BOLD}$docker_image"${NC}
echo $PWD

echo -e "${GREEN}Setting X11 permissions...${NC}"
xhost +local:docker

#  -v /home/nvidia/git/ros2_workspace:/workspaces/ros2_workspace \
run_docker_cmd="docker run \
  --name operator \
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
  --device=/dev/input/js0 \
  -e ROS_DOMAIN_ID=1 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e PYTHONOPTIMIZE=2 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/xdg \
   $docker_image \
  /bin/bash -c \"pip install python-can && \
  python3 -m pip install --upgrade pip && \
  pip install black==21.12b0 uvloop==0.17.0 && \
  pip install --upgrade typing-extensions && \
  rm -rf src/drive_hub/ src/ros2can/ src/tester/ src/rook_description/ &&\
  /bin/bash\""
  
echo -e "${CYAN_BOLD}$run_docker_cmd${NC}"
eval "$run_docker_cmd"