docker run -it --network host \
    -v $(pwd)/:/workspaces/ros2_workspace \
    ros2_humble_canopen
    # # Clone the repositories if they do not already exist
    #     if [ ! -d /workspaces/ros2_workspace/src/ros2_moveit_canopen_example ]; then
    #         git clone https://github.com/lucasmluza/ros2_moveit_canopen_example.git /workspaces/ros2_workspace/src/ros2_moveit_canopen_example
    #     fi
    #     if [ ! -d /workspaces/ros2_workspace/src/CANopenNode ]; then
    #         git clone https://github.com/CANopenNode/CANopenNode.git /workspaces/ros2_workspace/src/CANopenNode
    #         cd /workspaces/ros2_workspace/src/CANopenNode
    #         git submodule update --init --recursive
    #     fi
        
    #     # Optionally build the workspace
    #     cd /workspaces/ros2_workspace
    #     source /opt/ros/humble/setup.bash
    #     colcon build
    #     exec bash
    # "