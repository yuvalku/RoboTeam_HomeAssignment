cmake_minimum_required(VERSION 3.8)
project(interfaces)

# Add compile options if using GNU or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament_cmake package
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Declare the message files to be processed
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/mpc.msg"
)

# Install the generated message files
install(DIRECTORY
  msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

# ament package
ament_package()

# If building with testing enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()