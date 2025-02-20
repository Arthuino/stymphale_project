cmake_minimum_required(VERSION 3.8)
project(antikythera_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require C++17 (important for ROS2 and compatibility)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(pcl_conversions REQUIRED)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME} 
  "msg/LandMarkFeature.msg" 
  "msg/LandMarkObject.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs
)


# Ensure runtime dependencies are correctly exported
ament_export_dependencies(rosidl_default_runtime)


# Required for message packages
ament_package()
