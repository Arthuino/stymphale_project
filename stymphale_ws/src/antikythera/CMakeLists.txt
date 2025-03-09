cmake_minimum_required(VERSION 3.8)
project(antikythera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# FINDING PACKAGES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(antikythera_msgs REQUIRED)

# INCLUDE DIRECTORIES
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${pcl_conversions_INCLUDE_DIRS})
include_directories(${pcl_ros_INCLUDE_DIRS})
include_directories(${antikythera_msgs_INCLUDE_DIRS})

# LINK DIRECTORIES
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# VARIABLES
set(ANTIKYTHERA_LIBS ${OpenCV_LIBRARIES} ${Eigen3_LIBRARIES} ${PCL_LIBRARIES})

set(land_mark_object_SRC src/land_mark_object.cpp
                         src/land_mark_feature.cpp
                         src/point_cloud_feature.cpp)
set(land_mark_object_DEPS std_msgs
                          sensor_msgs
                          pcl_conversions
                          pcl_ros
                          antikythera_msgs)

# EXECUTABLES
# EMULATOR
add_executable(antikythera_emulator src/antikythera_emulator.cpp)
target_link_libraries(antikythera_emulator ${ANTIKYTHERA_LIBS})

ament_target_dependencies(antikythera_emulator
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  OpenCV
  Eigen3
)

target_include_directories(antikythera_emulator PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
target_compile_features(antikythera_emulator PUBLIC c_std_99 cxx_std_17)

# LAND MARK SLICER
add_executable(antikythera_land_mark_slicer src/antikythera_land_mark_slicer.cpp
                                            ${land_mark_object_SRC})
target_link_libraries(antikythera_land_mark_slicer ${ANTIKYTHERA_LIBS})

ament_target_dependencies(antikythera_land_mark_slicer
  rclcpp
  ${land_mark_object_DEPS}
)

target_include_directories(antikythera_land_mark_slicer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(antikythera_land_mark_slicer PUBLIC c_std_99 cxx_std_17)

# LAND MARK VISUALIZER
add_executable(land_mark_visualizer src/land_mark_visualizer.cpp
                                    ${land_mark_object_SRC})
target_link_libraries(land_mark_visualizer ${ANTIKYTHERA_LIBS})

ament_target_dependencies(land_mark_visualizer
  rclcpp
  ${land_mark_object_DEPS}
)

target_include_directories(land_mark_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(land_mark_visualizer PUBLIC c_std_99 cxx_std_17)

# INSTALL
install(TARGETS antikythera_emulator antikythera_land_mark_slicer land_mark_visualizer
  DESTINATION lib/${PROJECT_NAME})

# TESTING
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_test REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/dev_test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}_test
    std_msgs
    rclcpp
    sensor_msgs
    pcl_ros
    pcl_conversions
  )

  target_link_libraries(${PROJECT_NAME}_test ${ANTIKYTHERA_LIBS})

  # install test
  install(TARGETS ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
