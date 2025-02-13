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

# INCLUDE DIRECTORIES
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# VARIABLES FOR LIBRARIES
set(OPENCV_LIBS ${OpenCV_LIBRARIES})
set(EIGEN_LIBS ${EIGEN3_LIBRARIES})
set(PCL_LIBS ${PCL_LIBRARIES})
set(ANTIKYTHERA_LIBS ${OPENCV_LIBS} ${EIGEN_LIBS} ${PCL_LIBS})

# EXECUTABLES
add_executable(antikythera_emulator src/antikythera_emulator.cpp)
add_executable(antikythera_land_mark_slicer src/antikythera_land_mark_slicer.cpp)

# LINKING
target_link_libraries(antikythera_emulator ${ANTIKYTHERA_LIBS})
target_link_libraries(antikythera_land_mark_slicer ${ANTIKYTHERA_LIBS})

ament_target_dependencies(antikythera_emulator
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(antikythera_land_mark_slicer
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
)

target_include_directories(antikythera_emulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(antikythera_emulator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(antikythera_land_mark_slicer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(antikythera_land_mark_slicer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS antikythera_emulator antikythera_land_mark_slicer
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

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
  )

  target_link_libraries(${PROJECT_NAME}_test ${ANTIKYTHERA_LIBS})

endif()

ament_package()