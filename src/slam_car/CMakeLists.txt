cmake_minimum_required(VERSION 3.8)
project(slam_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Gazebo Sim (Ignition) dependencies
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(sdformat12 REQUIRED)

set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})



# Create keyboard control executable
add_executable(keyboard_control
  src/keyboard_control.cpp
)

target_include_directories(keyboard_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(keyboard_control PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(keyboard_control
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Create diff-drive system plugin (Gazebo Sim / Ignition)
add_library(diff-drive SHARED
  src/DiffDrive.cc
)

target_include_directories(diff-drive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ignition-gazebo6_INCLUDE_DIRS}
  ${ignition-common4_INCLUDE_DIRS}
  ${ignition-math6_INCLUDE_DIRS}
  ${ignition-plugin1_INCLUDE_DIRS}
  ${ignition-transport11_INCLUDE_DIRS}
  ${ignition-msgs8_INCLUDE_DIRS}
  ${sdformat12_INCLUDE_DIRS}
)

target_link_libraries(diff-drive
  ignition-gazebo${GZ_SIM_VER}::core
  ignition-common4::ignition-common4
  ignition-math6::ignition-math6
  ignition-plugin1::register
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8
  sdformat12::sdformat12
)

# Create four-wheel-drive system plugin (Gazebo Sim / Ignition)
add_library(four-wheel-drive SHARED
  src/FourWheelDrive.cc
)

target_include_directories(four-wheel-drive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ignition-gazebo6_INCLUDE_DIRS}
  ${ignition-common4_INCLUDE_DIRS}
  ${ignition-math6_INCLUDE_DIRS}
  ${ignition-plugin1_INCLUDE_DIRS}
  ${ignition-transport11_INCLUDE_DIRS}
  ${ignition-msgs8_INCLUDE_DIRS}
  ${sdformat12_INCLUDE_DIRS}
)

target_link_libraries(four-wheel-drive
  ignition-gazebo${GZ_SIM_VER}::core
  ignition-common4::ignition-common4
  ignition-math6::ignition-math6
  ignition-plugin1::register
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8
  sdformat12::sdformat12
)

# Install the libraries and executables
install(TARGETS 
  diff-drive
  four-wheel-drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  keyboard_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  meshes
  urdf
  world
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when a
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()