cmake_minimum_required(VERSION 3.8)
project(so_arm_commander_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================
# Find Dependencies
# ============================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ============================================
# Include Directories
# ============================================

include_directories(
  include
)

# ============================================
# Library: so_arm_commander
# ============================================

add_library(${PROJECT_NAME} SHARED
  src/so_arm_commander.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  tf2_geometry_msgs
)

# ============================================
# Register as rclcpp component
# ============================================

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "so_arm_commander::SoArmCommander"
  EXECUTABLE so_arm_commander_node
)

# ============================================
# Example Executables (optional)
# ============================================

# Basic Motion Example
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/basic_motion_example.cpp)
  add_executable(basic_motion_example
    examples/basic_motion_example.cpp
  )
  target_link_libraries(basic_motion_example ${PROJECT_NAME})
  ament_target_dependencies(basic_motion_example
    rclcpp
    geometry_msgs
  )
endif()

# Pick and Place Example
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/pick_place_example.cpp)
  add_executable(pick_place_example
    examples/pick_place_example.cpp
  )
  target_link_libraries(pick_place_example ${PROJECT_NAME})
  ament_target_dependencies(pick_place_example
    rclcpp
    geometry_msgs
  )
endif()

# Named Poses Example
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/named_poses_example.cpp)
  add_executable(named_poses_example
    examples/named_poses_example.cpp
  )
  target_link_libraries(named_poses_example ${PROJECT_NAME})
  ament_target_dependencies(named_poses_example
    rclcpp
    geometry_msgs
  )
endif()

# ============================================
# Installation
# ============================================

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install example executables (if built)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/basic_motion_example.cpp)
  install(TARGETS basic_motion_example
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/pick_place_example.cpp)
  install(TARGETS pick_place_example
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/named_poses_example.cpp)
  install(TARGETS named_poses_example
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install config directory (if exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

# ============================================
# Export Information
# ============================================

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  tf2_geometry_msgs
)

# ============================================
# Testing
# ============================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ============================================
# Package
# ============================================

ament_package()
