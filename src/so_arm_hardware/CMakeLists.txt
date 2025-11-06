cmake_minimum_required(VERSION 3.8)
project(so_arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
)

# FTServo driver library (static)
# Only includes files needed for STS servo support
add_library(ft_servo_driver STATIC
  src/STSServo.cpp
  src/SMS_STS.cpp
  src/SCS.cpp
  src/SCSerial.cpp
)

# Set position independent code for static library
set_target_properties(ft_servo_driver PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(ft_servo_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
)

# Hardware interface library (shared)
add_library(${PROJECT_NAME} SHARED
  src/so_arm_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

target_link_libraries(${PROJECT_NAME}
  ft_servo_driver
)

# Export the plugin description
pluginlib_export_plugin_description_file(hardware_interface so_arm_hardware_plugin.xml)

# Install targets
install(TARGETS ${PROJECT_NAME} ft_servo_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
  ft_servo_driver
)

ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
