cmake_minimum_required(VERSION 3.8)
project(so_arm_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(so_arm_commander_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Create task_manager library
add_library(task_manager SHARED
  src/task_manager.cpp
)

ament_target_dependencies(task_manager
  rclcpp
  so_arm_commander_cpp
  geometry_msgs
  std_msgs
)

# Create task_examples executable
add_executable(task_examples
  src/task_examples.cpp
)

target_link_libraries(task_examples
  task_manager
)

ament_target_dependencies(task_examples
  rclcpp
  so_arm_commander_cpp
  geometry_msgs
  std_msgs
)

# Install library
install(TARGETS task_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS task_examples
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(task_manager)
ament_export_dependencies(
  rclcpp
  so_arm_commander_cpp
  geometry_msgs
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
