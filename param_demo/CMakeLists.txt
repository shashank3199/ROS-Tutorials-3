cmake_minimum_required(VERSION 3.8)
project(param_demo)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)

# Add C++ executables
add_executable(cpp_read_node src/cpp_read_node.cpp)
ament_target_dependencies(cpp_read_node rclcpp)

add_executable(cpp_write_node src/cpp_write_node.cpp)
ament_target_dependencies(cpp_write_node rclcpp)

add_executable(cpp_modify_node src/cpp_modify_node.cpp)
ament_target_dependencies(cpp_modify_node rclcpp)

# Install C++ executables
install(TARGETS
  cpp_read_node
  cpp_write_node
  cpp_modify_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python nodes
install(PROGRAMS
  param_demo/py_read_node.py
  param_demo/py_write_node.py
  param_demo/py_modify_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if you have any
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()