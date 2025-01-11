# ROS2 Tutorial - Parameters Demonstration

[![ROS2 Version](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Developer](https://img.shields.io/badge/Developer-shashank3199-green)](https://github.com/shashank3199)

This repository demonstrates the usage of ROS2 Parameters through a collection of C++ and Python nodes. The examples showcase parameter declaration, reading, writing, and dynamic updates, highlighting ROS2's parameter system capabilities and cross-language interoperability.

## Table of Contents

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Package Description](#package-description)
  - [Parameter Nodes](#parameter-nodes)
  - [Sample Configuration](#sample-configuration)
- [Building the Package](#building-the-package)
- [Running the Examples](#running-the-examples)
- [Package Dependencies](#package-dependencies)

## Overview

This tutorial demonstrates three different approaches to working with ROS2 Parameters:

1. Reading parameters across nodes
2. Writing and updating parameters
3. Dynamic parameter modification

Each node demonstrates various parameter operations using both C++ and Python implementations, showcasing how parameters can be:

- Declared with initial values and descriptors
- Read synchronously
- Updated individually or in batches
- Modified dynamically based on conditions
- Monitored for changes

## Repository Structure

```plaintext
📦 param_demo
 ┣ 📂 config
 ┃ ┗ 📄 default_params.yaml    # Default parameter configurations
 ┣ 📂 src
 ┃ ┣ 📄 cpp_read_node.cpp      # C++ parameter reader
 ┃ ┣ 📄 cpp_write_node.cpp     # C++ parameter writer
 ┃ ┗ 📄 cpp_modify_node.cpp    # C++ parameter modifier
 ┣ 📂 param_demo
 ┃ ┣ 📄 py_read_node.py        # Python parameter reader
 ┃ ┣ 📄 py_write_node.py       # Python parameter writer
 ┃ ┗ 📄 py_modify_node.py      # Python parameter modifier
 ┣ 📄 CMakeLists.txt           # Build configuration
 ┣ 📄 package.xml              # Package dependencies
 ┗ 📄 setup.py                 # Python package setup
```

## Package Description

### Parameter Nodes

1. **Reader Nodes** (`cpp_read_node`, `py_read_node`)
   - Subscribe to parameter changes
   - Monitor specific parameter namespaces
   - Implement parameter change callbacks
   - Handle reader status states

2. **Writer Nodes** (`cpp_write_node`, `py_write_node`)
   - Declare parameters with descriptors
   - Perform individual parameter updates
   - Execute batch parameter updates
   - Implement periodic parameter modifications

3. **Modifier Nodes** (`cpp_modify_node`, `py_modify_node`)
   - Toggle parameter states
   - Demonstrate remote parameter updates
   - Handle parameter update callbacks
   - Manage parameter service interactions

### Sample Configuration

The package includes a default parameter configuration (`config/default_params.yaml`) with sample robot parameters:

```yaml
/**:
  ros__parameters:
    my_robot:
      name: 'RoboX_loaded'
      debug_mode: true
      max_speed: 1.5
      sensor_list:
        - 'lidar_loaded'
        - 'camera_loaded'
        - 'ultrasonic_loaded'
```

## Building the Package

1. Create a new ROS2 workspace:

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2. Clone this repository:

    ```bash
    git clone <repository-url>
    ```

3. Install dependencies:

    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. Build the package:

    ```bash
    colcon build --packages-select param_demo
    ```

5. Source the workspace:

    ```bash
    source install/setup.bash
    ```

## Running the Examples

You can run different combinations of nodes to observe parameter interactions:

### C++ Nodes

```bash
# Terminal 1: Run the parameter reader
ros2 run param_demo cpp_read_node

# Terminal 2: Run the parameter writer
ros2 run param_demo cpp_write_node

# Terminal 3: Run the parameter modifier
ros2 run param_demo cpp_modify_node
```

### Python Nodes

```bash
# Terminal 1: Run the parameter reader
ros2 run param_demo py_read_node

# Terminal 2: Run the parameter writer
ros2 run param_demo py_write_node

# Terminal 3: Run the parameter modifier
ros2 run param_demo py_modify_node
```

### Command Line Parameter Operations

```bash
# List all parameters
ros2 param list

# Get a specific parameter
ros2 param get /parameter_writer my_robot.name

# Set a parameter
ros2 param set /parameter_writer my_robot.max_speed 2.0

# Load parameters from file
ros2 param load /parameter_writer config/default_params.yaml

# Save parameters to file
ros2 param dump /parameter_writer params_backup.yaml
```

## Package Dependencies

Common dependencies for all nodes:

- ROS2 Humble
- rclcpp (C++ nodes)
- rclpy (Python nodes)
- rcl_interfaces
- rclcpp_components
- ament_cmake
- ament_cmake_python

Additional testing dependencies:

- ament_lint_auto
- ament_lint_common
- ament_cmake_pytest

Each node's specific dependencies can be found in the `package.xml` file.
