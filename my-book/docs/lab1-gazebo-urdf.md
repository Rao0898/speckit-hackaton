---
sidebar_position: 2
---

# Lab: Simulating a URDF Model in Gazebo

This lab will guide you through launching a simple URDF (Unified Robot Description Format) model in Gazebo and interacting with it using ROS 2.

## Prerequisites

-   A working ROS 2 installation.
-   Gazebo installed (usually `ros-<distro>-gazebo-ros-pkgs`).
-   Familiarity with ROS 2 concepts (nodes, topics) from Module 1.

## 1. Create a URDF Package

1.  **Create a new ROS 2 workspace** (if you don't have one):
    ```bash
    mkdir -p ~/gazebo_urdf_ws/src
    cd ~/gazebo_urdf_ws/src
    ```

2.  **Create a new package**:
    ```bash
    ros2 pkg create --build-type ament_cmake my_robot_description
    cd my_robot_description
    ```

3.  **Create a `urdf` directory**:
    ```bash
    mkdir urdf
    ```

## 2. Define Your Robot (URDF)

Create a file named `simple_robot.urdf` inside the `urdf` directory (`~/gazebo_urdf_ws/src/my_robot_description/urdf/simple_robot.urdf`).

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

</robot>
```

## 3. Launch File for Gazebo

Create a `launch` directory in your package (`~/gazebo_urdf_ws/src/my_robot_description/launch/`).
Create a file named `display_robot.launch.py` inside it.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'my_robot_description'
    urdf_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'simple_robot.urdf')

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '1'],
            output='screen'
        )
    ])
```

## 4. Update `CMakeLists.txt` and `package.xml`

Modify `~/gazebo_urdf_ws/src/my_robot_description/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

install(
    DIRECTORY urdf launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

Modify `~/gazebo_urdf_ws/src/my_robot_description/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_description</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>gazebo_ros</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## 5. Build and Launch

1.  **Build your workspace**:
    ```bash
    cd ~/gazebo_urdf_ws
    colcon build --packages-select my_robot_description
    ```

2.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

3.  **Launch the robot in Gazebo**:
    ```bash
    ros2 launch my_robot_description display_robot.launch.py
    ```

Gazebo should launch with your simple blue box robot. You can interact with it by pushing it around with your mouse.
Congratulations, you've successfully launched a URDF model in Gazebo using ROS 2!
