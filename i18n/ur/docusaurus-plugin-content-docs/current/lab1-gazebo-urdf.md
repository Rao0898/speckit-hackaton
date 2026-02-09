---
sidebar_position: 2
---

# لیب: گزیبو میں URDF ماڈل کی سمولیشن

یہ لیب آپ کو گزیبو میں ایک سادہ URDF (Unified Robot Description Format) ماڈل لانچ کرنے اور ROS 2 کا استعمال کرتے ہوئے اس کے ساتھ تعامل کرنے کے طریقے کے بارے میں رہنمائی فراہم کرے گی۔

## پیشگی ضروریات

-   ایک فعال ROS 2 انسٹالیشن۔
-   گزیبو انسٹال ہونا چاہیے (عام طور پر `ros-<distro>-gazebo-ros-pkgs`)۔
-   ماڈیول 1 سے ROS 2 تصورات (نوڈس، ٹاپکس) سے واقفیت۔

## 1. URDF پیکیج بنائیں

1.  **ایک نیا ROS 2 ورک اسپیس بنائیں** (اگر آپ کے پاس پہلے سے نہیں ہے):
    ```bash
    mkdir -p ~/gazebo_urdf_ws/src
    cd ~/gazebo_urdf_ws/src
    ```

2.  **ایک نیا پیکیج بنائیں**:
    ```bash
    ros2 pkg create --build-type ament_cmake my_robot_description
    cd my_robot_description
    ```

3.  **ایک `urdf` ڈائریکٹری بنائیں**:
    ```bash
    mkdir urdf
    ```

## 2. اپنے روبوٹ کی تعریف کریں (URDF)

`urdf` ڈائریکٹری (`~/gazebo_urdf_ws/src/my_robot_description/urdf/simple_robot.urdf`) کے اندر `simple_robot.urdf` نامی ایک فائل بنائیں۔

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

## 3. گزیبو کے لیے لانچ فائل

اپنے پیکیج (`~/gazebo_urdf_ws/src/my_robot_description/launch/`) میں ایک `launch` ڈائریکٹری بنائیں۔
اس کے اندر `display_robot.launch.py` نامی ایک فائل بنائیں۔

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
        # گزیبو لانچ کریں۔
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # گزیبو میں روبوٹ کو سپان کریں۔
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '1'],
            output='screen'
        )
    ])
```

## 4. `CMakeLists.txt` اور `package.xml` کو اپ ڈیٹ کریں

`~/gazebo_urdf_ws/src/my_robot_description/CMakeLists.txt` میں ترمیم کریں:

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

`~/gazebo_urdf_ws/src/my_robot_description/package.xml` میں ترمیم کریں:

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

## 5. بنائیں اور لانچ کریں

1.  **اپنی ورک اسپیس بنائیں**:
    ```bash
    cd ~/gazebo_urdf_ws
    colcon build --packages-select my_robot_description
    ```

2.  **ورک اسپیس کو سورس کریں**:
    ```bash
    source install/setup.bash
    ```

3.  **گزیبو میں روبوٹ لانچ کریں**:
    ```bash
    ros2 launch my_robot_description display_robot.launch.py
    ```

گزیبو آپ کے سادہ نیلے باکس روبوٹ کے ساتھ لانچ ہونا چاہیے۔ آپ اسے ماؤس کے ساتھ ادھر ادھر دھکیل کر اس کے ساتھ تعامل کر سکتے ہیں۔
مبارک ہو، آپ نے ROS 2 کا استعمال کرتے ہوئے گزیبو میں کامیابی سے ایک URDF ماڈل لانچ کیا ہے!