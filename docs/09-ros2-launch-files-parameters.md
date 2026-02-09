---
sidebar_position: 9
---

# ROS 2 Launch Files and Parameter Management

As your robotic systems grow in complexity, running each node in a separate terminal becomes tedious and unmanageable. ROS 2 provides a powerful **launch system** that allows you to start and configure multiple nodes at once from a single command. Additionally, the launch system provides a robust way to manage **parameters**, which are configurable settings for your nodes.

## What is a ROS 2 Launch File?

A launch file is a script (typically written in Python) that describes how to run a set of nodes. With a launch file, you can:
- Start multiple nodes simultaneously.
- Automatically restart nodes if they crash.
- Remap topic, service, or action names.
- Set parameters for your nodes.

## Creating a Python Launch File

Launch files are usually placed in a `launch` directory inside your ROS 2 package.

Here is an example of a simple Python launch file that starts two nodes: a talker and a listener.

**`my_package/launch/my_launch_file.py`**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='talker',
            name='my_talker'
        ),
        Node(
            package='my_package',
            executable='listener',
            name='my_listener'
        ),
    ])
```

- **`generate_launch_description()`**: This is the main function that the `ros2 launch` command looks for. It must return a `LaunchDescription` object.
- **`LaunchDescription`**: This object is a container for all the entities (like nodes) you want to launch.
- **`Node`**: This action represents a single ROS 2 node. You specify the `package` it belongs to, its `executable` name (as defined in `setup.py`), and you can assign it a custom `name`.

To run this launch file, you would execute:
```bash
ros2 launch my_package my_launch_file.py
```

## Parameter Management

Parameters are a way to configure your nodes without changing their source code. You can think of them as settings that you can change at runtime or at launch time.

### Declaring Parameters in a Python Node

First, you must declare the parameters that your node accepts. This is typically done in the node's constructor.

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_parameter_node')
        
        # Declare a parameter named 'my_parameter' with a default value
        self.declare_parameter('my_parameter', 'default_value')

        # Get the parameter's value
        my_param_value = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info(f'My parameter is: {my_param_value}')
```
You can declare parameters of different types, such as strings, integers, booleans, and floats.

### Setting Parameters in a Launch File

The most common way to set parameters is through your launch file. This allows you to manage all your node configurations in one place.

You can pass parameters to a `Node` action using a YAML file.

**`my_package/config/my_params.yaml`**
```yaml
my_parameter_node:
  ros__parameters:
    my_parameter: "a_new_value_from_yaml"
```

**Note**: The structure is `node_name` -> `ros__parameters` -> `param_name: value`.

Now, you can modify your launch file to load and pass this YAML file to the node.

**`my_package/launch/my_param_launch.py`**
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package's share directory
    pkg_share = get_package_share_directory('my_package')
    # Get the path to the parameter file
    param_file = os.path.join(pkg_share, 'config', 'my_params.yaml')

    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_parameter_node',
            name='my_parameter_node',
            parameters=[param_file]  # Pass the parameter file
        ),
    ])
```

When you run this launch file, your node will start with `my_parameter` set to `"a_new_value_from_yaml"` instead of its default value.

Using launch files and parameters is essential for building complex, configurable, and reusable robotic applications. It separates your code (the "how") from your configuration (the "what"), which is a fundamental principle of good software engineering.
