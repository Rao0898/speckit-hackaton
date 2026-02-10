---
sidebar_position: 8
---

# Building ROS 2 Packages with Python

In ROS 2, code is organized into **packages**. A package is a directory containing your source code (e.g., Python scripts), launch files, configuration files, and a `package.xml` file that provides metadata about the package. The build system used by ROS 2 is called **colcon**.

This guide will walk you through the standard structure of a Python-based ROS 2 package and the process of building it.

## The ROS 2 Workspace

Before creating a package, you need a **workspace**. A workspace is a directory where you will store and build your ROS 2 packages.

A typical workspace structure looks like this:
```
ros2_ws/
├── src/      # Source space: where you put your package source code
├── build/    # Build space: where colcon stores intermediate build files
├── install/  # Install space: where the final package files are installed
└── log/      # Log space: where colcon stores log information
```

## Creating a ROS 2 Package with Python

You can create a new package using the `ros2 pkg create` command. Let's create a simple publisher/subscriber package.

1.  Navigate to your workspace's `src` directory:
    ```bash
    cd ~/ros2_ws/src
    ```

2.  Run the package creation command:
    ```bash
    ros2 pkg create --build-type ament_python --node-name my_publisher my_python_pkg
    ```
    - `--build-type ament_python`: Specifies that this is a Python package.
    - `--node-name my_publisher`: Automatically creates a basic executable script for a node named `my_publisher`.
    - `my_python_pkg`: The name of your new package.

This will generate a directory named `my_python_pkg` with the following structure:

```
my_python_pkg/
├── my_python_pkg/
│   ├── __init__.py
│   └── my_publisher.py   # Your Python node script
├── resource/
│   └── my_python_pkg
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml           # Package manifest
└── setup.py              # Python package setup script
└── setup.cfg
```

## Key Files in a Python Package

### `package.xml`

This file contains metadata about your package, such as its name, version, author, and dependencies. When you create a package, you need to add its dependencies here. For a simple Python publisher, you would add:

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
```

`rclpy` is the ROS 2 client library for Python, and `std_msgs` contains standard message types.

### `setup.py`

This is the build script for your Python package. It tells `colcon` how to install your package and where to find its executable scripts (your nodes). A crucial part of this file is the `entry_points` dictionary within the `setup()` function.

```python
entry_points={
    'console_scripts': [
        'my_publisher = my_python_pkg.my_publisher:main',
    ],
},
```
This line creates a "console script" or executable named `my_publisher`. When you run this script, it will execute the `main` function located in the `my_publisher.py` file inside your `my_python_pkg` module.

### Your Python Node (`my_publisher.py`)

This is where you write the logic for your ROS 2 node. The template will create a minimal class structure for you to fill in. You will import `rclpy`, message types, and then define your publisher, subscriber, service, or action clients/servers within your node's class.

## Building and Sourcing Your Package

1.  **Build the Package**: From the root of your workspace (`~/ros2_ws`), run the `colcon build` command:
    ```bash
    colcon build
    ```
    `colcon` will find all the packages in your `src` directory, resolve their dependencies, and build them, placing the output in the `build` and `install` directories.

2.  **Source the Workspace**: After a successful build, you need to "source" the `setup.bash` file in the `install` directory. This script adds your newly built package to your shell's environment, allowing ROS 2 to find your nodes.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```
    You must do this in every new terminal you open.

3.  **Run Your Node**: Now you can run your node using `ros2 run`:
    ```bash
    ros2 run my_python_pkg my_publisher
    ```

This workflow—create, code, build, source, run—is the fundamental development cycle for creating any ROS 2 application.
