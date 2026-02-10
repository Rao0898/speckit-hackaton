---
sidebar_position: 10
---

# Setting Up the Gazebo Simulation Environment

**Gazebo** is a powerful and widely-used 3D robotics simulator. It allows you to model complex robots and environments, and it includes a high-fidelity physics engine that simulates gravity, friction, and collisions. For ROS 2 developers, Gazebo is an essential tool for testing and developing robot software before deploying it on physical hardware.

This guide covers the basics of installing and running Gazebo with ROS 2.

## Installation

Gazebo is tightly integrated with ROS 2. The recommended way to install it is through the `ros-dev-tools` and the Gazebo ROS package.

For most ROS 2 distributions (like Humble or Iron), you can install Gazebo with the following command:

```bash
sudo apt-get install ros-<distro>-gazebo-ros-pkgs
```
Replace `<distro>` with the name of your ROS 2 distribution (e.g., `humble`). This command installs Gazebo itself, as well as the necessary plugins to interface it with ROS 2.

## The Gazebo Architecture

Gazebo runs as two main processes:

1.  **Gazebo Server (`gzserver`)**: This is the headless backend that runs the physics simulation, generates sensor data, and accepts commands. You can run `gzserver` on its own without any graphical interface.
2.  **Gazebo Client (`gzclient`)**: This is the graphical front-end that connects to the `gzserver` and visualizes the simulation. You can use it to inspect the environment, interact with models, and view sensor outputs.

## Launching Gazebo

You can launch a simple, empty world in Gazebo using the following command:

```bash
gazebo
```

To launch a specific world file, you can pass it as an argument:
```bash
gazebo worlds/empty.world
```

### Launching Gazebo with ROS 2

The `gazebo_ros` package provides a convenient launch file for starting Gazebo. You can run it with:

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

This launch file starts the Gazebo server and client. You can also pass arguments to it. For example, to launch a specific world file:

```bash
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/your/world.sdf
```

## The Gazebo World

A Gazebo **world** is described in a file using the **Simulation Description Format (SDF)**. A `.world` file (which is an SDF file) defines everything in the simulation, including:
- The environment (lighting, gravity).
- Static objects (buildings, furniture, terrain).
- Robot models.
- The physics properties of the world.

Here is a snippet from a simple world file:

```xml
<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      ...
    </light>
    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
    </scene>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

## Spawning a Robot

To add a robot to your Gazebo simulation, you "spawn" it. This is typically done using a ROS 2 node that calls the `/spawn_entity` service provided by Gazebo.

The process usually involves:
1.  Having your robot's model described in a URDF or SDF file.
2.  Starting a Gazebo world.
3.  Using a launch file to run a `spawn_entity.py` node, which reads your robot's description file and sends it to Gazebo to be added to the simulation.

We will cover robot description formats like URDF and SDF in the next section. Mastering Gazebo is fundamental for any roboticist, as it provides a safe, fast, and cost-effective way to iterate on your robot's design and control algorithms.
