---
sidebar_position: 1
---

# Introduction to Gazebo

**Gazebo** is a powerful 3D robot simulator that is widely used in robotics research and development. It allows you to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. Gazebo offers the ability to simulate realistic sensor feedback and apply physics to robot models.

## Why use Gazebo?

For humanoid robotics, simulating in Gazebo offers several key advantages:

-   **Safety**: Test complex robot behaviors without risking damage to expensive physical hardware or endangering humans.
-   **Reproducibility**: Easily recreate specific scenarios for testing and debugging, ensuring consistent results.
-   **Cost-effectiveness**: Develop and iterate on robot designs and algorithms without needing to purchase or maintain physical robots.
-   **Scalability**: Simulate multiple robots and diverse environments, allowing for large-scale experiments and multi-robot coordination.
-   **Sensor Simulation**: Gazebo can accurately simulate common robot sensors like LiDAR, cameras (RGB, depth), IMUs (Inertial Measurement Units), and force-torque sensors. This allows you to develop perception and control algorithms with realistic data.

## Key Features of Gazebo

-   **Physics Engine**: Gazebo includes powerful physics engines (e.g., ODE, Bullet, Simbody, DART) that accurately simulate rigid body dynamics, gravity, friction, and collisions.
-   **Rendering**: High-quality 3D graphics engine allows for realistic visualization of robots and environments.
-   **Robot Models (URDF/SDF)**: Supports URDF (Unified Robot Description Format) and SDF (Simulation Description Format) for describing robot kinematics, dynamics, and visual properties.
-   **Plugins**: Extend Gazebo's functionality with custom plugins for sensors, actuators, and control algorithms. Many ROS 2 packages provide Gazebo plugins.
-   **Integration with ROS 2**: Seamless integration with ROS 2 provides standard interfaces for communication between your robot software and the simulation environment.

Throughout this module, you will learn how to build and import robot models into Gazebo, create virtual environments, and simulate interactions. You will also learn how to integrate Gazebo with ROS 2 to send commands to your simulated humanoid and receive sensor data.
