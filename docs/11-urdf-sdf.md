---
sidebar_position: 11
---

# URDF and SDF: Describing Your Robot

To simulate a robot in Gazebo or visualize it in RViz, you first need a way to describe its physical structure. ROS 2 and Gazebo use two primary formats for this: the **Unified Robot Description Format (URDF)** and the **Simulation Description Format (SDF)**.

## URDF: The Standard for ROS

**URDF** is an XML-based format used in ROS to describe the kinematics and dynamics of a robot model. It is the standard for describing robots outside of simulation.

A URDF file defines the robot as a tree of **links** and **joints**.

- **`<link>`**: A link represents a rigid body part of the robot (e.g., a forearm, a wheel, a torso). Each link has physical properties like mass and inertia, as well as visual and collision properties.
    - **`<visual>`**: Defines the appearance of the link (what it looks like). This is often a 3D mesh file (like `.dae` or `.stl`).
    - **`<collision>`**: Defines the collision geometry of the link (its physical shape for collision checking). This can be a simple shape (like a box or cylinder) or a mesh.
    - **`<inertial>`**: Defines the dynamic properties of the link, including its mass and inertia matrix.

- **`<joint>`**: A joint connects two links together and defines their relative motion.
    - **`parent` and `child`**: Specifies which two links are being connected.
    - **`type`**: The type of motion allowed by the joint. Common types include:
        - `revolute`: A hinge joint that rotates around a single axis (like an elbow).
        - `continuous`: A revolute joint with no angle limits (like a wheel).
        - `prismatic`: A sliding joint that moves along a single axis.
        - `fixed`: A rigid connection between two links.

**Example URDF Snippet:**
```xml
<robot name="simple_arm">
  <!-- Base Link -->
  <link name="base_link">
    ...
  </link>

  <!-- Arm Link -->
  <link name="arm_link">
    ...
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="base_to_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.5"/>
    <axis xyz="0 1 0"/>
    <limit effort="1000.0" lower="-1.57" upper="1.57" velocity="0.5"/>
  </joint>
</robot>
```

### URDF Limitations

URDF is excellent for describing the kinematics of a single robot, but it has limitations:
- It cannot represent a closed-loop chain (a kinematic loop).
- It cannot be used to describe non-robot items in the world (like lights, tables, or buildings).
- It has limited support for specifying sensor models directly.

## SDF: The Language of Gazebo

**SDF** is the native format for Gazebo. It is also an XML-based format, but it is a superset of URDF. SDF is designed to describe everything about a simulation, not just a single robot.

Key features of SDF:
- **World Description**: Can describe an entire world, including robots, static objects, lights, physics properties, and more.
- **Support for Closed Loops**: Can model complex mechanisms that URDF cannot.
- **Detailed Sensor Models**: Provides a rich set of tags for defining sensors (cameras, LiDAR, IMUs) and their noise models.
- **Plugin Specification**: Allows you to specify which plugins should be loaded for a given model or sensor.

You can think of SDF as a more complete language for simulation. While URDF describes *what your robot is*, SDF describes *what your simulation is*.

## URDF vs. SDF: Which One to Use?

- **Use URDF as your primary robot description format**. It is the standard across the ROS ecosystem, and tools like RViz and MoveIt rely on it.
- **Use SDF for your Gazebo world files (`.world`)**.
- When you need to add Gazebo-specific features to your robot model (like sensors or plugins), you can add special `<gazebo>` tags inside your URDF file. ROS tools will ignore these tags, but Gazebo will parse them.

**Example: Adding a Gazebo color tag to a URDF link**
```xml
<link name="my_link">
  ...
  <gazebo reference="my_link">
    <material>Gazebo/Red</material>
  </gazebo>
</link>
```

For complex simulations, you might convert your URDF to an SDF file on the fly when you spawn the robot into Gazebo. The `gazebo_ros` package provides tools to do this automatically. This approach allows you to maintain a single, canonical URDF for your robot while still leveraging the full power of SDF for simulation.
