---
sidebar_position: 6
---

# A Guide to Humanoid Sensor Systems

For a humanoid robot to perceive and navigate its environment, it must rely on a sophisticated suite of sensors. These sensors are the robot's equivalent of human senses, providing the raw data needed to see, hear, feel, and balance. In ROS 2, each of these sensors is typically managed by a dedicated node that publishes its data to the rest of the system.

Let's explore the most critical sensor systems for a humanoid robot.

## 1. Vision: Cameras

Cameras are the primary vision sensor, providing rich, dense information about the environment.

- **Types**:
    - **Monocular Cameras**: A single camera, providing a 2D image. Used for object recognition, color detection, and optical flow.
    - **Stereo Cameras**: Two cameras spaced a known distance apart. By comparing the two images, the robot can calculate depth and perceive the world in 3D, which is crucial for navigation and grasping.
- **ROS Message Type**: `sensor_msgs/msg/Image` for raw image data, `sensor_msgs/msg/CameraInfo` for calibration data.

## 2. 3D Perception: LiDAR

**LiDAR** (Light Detection and Ranging) is a sensor that uses laser beams to create a precise, 3D map of the environment.

- **How it Works**: A LiDAR sensor emits pulses of laser light and measures the time it takes for the light to reflect off objects and return. This allows it to calculate distance with very high accuracy.
- **Strengths**: Excellent for obstacle avoidance, localization (figuring out where the robot is), and mapping. It is unaffected by lighting conditions, unlike cameras.
- **ROS Message Type**: `sensor_msgs/msg/LaserScan` for 2D LiDAR or `sensor_msgs/msg/PointCloud2` for 3D LiDAR.

## 3. Balance and Orientation: IMUs

An **Inertial Measurement Unit (IMU)** is a critical sensor for balance and estimating the robot's orientation. It's the robot's inner ear.

- **Components**:
    - **Accelerometer**: Measures linear acceleration (changes in velocity). Can also be used to sense the direction of gravity.
    - **Gyroscope**: Measures angular velocity (how fast the robot is rotating).
- **Function**: By integrating the data from the accelerometer and gyroscope, the robot can estimate its **pose** (position and orientation) in 3D space. This is fundamental for bipedal locomotion and keeping the robot from falling over.
- **ROS Message Type**: `sensor_msgs/msg/Imu`.

## 4. Touch and Force: Force/Torque Sensors

To manipulate objects with dexterity, a robot needs a sense of touch. **Force/Torque (F/T) sensors** provide this capability.

- **Location**: These sensors are typically placed in the robot's wrists, ankles, and fingertips.
- **Function**:
    - **In the hands**: F/T sensors allow the robot to perform delicate tasks like grasping a fragile object without crushing it. They provide feedback on how much force is being applied.
    - **In the feet/ankles**: They measure the forces exerted by the ground, which is essential for balance control, adjusting to different surfaces (e.g., hard floor vs. soft carpet), and detecting collisions.
- **ROS Message Type**: `geometry_msgs/msg/WrenchStamped`.

These sensors, working in concert, provide the robot with a comprehensive understanding of its body and its immediate surroundings. A central challenge in Physical AI is **sensor fusion**—the process of combining data from these multiple, imperfect sensor sources to create a single, coherent model of the world.
