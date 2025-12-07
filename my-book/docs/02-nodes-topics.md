---
sidebar_position: 2
---

# ROS 2 Nodes and Topics

In ROS 2, the fundamental unit of computation is a **Node**. Nodes are essentially executable processes that perform a specific task within the robot system. For example, you might have one node for reading data from a camera, another for controlling a motor, and a third for navigating.

## Nodes

Each node should be designed to be modular and focused on a single responsibility. This makes the system easier to understand, debug, and maintain.

Key characteristics of nodes:
- **Executables**: Can be written in Python, C++, or other languages.
- **Independent**: Nodes run independently and communicate with each other.
- **Named**: Each node has a unique name within the ROS graph to prevent conflicts.

## Topics

**Topics** are the primary mechanism for nodes to exchange data in a publish-subscribe communication model. Think of topics as named data buses where nodes can send (publish) messages and receive (subscribe) messages.

- **Publishers**: Nodes that send data to a topic.
- **Subscribers**: Nodes that receive data from a topic.
- **Messages**: The actual data transmitted over a topic. Messages have a defined type (e.g., `std_msgs/msg/String`, `sensor_msgs/msg/Image`).

### How Topics Work

When a publisher publishes a message to a topic, all nodes subscribed to that topic will receive a copy of the message. This communication is asynchronous and one-way, making it suitable for streaming data like sensor readings, joint states, or velocity commands.

**Example Use Cases for Topics:**
- Publishing camera images from a camera node to an image processing node.
- Publishing joint positions from a motor control node to a visualization node.
- Subscribing to joystick commands to control robot movement.

Understanding nodes and topics is crucial for building any ROS 2 application, as they form the backbone of inter-process communication in a distributed robotic system.
