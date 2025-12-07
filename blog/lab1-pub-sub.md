---
sidebar_position: 3
---

# Lab: Creating a Simple Publisher/Subscriber

This lab will guide you through creating your first ROS 2 nodes in Python. You will create two nodes: one that publishes a "hello world" message to a topic, and another that subscribes to that topic and prints the received messages.

## Prerequisites

-   A working ROS 2 installation (e.g., Foxy, Galactic, Humble).
-   Basic familiarity with Python programming.

## Setup

1.  **Create a ROS 2 Workspace**: If you don't have one, create a new ROS 2 workspace.
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **Create a Python Package**: Create a new Python package named `my_ros2_package`.
    ```bash
    ros2 pkg create --build-type ament_python my_ros2_package
    ```

3.  **Navigate to the Package Directory**:
    ```bash
    cd my_ros2_package
    ```

## 1. Create the Publisher Node

Create a new file named `minimal_publisher.py` in `~/ros2_ws/src/my_ros2_package/my_ros2_package/`.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2 World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2. Create the Subscriber Node

Create a new file named `minimal_subscriber.py` in the same directory: `~/ros2_ws/src/my_ros2_package/my_ros2_package/`.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. Update `setup.py`

Modify `~/ros2_ws/src/my_ros2_package/setup.py` to include the entry points for your new executables.

```python
from setuptools import find_packages, setup

package_name = 'my_ros2_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_launch_file.launch.py']), # Example launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_publisher = my_ros2_package.minimal_publisher:main',
            'minimal_subscriber = my_ros2_package.minimal_subscriber:main',
        ],
    },
)
```

## 4. Build and Run

1.  **Build your package**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_ros2_package
    ```

2.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

3.  **Run the publisher**: In one terminal:
    ```bash
    ros2 run my_ros2_package minimal_publisher
    ```

4.  **Run the subscriber**: In another terminal:
    ```bash
    ros2 run my_ros2_package minimal_subscriber
    ```

You should see the publisher sending messages and the subscriber receiving and printing them. Congratulations, you've created your first ROS 2 publisher and subscriber nodes!
