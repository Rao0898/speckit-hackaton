---
sidebar_position: 3
---

# لیب: ایک سادہ پبلشر/سبسکرائبر بنانا

یہ لیب آپ کو پائتھون میں اپنے پہلے ROS 2 نوڈز بنانے میں رہنمائی کرے گی۔ آپ دو نوڈز بنائیں گے: ایک جو ایک ٹاپک پر "ہیلو ورلڈ" پیغام شائع کرے گا، اور دوسرا جو اس ٹاپک کو سبسکرائب کرے گا اور موصول ہونے والے پیغامات کو پرنٹ کرے گا۔

## پیشگی ضروریات

-   ایک فعال ROS 2 انسٹالیشن (مثال کے طور پر، Foxy، Galactic، Humble)۔
-   پائتھون پروگرامنگ سے بنیادی واقفیت۔

## سیٹ اپ

1.  **ایک ROS 2 ورک اسپیس بنائیں**: اگر آپ کے پاس پہلے سے کوئی نہیں ہے، تو ایک نیا ROS 2 ورک اسپیس بنائیں۔
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **ایک پائتھون پیکیج بنائیں**: `my_ros2_package` نامی ایک نیا پائتھون پیکیج بنائیں۔
    ```bash
    ros2 pkg create --build-type ament_python my_ros2_package
    ```

3.  **پیکیج ڈائریکٹری میں جائیں**:
    ```bash
    cd my_ros2_package
    ```

## 1. پبلشر نوڈ بنائیں

`~/ros2_ws/src/my_ros2_package/my_ros2_package/` میں `minimal_publisher.py` نامی ایک نئی فائل بنائیں۔

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

## 2. سبسکرائبر نوڈ بنائیں

اسی ڈائریکٹری میں `minimal_subscriber.py` نامی ایک نئی فائل بنائیں۔

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

## 3. `setup.py` کو اپ ڈیٹ کریں

`~/ros2_ws/src/my_ros2_package/setup.py` میں اپنے نئے ایگزیکیوٹیبلز کے لیے انٹری پوائنٹس شامل کریں۔

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

## 4. بنائیں اور چلائیں

1.  **اپنا پیکیج بنائیں**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_ros2_package
    ```

2.  **ورک اسپیس کو سورس کریں**:
    ```bash
    source install/setup.bash
    ```

3.  **پبلشر چلائیں**: ایک ٹرمینل میں:
    ```bash
    ros2 run my_ros2_package minimal_publisher
    ```

4.  **سبسکرائبر چلائیں**: دوسرے ٹرمینل میں:
    ```bash
    ros2 run my_ros2_package minimal_subscriber
    ```

آپ دیکھیں گے کہ پبلشر پیغامات بھیج رہا ہے اور سبسکرائبر انہیں وصول کرکے پرنٹ کر رہا ہے۔ مبارک ہو، آپ نے اپنے پہلے ROS 2 پبلشر اور سبسکرائبر نوڈز بنائے ہیں!
