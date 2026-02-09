---
sidebar_position: 9
---

# ROS 2 لانچ فائلز اور پیرامیٹر مینجمنٹ

جیسے جیسے آپ کے روبوٹک سسٹمز کی پیچیدگی بڑھتی ہے، ہر نوڈ کو ایک علیحدہ ٹرمینل میں چلانا مشکل اور ناقابل انتظام ہو جاتا ہے۔ ROS 2 ایک طاقتور **لانچ سسٹم** فراہم کرتا ہے جو آپ کو ایک ہی کمانڈ سے بیک وقت متعدد نوڈز کو شروع اور کنفیگر کرنے کی اجازت دیتا ہے۔ مزید برآں، لانچ سسٹم **پیرامیٹرز** کو منظم کرنے کا ایک مضبوط طریقہ فراہم کرتا ہے، جو آپ کے نوڈز کے لیے قابل ترتیب سیٹنگز ہیں۔

## ROS 2 لانچ فائل کیا ہے؟

ایک لانچ فائل ایک اسکرپٹ (عام طور پر پائتھون میں لکھی جاتی ہے) ہے جو نوڈز کے سیٹ کو چلانے کا طریقہ بیان کرتی ہے۔ لانچ فائل کے ذریعے، آپ یہ کر سکتے ہیں:
- ایک ساتھ متعدد نوڈز شروع کریں۔
- اگر نوڈز کریش ہو جائیں تو خود بخود دوبارہ شروع کریں۔
- ٹاپک، سروس، یا ایکشن کے ناموں کو دوبارہ نقشہ (Remap) کریں۔
- اپنے نوڈز کے لیے پیرامیٹرز سیٹ کریں۔

## پائتھون لانچ فائل بنانا

لانچ فائلز عام طور پر آپ کے ROS 2 پیکیج کے اندر ایک `launch` ڈائریکٹری میں رکھی جاتی ہیں۔

یہ ایک سادہ پائتھون لانچ فائل کی مثال ہے جو دو نوڈز شروع کرتی ہے: ایک ٹاکر (talker) اور ایک لسنر (listener)۔

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

-   **`generate_launch_description()`**: یہ وہ اہم فنکشن ہے جسے `ros2 launch` کمانڈ تلاش کرتی ہے۔ اسے ایک `LaunchDescription` آبجیکٹ واپس کرنا چاہیے۔
-   **`LaunchDescription`**: یہ آبجیکٹ ان تمام اداروں (جیسے نوڈز) کے لیے ایک کنٹینر ہے جنہیں آپ لانچ کرنا چاہتے ہیں۔
-   **`Node`**: یہ عمل ایک واحد ROS 2 نوڈ کی نمائندگی کرتا ہے۔ آپ پیکیج کو مخصوص کرتے ہیں جس سے اس کا تعلق ہے، اس کا `executable` نام (جیسا کہ `setup.py` میں بیان کیا گیا ہے)، اور آپ اسے ایک کسٹم `name` تفویض کر سکتے ہیں۔

اس لانچ فائل کو چلانے کے لیے، آپ یہ عمل کریں گے:
```bash
ros2 launch my_package my_launch_file.py
```

## پیرامیٹر مینجمنٹ

پیرامیٹرز آپ کے نوڈز کو ان کے سورس کوڈ کو تبدیل کیے بغیر کنفیگر کرنے کا ایک طریقہ ہیں۔ آپ انہیں ایسی سیٹنگز کے طور پر سوچ سکتے ہیں جنہیں آپ رن ٹائم پر یا لانچ ٹائم پر تبدیل کر سکتے ہیں۔

### پائتھون نوڈ میں پیرامیٹرز کا اعلان کرنا

سب سے پہلے، آپ کو ان پیرامیٹرز کا اعلان کرنا ہوگا جنہیں آپ کا نوڈ قبول کرتا ہے۔ یہ عام طور پر نوڈ کے کنسٹرکٹر میں کیا جاتا ہے۔

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_parameter_node')
        
        # 'my_parameter' نامی ایک پیرامیٹر کا اعلان کریں جس کی ڈیفالٹ ویلیو ہو۔
        self.declare_parameter('my_parameter', 'default_value')

        # پیرامیٹر کی ویلیو حاصل کریں۔
        my_param_value = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info(f'My parameter is: {my_param_value}')
```
آپ مختلف قسم کے پیرامیٹرز کا اعلان کر سکتے ہیں، جیسے سٹرنگز، انٹیجرز، بولینز، اور فلوٹس۔

### لانچ فائل میں پیرامیٹرز سیٹ کرنا

پیرامیٹرز کو سیٹ کرنے کا سب سے عام طریقہ آپ کی لانچ فائل کے ذریعے ہے۔ یہ آپ کو اپنی تمام نوڈ کنفیگریشنز کو ایک جگہ منظم کرنے کی اجازت دیتا ہے۔

آپ YAML فائل کا استعمال کرکے `Node` ایکشن میں پیرامیٹرز پاس کر سکتے ہیں۔

**`my_package/config/my_params.yaml`**
```yaml
my_parameter_node:
  ros__parameters:
    my_parameter: "a_new_value_from_yaml"
```

**نوٹ**: ساخت `node_name` -> `ros__parameters` -> `param_name: value` ہے۔

اب، آپ نوڈ کو اس YAML فائل کو لوڈ اور پاس کرنے کے لیے اپنی لانچ فائل میں ترمیم کر سکتے ہیں۔

**`my_package/launch/my_param_launch.py`**
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # پیکیج کی شیئر ڈائریکٹری کا راستہ حاصل کریں۔
    pkg_share = get_package_share_directory('my_package')
    # پیرامیٹر فائل کا راستہ حاصل کریں۔
    param_file = os.path.join(pkg_share, 'config', 'my_params.yaml')

    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_parameter_node',
            name='my_parameter_node',
            parameters=[param_file]  # پیرامیٹر فائل پاس کریں۔
        ),
    ])
```

جب آپ یہ لانچ فائل چلاتے ہیں، تو آپ کا نوڈ `my_parameter` کو اس کی ڈیفالٹ ویلیو کے بجائے `"a_new_value_from_yaml"` پر سیٹ کر کے شروع ہوگا۔

لانچ فائلز اور پیرامیٹرز کا استعمال پیچیدہ، قابل ترتیب، اور دوبارہ قابل استعمال روبوٹک ایپلیکیشنز بنانے کے لیے ضروری ہے۔ یہ آپ کے کوڈ ("کیسے") کو آپ کی کنفیگریشن ("کیا") سے الگ کرتا ہے، جو اچھے سافٹ ویئر انجینئرنگ کا ایک بنیادی اصول ہے۔