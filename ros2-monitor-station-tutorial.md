Create a new package
```bash
cd ~/ros2_ws/src
ros2 pkg create monitor_station --build-type ament_python --dependencies rclpy
code monitor_station/
```

Install extra dependencies

```bash
sudo apt install python3-pynput
```

Starter Publisher Node Code

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ExamplePublisher(Node):
    def __init__(self):
        super().__init__('example_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = ExamplePublisher()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Starter Subscriber Node Code

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ExampleSubscriber(Node):
    def __init__(self):
        super().__init__('example_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10,
        )

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="monitor_station",
            executable="temperature_sensor",
        ),
        Node(
            package="monitor_station",
            executable="mouse_sensor",
        ),
        Node(
            package="monitor_station",
            executable="dashboard",
        ),
    ])
```