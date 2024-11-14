# Part 1 - Preparing the project workspace

1. Install needed tools
```bash
sudo apt install git -y
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep2 -y
```

2. Create a new directory for your workspace and "change directory" (`cd`) to it in terminal
* NOTE: The name of the source directory can be anything but we're making it `ros2_ws` for this tutorial.
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

3. Download the example starter package
```bash
git clone https://github.com/ros/ros_tutorials.git -b humble
```

4. Resolve dependencies
```bash
cd ..
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

5. Build the workspace
```bash
colcon build
```

You'll see a message along the lines of "All required rosdeps installed successfully"

6. In a **new terminal window**, run the following to start up the TurtleSim
```bash
cd ~/ros2_ws
source install/local_setup.bash
ros2 run turtlesim turtlesim_node
```

7. In a **new terminal window**, run the following to control the turtle in TurtleSim
```bash
cd ~/ros2_ws
source install/local_setup.bash
ros2 run turtlesim turtle_teleop_key
```

# Part 2 - Setting up the project
1. Create a new package
```bash
cd ~/ros2_ws
ros2 pkg create --build-type ament_python py_pubsub
```

2. Open VSCode, create an new file and copy this starter code for the publisher
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
        msg.data = 'Hello World: %d' % self.i
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

3. Create a new file and copy the starter code for the subscriber node
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