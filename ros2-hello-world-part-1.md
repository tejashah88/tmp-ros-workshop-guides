# Part 1 - Preparing the workspace

1. Install needed tools
```bash
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep2 -y

# !!! NEW Needed Tools !!!
sudo apt install python3-pip
pip install setuptools==58.2.0
```

2. Create a new directory for your workspace and "change directory" (`cd`) to it in terminal
* NOTE: The name of the source directory can be anything but we're making it `ros2_ws` for this tutorial.
```bash
mkdir -p ~/ros2_helloworld_ws/src
cd ~/ros2_helloworld_ws/src
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

6. In a **brand new terminal window**, run the following to start up the TurtleSim
```bash
cd ~/ros2_helloworld_ws
source install/local_setup.bash
ros2 run turtlesim turtlesim_node
```

7. In a **brand new terminal window**, run the following to control the turtle in TurtleSim
```bash
cd ~/ros2_helloworld_ws
source install/local_setup.bash
ros2 run turtlesim turtle_teleop_key
```

# Part 2 - Setting up the project
1. Open a new terminal and source the workspace
```bash
cd ~/ros2_helloworld_ws
source install/local_setup.bash
```

2. Create a new package
```bash
cd src
ros2 pkg create --build-type ament_python ros2_hello_world
```

3. Create two files, one for the publisher and one for the subscriber
```bash
cd ros2_hello_world
touch ros2_hello_world/talker_node.py
touch ros2_hello_world/listener_node.py
```

4. Open VSCode within the working directory
```bash
code .
```

5. In `talker_node.py`, copy and paste the following template code. Fill out both functions below.
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker_node')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

    def timer_callback(self):
        message = String()
        message.data = f'Hello World: {self.i}'

        self.publisher.publish(message)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    
    # Initialize the node and start it up
    talker_node = TalkerNode()
    rclpy.spin(talker_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    talker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

6. In `listener_node.py`, copy and paste the following template code. Fill out both functions below.
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        self.subscription = self.create_subscription(String, 'chatter', self.listener_callback, 10)
    

    def listener_callback(self, msg):
        print(msg.data)


def main(args=None):
    rclpy.init(args=args)
    
    # Initialize the node and start it up
    listener_node = ListenerNode()
    rclpy.spin(listener_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    listener_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

7. In `ros2_hello_world/package.xml`, add the following lines underneath `<build_type>ament_python</build_type>`
```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

![image](https://gist.github.com/assets/11187343/f49aa79f-e917-406e-99dd-713896ba2d93)

8. In `ros2_hello_world/setup.py`, edit the following lines.
```python
entry_points={
        'console_scripts': [
                'talker = ros2_hello_world.talker_node:main',
                'listener = ros2_hello_world.listener_node:main',
        ],
},
```

![image](https://gist.github.com/assets/11187343/e5f31936-84b4-4d8b-8be4-6f270f861b65)

9. Update the ROS package dependencies from your workspace
```bash
cd ~/ros2_helloworld_ws
rosdep install -i --from-path src --rosdistro humble -y
```

10. Build your package
```bash
colcon build --packages-select ros2_hello_world
```

11. In a **brand new terminal window**, run the following to start up the talker node
```bash
cd ~/ros2_helloworld_ws
source install/local_setup.bash
ros2 run ros2_hello_world talker
```

12. In a **brand new terminal window**, run the following to start up the listener node
```bash
cd ~/ros2_helloworld_ws
source install/local_setup.bash
ros2 run ros2_hello_world listener
```