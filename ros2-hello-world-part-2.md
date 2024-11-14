# ROS 2 - Hello World Part 2 - Mouse Sensor

1. Open a new terminal and source the workspace
```bash
cd ~/ros2_helloworld_ws
source install/local_setup.bash
```

2. Create a new package
```bash
cd src
ros2 pkg create --build-type ament_python ros2_mouse_listener
```

3. Install extra "system" dependencies
```bash
sudo apt install python3-pynput
```

4. Create two files, one for the mouse sensor and one for the position blaster
```bash
cd ros2_mouse_listener
touch ros2_mouse_listener/mouse_node.py
touch ros2_mouse_listener/blaster_node.py
```

5. Open VSCode within the working directory
```bash
code .
```

6. In `mouse_node.py`, copy and paste the following template code. Fill out both functions below.
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray

from pynput.mouse import Controller


class MouseSensorNode(Node):
    def __init__(self):
        super().__init__('mouse_sensor_node')
        # TODO: Implement publisher logic
        

    def publish_position(self):
        # TODO: Implement publishing logic
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MouseSensorNode()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

7. In `blaster_node.py`, copy and paste the following template code. Fill out both functions below.
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32MultiArray


class BlasterNode(Node):
    def __init__(self):
        super().__init__('blaster_node')
        
        # TODO: Implement subscribing logic

    def on_mouse_reading(self, msg):
        # TODO: Implement callback
        pass


def main(args=None):
    rclpy.init(args=args)
    node = BlasterNode()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

8. Back to the terminal, make a launch file
```bash
mkdir launch
touch launch/launch.py
```

9. Copy and paste the following into the launch.py
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros2_mouse_listener",
            executable="mouse_sensor",
        ),
        Node(
            package="ros2_mouse_listener",
            executable="mouse_blaster",
        ),
    ])
```

10. In `ros2_mouse_listener/package.xml`, add the following lines underneath `<build_type>ament_python</build_type>`
```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>ros2launch</exec_depend>
```

![image](https://gist.github.com/assets/11187343/f49aa79f-e917-406e-99dd-713896ba2d93)

11. In `ros2_mouse_listener/setup.py`, edit the following lines near `entry_points`.
```python
entry_points={
        'console_scripts': [
                'mouse_sensor = ros2_mouse_listener.mouse_node:main',
                'mouse_blaster = ros2_mouse_listener.blaster_node:main',
        ],
},
```

![image](https://gist.github.com/assets/11187343/e5f31936-84b4-4d8b-8be4-6f270f861b65)


12. In `ros2_mouse_listener/setup.py`, edit the following lines.
At the top of the file, add the following imports.
```python
import os
from glob import glob
```

![image](https://gist.github.com/assets/11187343/3815126c-045a-4b5b-bb37-65469985a255)

Near the `data_files` section, make it look like the following.
```python
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
```

![image](https://gist.github.com/assets/11187343/17d78dc7-b149-4030-8505-d906a7e69d63)

Your final `ros2_mouse_listener/setup.py` should look like this.
```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_mouse_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'mouse_sensor = ros2_mouse_listener.mouse_node:main',
                'mouse_blaster = ros2_mouse_listener.blaster_node:main',
        ],
    },
)
```

12. Update the ROS package dependencies from your workspace
```bash
cd ~/ros2_helloworld_ws
rosdep install -i --from-path src --rosdistro humble -y
```

13. Build your package
```bash
colcon build --packages-select ros2_mouse_listener
```

## Method A - Running nodes individually
1. In a **brand new terminal window**, run the following to start up the mouse sensor node
```bash
cd ~/ros2_helloworld_ws
source install/local_setup.bash
ros2 run ros2_mouse_listener mouse_sensor
```

2. In a **brand new terminal window**, run the following to start up the mouse blaster node
```bash
cd ~/ros2_helloworld_ws
source install/local_setup.bash
ros2 run ros2_mouse_listener mouse_blaster
```

## Method B - Running nodes all at once
In a **brand new terminal window**, run the following to start up both nodes at the same time
```bash
cd ~/ros2_helloworld_ws
source install/local_setup.bash
ros2 launch ros2_mouse_listener launch
```