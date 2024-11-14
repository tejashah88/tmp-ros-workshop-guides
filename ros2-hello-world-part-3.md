# ROS 2 - Calculator

## Part 1 - Setting up service definitons

1. Open a new terminal and source the workspace
```bash
cd ~/ros2_helloworld_ws
source install/local_setup.bash
```

2. Create a new package for the service files
```bash
cd src
ros2 pkg create --build-type ament_cmake ros2_calculator_interfaces
```

3. Create extra folders for use with services
```bash
cd ros2_calculator_interfaces/
mkdir srv
```

4. Create a file for the 'process two numbers' service
```bash
touch srv/ProcessTwoNums.srv
```

5. Open VSCode within the working directory
```bash
code .
```

6a. Paste the following into `srv/ProcessTwoNums.srv`
```
int64 a
int64 b
int64 op
---
int64 result
```

7. Paste the following in `CMakeLists.txt`, just below the `# find_package(<dependency> REQUIRED)` line.
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ProcessTwoNums.srv"
)
```

![image](https://gist.github.com/assets/11187343/4f09a286-97cb-4aa2-a977-8e6fff42da1a)

Your final file should look like this:
```cmake
cmake_minimum_required(VERSION 3.8)
project(ros2_calculator_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ProcessTwoNums.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

8. Paste the following in `package.xml`, just below the `<buildtool_depend>ament_cmake</buildtool_depend>` line.
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

![image](https://gist.github.com/assets/11187343/0aa89a7d-6725-4109-b1ff-c281426d6fc1)

Your final file should look like this:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ros2_calculator_interfaces</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="ros2@todo.todo">ros2</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

9. Build the interface package you've just made
```bash
cd ~/ros2_helloworld_ws
colcon build --packages-select ros2_calculator_interfaces
```

10. In another terminal, verify that the services are correctly compiled (in a new terminal).
```bash
cd ~/ros2_helloworld_ws
source install/local_setup.bash

ros2 interface show ros2_calculator_interfaces/srv/ProcessTwoNums
```

![image](https://gist.github.com/assets/11187343/f5121508-a6c3-4c4e-953a-8b792ac43b70)

## Part 2 - Creating the calculator services

1. Create a new package for the calculator project
```bash
cd src
ros2 pkg create --build-type ament_python ros2_calculator
```

2. Create two files, one for the calculator node and one for consumer node.
```bash
cd ros2_calculator
touch ros2_calculator/calculator_node.py
touch ros2_calculator/consumer_node.py
```

5. Open VSCode within the working directory
```bash
code .
```

6. In `calculator_node.py`, copy and paste the following template code.
```python
from ros2_calculator_interfaces.srv import ProcessTwoNums

import rclpy
from rclpy.node import Node


class CalculatorServiceNode(Node):
    def __init__(self):
        super().__init__('calculator_service')
        self.srv = self.create_service(ProcessTwoNums, 'process_two_nums', self.process_two_nums_callback)


    def process_two_nums_callback(self, request, response):
        self.get_logger().info(f'Received numbers: {request.a} & {request.b} with operator {request.op}')

        # TODO: Implement logic
    

def main(args=None):
    rclpy.init(args=args)

    calculator_service_node = CalculatorServiceNode()

    rclpy.spin(calculator_service_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

7. In `consumer_node.py`, copy and paste the following code.
```python
from ros2_calculator_interfaces.srv import ProcessTwoNums
import sys
import rclpy
from rclpy.node import Node


class ConsumerNode(Node):
    def __init__(self):
        super().__init__('consumer_node')
        self.cli = self.create_client(ProcessTwoNums, 'process_two_nums')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = ProcessTwoNums.Request()


    def send_request(self, a, b, op):
        self.req.a = a
        self.req.b = b
        self.req.op = op
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    a = int(sys.argv[1])
    b = int(sys.argv[2])
    op = int(sys.argv[3])

    consumer_node = ConsumerNode()
    consumer_node.send_request(a, b, op)

    while rclpy.ok():
        rclpy.spin_once(consumer_node)

        if consumer_node.future.done():
            try:
                response = consumer_node.future.result()
            except Exception as e:
                consumer_node.get_logger().info('Service call failed %r' % (e,))
            else:
                consumer_node.get_logger().info(f'Input: (a = {consumer_node.req.a}, b = {consumer_node.req.b}, op = {consumer_node.req.op})')
                consumer_node.get_logger().info(f'Output: {response.result}')
            break

    consumer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

8. Run the service node.
```bash
python3 ros2_calculator/calculator_node.py
```

9. In a new terminal, run the client node. Substitute <a>, <b> and <op> with the corresponding nodes.
```bash
python3 ros2_calculator/consumer_node.py 1 1 1
```