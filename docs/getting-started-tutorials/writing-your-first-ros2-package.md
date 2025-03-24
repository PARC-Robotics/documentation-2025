# Writing your First ROS 2 Package

Assuming your workspace, `~/ros2_ws/` in this instance, is completed following the steps in [setting up your workspace](../getting-started-tutorials/setting-up-your-workspace.md),
this should be your folder structure:

```
~/ros2_ws/
├── build/
│   ├── .
│   └── .
├── install/
│   ├── .
│   └── .
├── log/
│   ├── .
│   └── .
└── src/
    ├── CMakeLists.txt
    └── PARC2025-Engineers-League/
        ├── parc_robot/
        │   ├── .
        │   ├── .
        │   ├── CMakeLists.txt
        │   └── package.xml
        ├── .
        └── .
```

First navigate to the source folder in your workspace,
```shell
cd ~/ros2_ws/src
```

Then create a new ROS 2 Python package called `test_publisher` (for example) by running the command below,
```shell
ros2 pkg create test_publisher --build-type ament_python \
--dependencies rclpy std_msgs geometry_msgs
```

Change directory into the newly created ROS 2 Python package,

```shell
cd test_publisher/
```

The `test_publisher` package file structure is as follows,

```
├── package.xml
├── resource
│   └── test_publisher
├── setup.cfg
├── setup.py
├── test
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── test_publisher
    └── __init__.py
```

## Moving the Robot Programmatically

The [setting up your workspace](../getting-started-tutorials/setting-up-your-workspace.md) guide has already shown how to control the robot with keyboard using `teleop_twist_keyboard`.

This guide will help you to move the robot by publishing commands to the `/robot_base_controller/cmd_vel_unstamped` topic programmically using Python.

To do this, create a file, `robot_publisher.py` inside the `test_publisher` directory, which has the `__init__` file, and make it executable.

```shell
cd ~/ros2_ws/src/test_publisher/test_publisher
touch robot_publisher.py
chmod +x robot_publisher.py
```

!!! note
    You need to change the permission of the file to executable to be able to run (as done in the last command shown above).

Now open the file and copy and paste the following code inside:


```python
#!/usr/bin/env python3
"""
Script to move Robot
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MoveRobot(Node):
    def __init__(self):
        super().__init__("move_robot")
        # Create a publisher which can "talk" to Robot and tell it to move
        self.pub = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )

    def run(self):
        # Create a Twist message and add linear x and angular z values
        move_cmd = Twist()

        ######## Move Straight ########
        print("Moving Straight")
        move_cmd.linear.x = 0.5  # move in x axis at 0.5 m/s
        move_cmd.angular.z = 0.0

        now = time.time()
        # For the next 4 seconds publish cmd_vel move commands
        while time.time() - now < 4:
            self.pub.publish(move_cmd)  # publish to robot
            
        ######## Stop ########
        print("Stopping")
        # Assigning both to 0.0 stops the robot
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        now = time.time()
        # For the next 5 seconds publish cmd_vel move commands
        while time.time() - now < 5:
            self.pub.publish(move_cmd)

        ######## Rotating Counterclockwise ########
        print("Rotating")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.7  # rotate at 0.7 rad/s

        now = time.time()
        # For the next 15 seconds publish cmd_vel move commands
        while time.time() - now < 15:
            self.pub.publish(move_cmd)

        ######## Stop ########
        print("Stopping")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        now = time.time()
        # For the next 3 seconds publish cmd_vel move commands
        while time.time() - now < 3:
            self.pub.publish(move_cmd)

        print("Exit")


def main(args=None):
    rclpy.init(args=args)

    move_robot = MoveRobot()
    move_robot.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

This code will make the robot move straight for 4 seconds, stop for 5 seconds, rotate counterclockwise for 15 seconds and then stop.

## Compile and Run

!!! Note 
    We need to update the `setup.py` file in the ROS 2 package to include our new program. Add the following line in the`console_scripts` section of the `setup.py` file:

    ```python
    entry_points={
            'console_scripts': [
                    'move_robot = test_publisher.robot_publisher:main',
            ],
    },
    ```
    `move_robot`is the ROS 2 executable node, `test_publisher` is the name of the created Python ROS 2 package, `robot_publisher` is the Python file we just created and `main` is the function called from the Python file.

Run the following commands to compile the code,

```shell
cd ~/ros2_ws
colcon build
```

To see it working, first run the robot in simulation by running the following command in one terminal,

```shell
source ~/ros2_ws/install/setup.bash
ros2 launch parc_robot_bringup task_launch.py
```

And run the following commands in another terminal to run this new program,

```shell
source ~/ros2_ws/install/setup.bash
ros2 run test_publisher move_robot
```

If you have set up everything well, you should see the robot moving in Gazebo as below,

![publisher demo](assets/getting_started_demo.gif)
