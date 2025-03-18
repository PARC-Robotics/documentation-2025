# How to set up your workspace

In this tutorial, you will set up a directory on your ROS 2 enabled PC as your workspace for development and install the competition ROS 2 packages. Please follow the instructions below carefully.

!!! note
    This can ONLY be completed after you have set up your PC (by following the tutorial here: [Setting up your PC](../getting-started-tutorials/setting-up-your-pc.md)).

<!-- uncommment once we have docker setup -->
<!-- !!! note -->
<!--     If you are using a Docker container, you can skip this tutorial and follow the instructions in [Setting up your PC using Docker](../getting-started-tutorials/setting-up-with-docker.md) instead. -->

### Step 1: Setup ROS 2 workspace

Open a new terminal on your PC, then copy and paste the following one line at a time:
```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 2: Clone the repository

In the same terminal (or in a new one), copy and paste the following:
```sh
cd ~/ros2_ws/src
git clone https://github.com/PARC-Robotics/PARC2025-Engineers-League.git .
```

### Step 3: Install dependencies

The cloned project repository contains ROS packages that require certain dependencies before they can be used. These dependencies are specified in the respective `package.xml` file
of each package and are installed using [rosdep](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html){target=_blank} a command-line utility for identifying and installing 
dependencies to build or install a package. 

To install the project dependencies, un the same terminal (or in a new one), copy and paste the following:
```sh
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
```

### Step 4: Compile packages

The next step is to compile the installed packages using `colcon build`:
```sh
cd ~/ros2_ws
colcon build
```

### Step 5: Set up ROS 2 environment
The following command needs to be run in every new terminal you open to get access to ROS 2 commands:

```sh
source /opt/ros/jazzy/setup.bash
```

To avoid sourcing the ROS setup file every time you launch a new terminal, you can add the command to your shell startup script by executing these lines:

```sh
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

The `ros2_ws` workspace is an **overlay** on top of the ROS installation, which is known as the **underlay**, and similarly to use the package executables or libraries
in `ros2_ws`, the workspace will need to be sourced in every new terminal opened with this command:


```sh
source ~/ros2_ws/install/setup.bash
```

Likewise to avoid manually sourcing the workspace in each newly launched terminal, the command can also be added to shell startup script:

```sh
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

!!! note
    As you develop, it is good to set the environment variables whenever you run a `colcon build` command to compile changes to your packages. You can do that by:
    ```sh
    source ~/ros2_ws/install/setup.bash
    ```

### Step 6: Gazebo Harmonic installation

Gazebo Harmonic is the robot simulator used in the autonomy track of the competition. It can be installed [here](https://gazebosim.org/docs/harmonic/ros_installation/){target=_blank}.

!!! Note 
    The 3D visualizer for ROS, [`RViz`](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html){target=_blank}, is automatically installed when ROS 2 Jazzy was installed on your PC in the [setting up your PC](../getting-started-tutorials/setting-up-your-pc.md) tutorial.


### Step 7: Test installation

If you completed the preceding steps successfully, you should be able to run this ROS 2 launch command which also opens the Gazebo Harmonic and RViz windows:

```sh
ros2 launch parc_robot_bringup task1_launch.py
```
![Gazebo Harmonic window](assets/gazebo.png)
Gazebo Harmonic window


![RViz window](assets/rviz.png)
RViz window

### Step 8: Controlling the robot using a keyboard

To move the PARC AgRobot, you need to publish/write messages on the `robot_base_controller/cmd_vel_unstamped` topic.

First of all, the `teleop_twist_keyboard` ROS 2 package is installed which will enable us to use the keyboard to control the robot in a terminal as follows,

```sh
sudo apt install ros-jazzy-teleop-twist-keyboard
```

Then run the following command in a new terminal,

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap \
/cmd_vel:=/robot_base_controller/cmd_vel_unstamped
```

Now keeping this second terminal active (or on top) press `i` to move the robot forward, you can see the robot moving in the "RViz" and "Gazebo" windows.
You can use the keys shown below to move the robot and `k` key to stop the movement.

```sh
Moving around:
   u    i    o
   j    k    l
   m    ,    .
```

Next, let's create your first ROS2 package with a ROS 2 node to control the robot.