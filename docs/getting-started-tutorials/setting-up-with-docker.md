# Setting up your PC with Docker on Windows

This guide will walk you through the process of setting up an Ubuntu environment and ROS workspace on your Windows PC using Docker, which will allow you to easily develop and test your ROS projects in a containerized environment. We will also cover how to connect VS Code to the Docker container and how to use X11 to run GUI Docker apps.

!!! warning "Note"
    This guide is designed for individuals who plan to use the Docker image. If you want to set up your PC and workspace on a host machine, please consult the instructions provided in [Setting up your workspace](../getting-started-tutorials/setting-up-your-workspace.md).

## Prerequisites
Before starting with the steps below, ensure that you have the following:

* [Docker Desktop](https://docs.docker.com/desktop/install/windows-install/#install-docker-desktop-on-windows){target=_blank} for Windows installed and running
* [Visual Studio Code](https://code.visualstudio.com/download){target=_blank} (VS Code) installed
* [Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git){target=_blank} installed

## Step 1: Install VcXsrv

1. Download and install [VcXsrv](https://sourceforge.net/projects/vcxsrv/){:target=_blank} on your computer.
2. Install VcXsrv by running the installer and following the prompts.
3. Once the installation is complete, open the VcXsrv by clicking on the Start menu and typing `XLaunch`.
4. In the XLaunch window, select the `Multiple windows` option and click `Next`.
    ![XLaunch step 1](assets/vcxsrv-install-1.png)
5. In the next window, select the `Start no client` option and click `Next`.
    ![XLaunch step 2](assets/vcxsrv-install-2.png)
6. In the next window, select the `Clipboard` option and click `Next`. Deselect the `Native opengl` option and select the `Disable access control` option.
    ![XLaunch step 3](assets/vcxsrv-install-3.png)
7. In the next window, click `Finish`.
    ![XLaunch step 4](assets/vcxsrv-install-4.png)

## Step 2: Creating the main folder and `docker-compose.yml`

1. Create a new folder in your preferred location and name it `PARC_docker`
2. Inside `PARC_docker` folder, create a ROS workspace by running the following command in Powershell:

    ```shell
    mkdir catkin_ws/src
    ```

3. Next, create a `docker-compose.yml` file inside `PARC_docker` folder. This file will contain the configuration for our ROS Docker container. Open the file in your favorite editor and add the following lines:

    ```yaml
    version: '3.8'
    services:
      ros:
        image: parcengineersleague/parc-2023
        container_name: parc-ros-docker-1
        environment:
          - DISPLAY=host.docker.internal:0.0
          - ROS_HOSTNAME=ros
          - ROS_MASTER_URI=http://ros:11311
        volumes:
          - ./catkin_ws:/catkin_ws
        ports:
          - "11311:11311"
        command: roscore
    ```
    This configuration will pull the official image for PARC 2023, if it does not already exist, and configure it for X11 server support. It will also mount the `catkin_ws` folder inside the container and start the `roscore` command when the container starts. 

4. Save the `docker-compose.yml` file and close your editor.

## Step 4: Building the Docker container

1. In a new Powershell window, navigate to the `PARC_docker` and run the command to build the Docker container:
    ```shell
    cd PARC_docker
    docker compose up -d
    ```

3. Once the container is built, you can verify that it's running by running the following command:

    ```shell
    docker ps
    ```

    You should see the following output:

    ```shell
    CONTAINER ID   IMAGE                          COMMAND                  CREATED          STATUS          PORTS                    NAMES
    db7df0798d9b   parcengineersleague/parc-2023   "/bin/bash -c 'sourc…"   20 seconds ago   Up 19 seconds    0.0.0.0:11311->11311/tcp   parc-ros-docker-1
    ```

## Step 5: Opening a terminal in the Docker container

1. To open a terminal in the Docker container, run the following command:

    ```shell
    docker exec -it parc-ros-docker-1 bash
    ```
    where `parc-ros-docker-1` is the name of the container. You can find the name of the container by running the `docker ps` command.

2. Once the terminal is open, you can verify that you are in the container by running the following command:

    ```shell
    echo $ROS_DISTRO
    ```

    You should see the following output:

    ```shell
    noetic
    ```
    This means that you are in the container and that the ROS distribution is set to Noetic.

## Step 6: Test installation

!!! note
    You might need to source the environment variables first using this command:
    
    `source /home/parc/catkin_ws/devel/setup.bash`

If you completed the preceding tasks successfully, you should be able to run this ROS launch command and see the Gazebo simulator and RViz simulator open with the following display:
```sh
roslaunch parc_robot task2.launch
```
![Gazebo Simulator window](assets/gazebo.png)
Gazebo Simulator window


![RViz window](assets/rviz.png)
RViz window


You need to `publish`/write to the `topic` `/cmd_vel` to move the robot.
In the following step, you will learn how to control the robot manually using your keyboard. Once you have tested that, you can follow the [Getting Started with ROS](../getting-started-with-ros) guide to learn how to write a program to control the robot.

## Step 7: Controlling the robot using keyboard
1. Open a new Powershell terminal and execute the Docker container as done above:
    ```shell
    docker exec -it parc-ros-docker-1 bash
    ```

2. Run the following command in the terminal:
    ```sh
    source /home/parc/catkin_ws/devel/setup.bash
    roslaunch parc_robot teleop.launch
    ```

Now keeping the second terminal on top (teleop.launch) press `i` to move the robot forward, you can see the robot moving in "RViz" and "Gazebo" windows.
you can use the keys shown below to move the robot and `k` key to stop the movement.
```sh
Moving around:
   u    i    o
   j    k    l
   m    ,    .
```

## Step 8: Developing inside the container with VSCode

1. Install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension in VSCode.

2. Click on the green icon in the bottom left corner of the VSCode window and select `Open Folder in Container...`.

3. Select the `catkin_ws` folder.

4. VSCode will now open the `catkin_ws` folder inside the container.

5. You can now use VSCode to edit files in the `catkin_ws` folder.

Alternatively, since we have already created a volume for the `catkin_ws` folder, you can also use your favorite editor to edit files in the `catkin_ws` folder on your host machine. The changes will be reflected inside the container. The advantage of using VSCode in the container is that you can use the integrated terminal to run commands inside the container.
