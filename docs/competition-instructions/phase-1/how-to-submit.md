# How to Submit

## Autonomy Track

Teams are expected to develop their solutions (ROS packages) in a 'solutions' folder inside the `~/ros2_ws/src` directory. You may have one or more ROS packages in this folder. 

This is the expected directory structure:

```
~/ros2_ws/src
.
├── parc_robot
│   ├── CMakeLists.txt
│   └── package.xml
├── parc_robot_bringup
│   ├── .
│   ├── .
│   ├── CMakeLists.txt
│   └── package.xml
├── parc_robot_description
│   ├── .
│   ├── .
│   ├── CMakeLists.txt
│   └── package.xml
├── .
├── .
└── <YOUR_SOLUTION_FOLDER>        # Zip this folder and submit
    ├── <your_ros_package1>
    │   ├── .
    │   ├── .
    │   ├── setup.py
    │   └── package.xml
    ├── <your_ros_package2>
    │   ├── .
    │   ├── .
    │   ├── setup.py
    │   └── package.xml
    ├── .
    ├── .
    └── README.md                   # Required
```

Follow these submission steps:

1. Prepare a README.md file following this format and store in solution folder (see [example](https://github.com/PARC-Robotics/PARC2025-Engineers-League/blob/main/resources/sample-submission-readme.md)):
    * Introduction Section: Briefly describe your approach
    * Dependencies: List all the packages installed and used in your solution
    * Task description and run command(s)
    * Challenges faced

2. Include all the packages (dependencies) used in your solution in your package's "package.xml" file ([see guide](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html){target=_blank})

3. Create simple short video demos of your solution. This can be done by taking a screen recording of your solution running in Gazebo. Please ensure the videos are less than 200 MB in size.

4. Zip your solution folder and upload the folder and the videos on the solution submission form (TO BE PROVIDED).

<!-- 4. Zip your solution folder and upload the folder and the videos on the [solution submission form](https://forms.gle/GwE7Tzm9FpYzUVQX9). -->

## Design Track

Teams that opt for the design track should follow these steps to submit their work:

* Create a folder and include these files: 
    * 3 View drawings of each part of your fertilizer implement with dimensional information.
    * 3D renderings of final implement design. This should be exported as a **.stp** or **.step** (STEP file).
    * 3D assembly drawings of the implement parts.
    * Bill of materials of the implement assembly parts.
    * A motion simulation video showing the implement in operation. Please ensure that the video is not be more than 1 minute long and less than 50 MB in size.
    * Design report which explains your design approach, considerations made, implement components and its working principle. The design report should include the team country and names of team members,starting with the team leader.


<!-- https://github.com/PARC-Robotics/PARC2025-Engineers-League/blob/main/resources/PARC-Eng-League-Agreement.pdf -->

* Zip the folder with the files listed above and upload the folder on the solution submission form (TO BE PROVIDED).