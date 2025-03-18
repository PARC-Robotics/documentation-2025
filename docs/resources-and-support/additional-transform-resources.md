# Additional Resources on Transforms

## Introduction

To successfully solve the weed detection challenge (Task 2), you will need to understand how to use transforms. This page provides additional resources on transforms. While there might be alternative ways to solve the challenge, we suggest you consider these series of transformations to solve the challenge (see figure below for corresponding illustration):

![Transforms](assets/transforms.png)

### Transform 1 (Image frame to camera frame)
First, you will need to transform the weed locations from the camera perspective to the camera reference frame. This is done by projecting the 2D points (u,v -- pixels) in the image plane into the 3D cartesian points (x,y,z -- meters) using a perspective transformation.
To perform the transformation from image plane to cartesian coordinates in camera frame, you will need a few camera properties which you can find [here](https://github.com/PARC-Robotics/PARC-Engineers-League/blob/master/parc_robot/urdf/side_cameras.xacro).

Relevant resources for Transform 1 can be found here:

- [Camera Calibration and 3D Reconstruction](https://docs.opencv.org/2.4.13/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)
- [CS231A Course Notes 1: Camera Models](https://web.stanford.edu/class/cs231a/course_notes/01-camera-models.pdf)


### Transform 2 (Camera frame to robot frame)
Next, you will need to transform the weed locations from the camera reference frame to the robot reference frame. This would require making use of the ROS tf transformation module (see useful links below).

### Transform 3 (robot frame to world frame)
Finally, you will need to transform the weed locations from the robot reference frame to the world frame. For this, you should consider getting the GPS coordinates of the robot and converting that to cartesian coordinate in the world frame using the **gps2cartesian** module described in [Task 1](https://parc-robotics.github.io/documentation-2023/competition-instructions/phase-1/task-1-autonomous-field-navigation/#converting-gps-to-cartesian).



## Transform Resources

Here are some additional resources on coordinate transforms, especially in the context of robotics:

- [Transformations Part 1: Coordinate Transforms and Robotics](https://articulatedrobotics.xyz/transformations-1-coordinate_transforms/)
- [Transformations Part 2: Linear Transformations](https://articulatedrobotics.xyz/transformations-2-linear_transforms/)
- [Transformations Part 3: 2D Rotations](https://articulatedrobotics.xyz/transformations-3-rotation_matrices_2d/)
- [Transformations Part 4: Translations](https://articulatedrobotics.xyz/4-translations/)
- [Transformations Part 5: Affine Transformation Matrices](https://articulatedrobotics.xyz/5-transformation_matrices/)
- [Transformations Part 6: 3D Rotations](https://articulatedrobotics.xyz/6-rotations_3d/)
- [Coordinate Transformations in Robotics - MathWorks](https://www.mathworks.com/help/robotics/ug/coordinate-transformations-in-robotics.html)
- [ROS Wiki: tf](https://wiki.ros.org/tf)
- [ROS Wiki: tf2](https://wiki.ros.org/tf2)
- [Access the tf Transformation Tree in ROS](https://www.mathworks.com/help/ros/ug/access-the-tf-transformation-tree-in-ros.html)
