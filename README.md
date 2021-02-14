# wrs
## Introduction
This Workspace is the result of my master thesis. The goal of the project was to create a force controlled assembly process, that works in a simulated and a real environment.
I used the following robot setup: Universal Robots UR5 with an attached Robotiq 2F-140 gripper and ROS Kinetic Kame. The task was to fit a pin and a bearing housing into a mounting plate and originated from the assembly task of the 2018 World Robot Challenge.

This video shows the gazebo simulated UR5-Robot, searching the mounting hole and fitting the Pin into the mounting plate by only using the integrated force sensor in the TCP.

[![youtube](https://share.gifyoutube.com/wVXjZX.gif)](https://youtu.be/F9nsdSvd_QM)

(Unfortunately, due to COVID-19, it was not possible for me to get into the Lab again and take a Video of the real Robot).

To make it work, some ROS-Packages, other than my own, needed some changes. For that reason, I decided to upload the whole workspace to GitHub.

## Getting set up
This workspace was developed with ROS Kinetic Kame. Other versions of ROS aren't tested, but might work.

To clone the git repository, navigate to your new ROS Catkin_Workspace and run the following command:

`$ git clone https://github.com/jhaardt/wrs.git src`

## Running the code
To run the different programms, run the following commands in different terminal windows:

### Simulation

`$ roslaunch wrs_gazebo wrs_simu.launch`

`$ roslaunch wrs_moveit_config wrs_moveit_planning_execution.launch sim:=true`

`$ rosrun wrs_run pick_place_simu`


For further information about the assembly process, you can run these two analysis tools:

`$ roslaunch wrs_moveit_config moveit_rviz.launch config:=true`

`$ rqt`


### Real

If the real robot is not connected, start Universal Robot UR5-Sim first:  

`$ ursim-3.5.4.10845/start-ursim.sh`


If the real robot is connected:

`$ roslaunch wrs_real_setup ur5_gripper.launch`

`$ roslaunch wrs_real_setup activate_gripper.launch`

`$ roslaunch wrs_moveit_config wrs_moveit_planning_execution.launch`

`$ roslaunch wrs_moveit_config moveit_rviz.launch config:=true`

`$ rosrun wrs_run pick_place_real`

## Known Issues
- Simulation: While trying to assemble the bearing housing, the robot model breakes, when the housing touches the mounting plate a second time, due to simulation problems with Gazebo. 
- Real: While assembling the pin, the forces are getting to big and the pin winds out of the gripper.

## References
### The links to the used packages are:

[gazebo_ros_link_attacher [GitHub]](https://github.com/pal-robotics/gazebo_ros_link_attacher)

[moveit_jog_arm [GitHub]](https://github.com/inmo-jang/moveit_jog_arm) (clone) The orignal git dosen't exist anymore since it got changed, while creating this project. It has been moved to MoveIt Experimental and is now known as MoveIt Servo.

[roboticsgroup_gazebo_plugins [GitHub]](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins)

[robotiq [GitHub]](https://github.com/ros-industrial/robotiq)

[kinetic-devel [GitHub]](https://github.com/machinekoder/ros_pytest/tree/kinetic-devel) (necessary for Jag Arm)

[universal_robot [GitHub]](https://github.com/ros-industrial/universal_robot)

[ur_modern_driver [GitHub]](https://github.com/ros-industrial/ur_modern_driver/tree/master)


### A big help and guideline was: 
[intuitivecomputing/icl_phri_ur5 [GitHub]](https://github.com/intuitivecomputing/icl_phri_ur5)

[philwall3/UR5-with-Robotiq-Gripper-and-Kinect [GitHub]](https://github.com/philwall3/UR5-with-Robotiq-Gripper-and-Kinect)

[intuitivecomputing/ur5_with_robotiq_gripper [GitHub]](https://github.com/intuitivecomputing/ur5_with_robotiq_gripper)
