# wrs
Project to Force Controle with an UR5 under ROS Kinetic.

Since I needed to change some other packges I uploaded the whole Workspace to Github.
To Clone th git. Go into your new ROS Catkin_Worksace and run.
`$ git clone https://github.com/jhaardt/wrs.git src`

To run the different Programms follow this order and run them in deiiferent Terminal Windows:

##Simulation
`$ roslaunch wrs_gazebo wrs_simu.launch`
`$ roslaunch wrs_moveit_config wrs_moveit_planning_execution.launch sim:=true`
`$ rosrun wrs_run pick_place_simu`

For further Informations you can run this two anylisy tools.
`$ roslaunch wrs_moveit_config moveit_rviz.launch config:=true`
`$ rqt>`

##Real
Start Universal Robot UR5 Simu:  
`$ ursim-3.5.4.10845/start-ursim.sh`

If the real robot is connected:
`$ roslaunch wrs_real_setup ur5_gripper.launch`
`$ roslaunch wrs_real_setup activate_gripper.launch`
`$ roslaunch wrs_moveit_config wrs_moveit_planning_execution.launch`
`$ roslaunch wrs_moveit_config moveit_rviz.launch config:=true`
`$ rosrun wrs_run pick_place_real`

###The Links to the used Packages are:

[gazebo_ros_link_attacher[GitHub]](https://github.com/pal-robotics/gazebo_ros_link_attacher)

[moveit_jog_arm[GitHub]](https://github.com/inmo-jang/moveit_jog_arm) (clone) Orignal Git dosen't exist anymore since it changed, while crealing this Project, to MoveIt Expremental and is now known as MoveIt Servo

[roboticsgroup_gazebo_plugins[GitHub]](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins)

[robotiq[GitHub]](https://github.com/ros-industrial/robotiq)

[kinetic-devel[GitHub]](https://github.com/machinekoder/ros_pytest/tree/kinetic-devel) (necessary for Jag Arm)

[universal_robot[GitHub]](https://github.com/ros-industrial/universal_robot)

[ur_modern_driver[GitHub]](https://github.com/ros-industrial/ur_modern_driver/tree/master)


###A big Help and Guideline was 
[intuitivecomputing/icl_phri_ur5[GitHub]](https://github.com/intuitivecomputing/icl_phri_ur5)

[philwall3/UR5-with-Robotiq-Gripper-and-Kinect[GitHub]](https://github.com/philwall3/UR5-with-Robotiq-Gripper-and-Kinect)

[intuitivecomputing/ur5_with_robotiq_gripper[GitHub]](https://github.com/intuitivecomputing/ur5_with_robotiq_gripper)
