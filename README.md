# wrs
Project to Force Controle with an UR5 under ROS Kinetic.

Since I needed to change some other packges I uploaded the whole Workspace to Github.
To Clone th git. Go into your new ROS Catkin_Worksace and run.
$ git clone https://github.com/jhaardt/wrs.git src

To run the different Programms follow this order and run them in deiiferent Terminal Windows:

-----Simulation-----
$ roslaunch wrs_gazebo wrs_simu.launch
$ roslaunch wrs_moveit_config wrs_moveit_planning_execution.launch sim:=true
$ rosrun wrs_run pick_place_simu

#For further Informations you can run this two anylisy tools.
$ roslaunch wrs_moveit_config moveit_rviz.launch config:=true
$ rqt

-----Real-----
Start Universal Robot UR5 Simu:  
$ ursim-3.5.4.10845/start-ursim.sh 

If the real robot is connected:
$ roslaunch wrs_real_setup ur5_gripper.launch
$ roslaunch wrs_real_setup activate_gripper.launch
$ roslaunch wrs_moveit_config wrs_moveit_planning_execution.launch 
$ roslaunch wrs_moveit_config moveit_rviz.launch config:=true
$ rosrun wrs_run pick_place_real

The Links to the used Packages are:
https://github.com/pal-robotics/gazebo_ros_link_attacher
https://github.com/inmo-jang/moveit_jog_arm (clone) Orignal Git dosen't exist anymore since it changed, while crealing this Project, to MoveIt Expremental and is now known as MoveIt Servo
https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins
https://github.com/ros-industrial/robotiq
https://github.com/machinekoder/ros_pytest/tree/kinetic-devel (neccesary for Jag Arm)
https://github.com/ros-industrial/universal_robot
https://github.com/ros-industrial/ur_modern_driver/tree/master

A big Help and Guideline was 
https://github.com/intuitivecomputing/icl_phri_ur5
https://github.com/philwall3/UR5-with-Robotiq-Gripper-and-Kinect
https://github.com/intuitivecomputing/ur5_with_robotiq_gripper
