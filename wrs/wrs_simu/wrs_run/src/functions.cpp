#include "functions.h"

// ROS
#include <ros/ros.h>

//Units
#include <std_msgs/Int8.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//MoveIt
#include <moveit_msgs/ExecuteTrajectoryAction.h>
// For chaning speed in computeCartesianPath
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <controller_manager_msgs/SwitchController.h>

moveit_msgs::ExecuteTrajectoryActionFeedback mvgr_status;
void callback_status(const moveit_msgs::ExecuteTrajectoryActionFeedbackConstPtr& msg) { 
  mvgr_status = *msg;  
}

void step(ros::Publisher pub){
   std_msgs::Int8 pick_place_steps;  
   pick_place_steps.data= -30;
   pub.publish(pick_place_steps);
   pick_place_steps.data=50;
   pub.publish(pick_place_steps);
   pick_place_steps.data= -30;
   pub.publish(pick_place_steps);
}

void moveGripper(moveit::planning_interface::MoveGroupInterface& group, float value){
  const robot_state::JointModelGroup* joint_model_group =
  group.getCurrentState()->getJointModelGroup("Gripper");
  
  moveit::core::RobotStatePtr current_state = group.getCurrentState();
  std::vector<double> gripper_pos;
  current_state->copyJointGroupPositions(joint_model_group, gripper_pos);
  gripper_pos[0] = value;
  
  group.setJointValueTarget(gripper_pos);
  group.move();
}


//https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=308080#post-id-308080
void fixtrajectorytime(moveit_msgs::RobotTrajectory &trajectory){
  moveit_msgs::RobotTrajectory trajectory2;
  std::size_t  WayPointCount = trajectory.joint_trajectory.points.size();
  trajectory2.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[0]);
  for(std::size_t i=1; i < WayPointCount; i++){
    if (trajectory.joint_trajectory.points[i].time_from_start.sec > trajectory.joint_trajectory.points[i-1].time_from_start.sec){
      trajectory2.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[i]);
    }
    else{
      if (trajectory.joint_trajectory.points[i].time_from_start.nsec > trajectory.joint_trajectory.points[i-1].time_from_start.nsec){
	trajectory2.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[i]);  
      }
      else{
	ROS_INFO("Removed trajectory.point[%d]=%d.%ds ([%d]=%d.%ds) ",i,trajectory.joint_trajectory.points[i].time_from_start.sec,trajectory.joint_trajectory.points[i].time_from_start.nsec,i-1,trajectory.joint_trajectory.points[i-1].time_from_start.sec,trajectory.joint_trajectory.points[i-1].time_from_start.nsec);
      }
    }
  }
  trajectory.joint_trajectory.points.clear();
  trajectory.joint_trajectory.points = trajectory2.joint_trajectory.points;
}

void moveCartesian(moveit::planning_interface::MoveGroupInterface& group, float x, float y, float z, float speed){
  geometry_msgs::Pose target_pose;
  target_pose = group.getCurrentPose().pose;
  std::vector<geometry_msgs::Pose> waypoints;
  target_pose.position.x += x;
  target_pose.position.y += y;
  target_pose.position.z += z;
  waypoints.push_back(target_pose); 
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  
  //https://answers.ros.org/question/288989/moveit-velocity-scaling-for-cartesian-path/
  //https://groups.google.com/d/msg/moveit-users/x5FwalM5ruk/WIj2ruYs4RwJ
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(rt, 0.05*speed, 1.0);
  rt.getRobotTrajectoryMsg(trajectory);
  fixtrajectorytime(trajectory);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.trajectory_ = trajectory;
  group.execute(my_plan);
}

void changeController(std::string start, std::string stop){
  ros::AsyncSpinner spinner(1);  
  controller_manager_msgs::SwitchController Controller;
  Controller.request.start_controllers.push_back(start);
  Controller.request.stop_controllers.push_back(stop);
  Controller.request.strictness = 2;
  ros::service::call("/controller_manager/switch_controller",Controller);  
}

bool touch(moveit::planning_interface::MoveGroupInterface& group, double *fIN, double *fREF, float fLOW, float fUP, float depth){
    ROS_INFO("fREF = %f", *fREF); 
    geometry_msgs::Pose target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose = group.getCurrentPose().pose;
    target_pose.position.z -= depth;
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, 0.0005, 1.0);
    rt.getRobotTrajectoryMsg(trajectory);
    fixtrajectorytime(trajectory);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_ = trajectory;
    //https://answers.ros.org/question/280443/moveit-stop-command-usage/?answer=280555#post-id-280555
    //https://answers.ros.org/question/249995/how-to-check-sate-of-plan-execution-in-moveit-during-async-execution-in-python/
    group.asyncExecute(my_plan);
    while(mvgr_status.status.status == 3 && ros::ok()){}  // since the topic is to slow, we need to wait till it switches from 3(executed) to 1(in motion)
    while(*fIN > *fREF +fLOW && *fIN < *fREF +fUP && mvgr_status.status.status != 3 && ros::ok())
    {
    } 
    group.stop(); 
    ROS_INFO("f_stop = %f", *fIN);
    if (mvgr_status.status.status == 3){
    ROS_INFO("No Surface touched!");
    return true;
    }
    return false;
}

bool touchjog(ros::Publisher pub, moveit::planning_interface::MoveGroupInterface& group, double *fIN, double *fREF, float fLOW, float fUP, float depth){  
  geometry_msgs::Pose target_pose,current_pose;
  current_pose = group.getCurrentPose().pose;
  target_pose = group.getCurrentPose().pose;
  target_pose.position.z -= depth;
  
  ros::Rate loop_rate(100);
  geometry_msgs::TwistStamped TwistStamped;
    TwistStamped.header.stamp = ros::Time::now();
    TwistStamped.twist.linear.x = 0.0;
    TwistStamped.twist.linear.y = 0.0;
    TwistStamped.twist.linear.z = -0.001;
    TwistStamped.twist.angular.x = 0.0;
    TwistStamped.twist.angular.y = 0.0;
    TwistStamped.twist.angular.z = 0.0;
    pub.publish(TwistStamped);
    ros::spinOnce();
    sleep(1.5);
  ROS_INFO("Start Touch: fREF = %f fIN = %f ",*fREF , *fIN);
  
  while (*fIN > *fREF + fLOW && *fIN < *fREF + fUP && current_pose.position.z >= target_pose.position.z && ros::ok ())
  {  
    current_pose = group.getCurrentPose().pose;
    TwistStamped.header.stamp = ros::Time::now();
    TwistStamped.twist.linear.x = 0.0;
    TwistStamped.twist.linear.y = 0.0;
    TwistStamped.twist.linear.z = -0.001; // in m/s
    TwistStamped.twist.angular.x = 0.0;
    TwistStamped.twist.angular.y = 0.0;
    TwistStamped.twist.angular.z = 0.0;
    pub.publish(TwistStamped);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Stop Touch fREF = %f fIN = %f ",*fREF , *fIN);
    TwistStamped.header.stamp = ros::Time::now();
    TwistStamped.twist.linear.x = 0.0;
    TwistStamped.twist.linear.y = 0.0;
    TwistStamped.twist.linear.z = 0.0;
    TwistStamped.twist.angular.x = 0.0;
    TwistStamped.twist.angular.y = 0.0;
    TwistStamped.twist.angular.z = 0.0;
    pub.publish(TwistStamped);
    ros::spinOnce();
    
    if(current_pose.position.z < target_pose.position.z){
      ROS_INFO("No Surface touched!");
      return true;     
    }   
    return false;
}
  
  void spiral(moveit::planning_interface::MoveGroupInterface& group,int offset, float b,double *fIN, double *fREF, float fLOW, float fUP){
    float x1,x2=0,y1,y2=0,dx,dy,angle;
    geometry_msgs::Pose target_pose;
    target_pose = group.getCurrentPose().pose;
    int n =1;
    while(n <= 20 && ros::ok()){ 
      std::vector<geometry_msgs::Pose> waypoints;
      for (int i = 0; i < (n+offset)*12; i++)
      {
	angle = 2*M_PI/(n+offset)/12 * i;
	x1 = ( b * (((n+offset)-1) + angle/2/M_PI)) * cos(angle);
	y1 = ( b * (((n+offset)-1) + angle/2/M_PI)) * sin(angle);
	dx=x1-x2;
	dy=y1-y2;
	x2=x1;
	y2=y1;
	target_pose.position.x += dx;
	target_pose.position.y += dy;
	waypoints.push_back(target_pose);
      }
      moveit_msgs::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.2*b;
      group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
      rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
      
      
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      iptp.computeTimeStamps(rt, 0.006, 1.0);
      rt.getRobotTrajectoryMsg(trajectory);  
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      fixtrajectorytime(trajectory);
      my_plan.trajectory_ = trajectory;
      group.asyncExecute(my_plan);
      while(mvgr_status.status.status == 3 && ros::ok()){}  // since the topic is to slow, we need to wait till it switches from 3(executed) to 1(in motion)
      while(*fIN > *fREF+ fLOW && *fIN < *fREF +fUP && mvgr_status.status.status != 3 && ros::ok())
      {
      } 
      
      if (mvgr_status.status.status != 3){
	group.stop();
	ROS_INFO("Found Hole! ft_ref = %f ft_touch = %f", *fREF, *fIN);
	return;   
      }
      n++;
    }
  }
  
  void upright(moveit::planning_interface::MoveGroupInterface& group, float gripperlength, float  objectradius, float savetyZ){
    geometry_msgs::Pose target_pose;
    tf2::Quaternion orientation;
    tfScalar yaw,pitch,roll;
    target_pose = group.getCurrentPose().pose;
    tf2::fromMsg(target_pose.orientation,orientation);
    tf2::Matrix3x3 mat(orientation);
    mat.getRPY(yaw, pitch, roll);
    ROS_INFO("yaw= %f, pitch= %f, roll= %f ", yaw, pitch, roll);
    orientation.setRPY(0, M_PI ,0 );
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x += sin(pitch)*gripperlength-cos(pitch)*objectradius;
    target_pose.position.y += sin(yaw)*gripperlength;// !!!!! RP abweichung noch nicht implementiert
    target_pose.position.z += 2*gripperlength+gripperlength*cos(yaw)-gripperlength*cos(pitch)+savetyZ;
    moveCartesian(group,0,0,+0.025,1); // move upwards before uprighting, to not crash into the taskboard.
    group.setPoseTarget(target_pose);
    group.move();    
  }
  
  void fitjog(ros::Publisher pub,moveit::planning_interface::MoveGroupInterface& group, geometry_msgs::WrenchStamped *ft_massage, float force, float depth, float speedXY, float speedZslow, float speedZfast){
  geometry_msgs::Pose target_pose,current_pose;
  current_pose = group.getCurrentPose().pose;
  target_pose = group.getCurrentPose().pose;
  target_pose.position.z -= depth;
  ros::Rate loop_rate(100);
  geometry_msgs::TwistStamped TwistStamped;
  std::size_t  i=0;
  
  while (current_pose.position.z >= target_pose.position.z && ros::ok())
  {
    current_pose = group.getCurrentPose().pose;
    
    TwistStamped.header.stamp = ros::Time::now();
    //x
    if (ft_massage->wrench.force.x >force){TwistStamped.twist.linear.x = +speedXY;ROS_INFO("+x");}
    else{
      if (ft_massage->wrench.force.x <-force){TwistStamped.twist.linear.x = -speedXY;ROS_INFO("-x");}  
      else{TwistStamped.twist.linear.x = 0.0;ROS_INFO("x0");}
    }
    //y
    if (ft_massage->wrench.force.y >force){TwistStamped.twist.linear.y = +speedXY;ROS_INFO("+y");}
    else{
      if (ft_massage->wrench.force.y <-force){TwistStamped.twist.linear.y = -speedXY; ROS_INFO("-y");}  
      else{TwistStamped.twist.linear.y = 0.0;ROS_INFO("y0");}
    }
    //z
    if(TwistStamped.twist.linear.x ==0 && TwistStamped.twist.linear.y == 0)
    {
      if(i>30){TwistStamped.twist.linear.z = -speedZfast;ROS_INFO("z++");}
      else{TwistStamped.twist.linear.z = -speedZslow;ROS_INFO("z+");i++;}
    }
    
    else{TwistStamped.twist.linear.z = 0.0; ROS_INFO("z0");i=0;}
    //twist
    TwistStamped.twist.angular.x = 0;
    TwistStamped.twist.angular.y = 0;
    TwistStamped.twist.angular.z = 0;
    
    pub.publish(TwistStamped);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}