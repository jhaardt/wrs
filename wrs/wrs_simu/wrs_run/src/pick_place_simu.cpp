
// ROS
#include <ros/ros.h>
// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Link_Attacher
#include <gazebo_ros_link_attacher/Attach.h>

//For adding CollionObjects
#include <geometric_shapes/shape_operations.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

#include <functions.h>

geometry_msgs::WrenchStamped ft_massage;
void callback_ft (const geometry_msgs::WrenchStampedConstPtr& msg){
   ft_massage = *msg;
}

geometry_msgs::WrenchStamped ft_massage_mean_5;
void callback_ft_mean_5 (const geometry_msgs::WrenchStampedConstPtr& msg){
   ft_massage_mean_5 = *msg;
}

geometry_msgs::WrenchStamped ft_massage_mean_50;
void callback_ft_mean_50 (const geometry_msgs::WrenchStampedConstPtr& msg){
   ft_massage_mean_50 = *msg; 
}

std_msgs::Float64 ft_massage_mean_conv_5;
void callback_ft_mean_conv_5 (const std_msgs::Float64Ptr& msg){
   ft_massage_mean_conv_5 = *msg;
}

std_msgs::Float64 ft_massage_mean_conv_50;
void callback_ft_mean_conv_50 (const std_msgs::Float64Ptr& msg){
   ft_massage_mean_conv_50 = *msg; 
}


void gazeboAttach(std::string m1, std::string l1, std::string m2, std::string l2){
  ros::AsyncSpinner spinner(1);
  gazebo_ros_link_attacher::AttachRequest req;
  req.model_name_1 = m1;
  req.link_name_1 = l1;
  req.model_name_2 = m2;
  req.link_name_2 = l2;
  gazebo_ros_link_attacher::AttachResponse res;
  ros::service::call("/link_attacher_node/attach",req, res);
}  

void gazeboDetach(std::string m1, std::string l1, std::string m2, std::string l2){
  ros::AsyncSpinner spinner(1);
  gazebo_ros_link_attacher::AttachRequest req;
  req.model_name_1 = m1;
  req.link_name_1 = l1;
  req.model_name_2 = m2;
  req.link_name_2 = l2;
  gazebo_ros_link_attacher::AttachResponse res;
  ros::service::call("/link_attacher_node/detach",req, res);
}  

void movePosition(moveit::planning_interface::MoveGroupInterface& group, int pose){
  geometry_msgs::Pose target_pose;
  tf2::Quaternion orientation;
  switch(pose){
    case 1 : //above Pin (10 cm)
      target_pose.orientation.x = 0;
      target_pose.orientation.y = 1;
      target_pose.orientation.z = 0;
      target_pose.orientation.w = 0;
      target_pose.position.x = 0.6;
      target_pose.position.y = 0.1;
      target_pose.position.z = 0.371;
      break;
    case 2 : // Pin
      target_pose.orientation.x = 0;
      target_pose.orientation.y = 1;
      target_pose.orientation.z = 0;
      target_pose.orientation.w = 0;
      target_pose.position.x = 0.65;
      target_pose.position.y = 0.1;
      target_pose.position.z = 0.271;
      break;
    case 3 : // Rotate 30°
      target_pose = group.getCurrentPose().pose;
      orientation.setRPY(0, M_PI *5/6, 0);
      target_pose.orientation = tf2::toMsg(orientation);
      break;
    case 4 : // Above plate
      orientation.setRPY(0, M_PI *5/6, 0);
      target_pose.orientation = tf2::toMsg(orientation);
      target_pose.position.x = 0.62; //0.62
      target_pose.position.y = -0.065;  //0.07
      target_pose.position.z = 0.4;
      break;
    case 5 : //above Bearing (10 cm)
      target_pose.orientation.x = 0;
      target_pose.orientation.y = 1;
      target_pose.orientation.z = 0;
      target_pose.orientation.w = 0;
      target_pose.position.x = 0.70;
      target_pose.position.y = 0.1;
      target_pose.position.z = 0.369;
      break;
    case 6 : // Above plate Bearing
      orientation.setRPY(0, M_PI *5/6, 0);
      target_pose.orientation = tf2::toMsg(orientation);
      target_pose.position.x = 0.62; //0.62
      target_pose.position.y = -0.31;  
      target_pose.position.z = 0.4;
      break;
      
    default: break;}
    group.setPoseTarget(target_pose);  
    group.move();
}

void movejogstart(){
  changeController("joint_group_position_controller","arm_controller");
}

void movejogstop(){
  changeController("arm_controller","joint_group_position_controller");
}
  
  void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){ 
    tf2::Quaternion orientation;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(4);
    
    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = "world";
    
    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 2;
    collision_objects[0].primitives[0].dimensions[1] = 2;
    collision_objects[0].primitives[0].dimensions[2] = 0.1;
    
    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = -0.05;
    // END_SUB_TUTORIAL
    
    collision_objects[0].operation = collision_objects[0].ADD;
    
    //https://answers.ros.org/question/245995/adding-collision-object-in-moveit/
    collision_objects[1].id = "taskboard";
    collision_objects[1].header.frame_id = "world";
    Eigen::Vector3d scale(0.001, 0.001, 0.001);
    shapes::Mesh* m = shapes::createMeshFromResource("package://wrs_description/meshes/collision/taskboard.stl",scale);
    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;  
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
    collision_objects[1].meshes.resize(1);
    collision_objects[1].mesh_poses.resize(1);
    collision_objects[1].meshes[0] = mesh; 
    orientation.setRPY(M_PI/2, 0, -M_PI/2);
    collision_objects[1].mesh_poses[0].orientation= tf2::toMsg(orientation);
    collision_objects[1].mesh_poses[0].position.x = 0.8;
    collision_objects[1].mesh_poses[0].position.y = 0.0;
    collision_objects[1].mesh_poses[0].position.z = 0.03; 
    collision_objects[1].meshes.push_back(mesh);
    collision_objects[1].mesh_poses.push_back(collision_objects[1].mesh_poses[0]);
    collision_objects[1].operation = collision_objects[1].ADD;
    
    collision_objects[2].id = "frame";
    collision_objects[2].header.frame_id = "world";
  //  Eigen::Vector3d scale(0.001, 0.001, 0.001); // Declared in Object 1
    m = shapes::createMeshFromResource("package://wrs_description/meshes/collision/frame_collision.stl",scale);
 //   shape_msgs::Mesh mesh;
 //   shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
    collision_objects[2].meshes.resize(1);
    collision_objects[2].mesh_poses.resize(1);
    collision_objects[2].meshes[0] = mesh; 
    orientation.setRPY(0, 0, -M_PI/2);
    collision_objects[2].mesh_poses[0].orientation= tf2::toMsg(orientation);
    collision_objects[2].mesh_poses[0].position.x = 0.145;
    collision_objects[2].mesh_poses[0].position.y = 0.525;
    collision_objects[2].mesh_poses[0].position.z = 0; 
    collision_objects[2].meshes.push_back(mesh);
    collision_objects[2].mesh_poses.push_back(collision_objects[2].mesh_poses[0]);
    collision_objects[2].operation = collision_objects[2].ADD;
    
    
    collision_objects[3].id = "robot_base";
    collision_objects[3].header.frame_id = "world";
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions.resize(3);
    collision_objects[3].primitives[0].dimensions[0] = 0.6;
    collision_objects[3].primitives[0].dimensions[1] = 0.6;
    collision_objects[3].primitives[0].dimensions[2] = 0.02;
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = -0.225;
    collision_objects[3].primitive_poses[0].position.y = 0.225;
    collision_objects[3].primitive_poses[0].position.z = 0.01;  
    collision_objects[3].operation = collision_objects[3].ADD;
       
    planning_scene_interface.applyCollisionObjects(collision_objects);
  }
  
  int main(int argc, char** argv){
    ros::init(argc, argv, "pick_place_simu");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Subscriber pick_place_ft_mean_5 = nh.subscribe("/wrench_mean_5", 200, callback_ft_mean_5);
    ros::Subscriber pick_place_ft_mean_50 = nh.subscribe("/wrench_mean_50", 200, callback_ft_mean_50);
    ros::Subscriber pick_place_ft_mean_conv_5 = nh.subscribe("/wrench_mean_conv_5", 200, callback_ft_mean_conv_5);
    ros::Subscriber pick_place_ft_mean_conv_50 = nh.subscribe("/wrench_mean_conv_50", 200, callback_ft_mean_conv_50);
    ros::Subscriber pick_place_movestatus =nh.subscribe("/execute_trajectory/feedback",5,callback_status);
    ros::Subscriber pick_place_ft =nh.subscribe("/wrench_world",1000,callback_ft);
    
    ros::Publisher jog_pub = nh.advertise<geometry_msgs::TwistStamped>("/jog_server/delta_jog_cmds", 100);
    ros::Publisher steps_pub = nh.advertise<std_msgs::Int8>("/pick_place_steps", 100);
    
    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface mvgr_manipulator("manipulator");
    moveit::planning_interface::MoveGroupInterface mvgr_gripper("Gripper");
    mvgr_manipulator.setPlanningTime(45.0);
    //mvgr_manipulator.setNumPlanningAttempts(3);
    
    addCollisionObjects(planning_scene_interface); 
    double fRef;
    
    // PIN
    movejogstop();
    movePosition(mvgr_manipulator,1);
    moveCartesian(mvgr_manipulator,0,0,-0.1,1);
    moveGripper(mvgr_gripper, 0.889);
    gazeboAttach("robot","gripper_right_inner_finger","wrs_pin","pin_link");
    moveCartesian(mvgr_manipulator,0,0,+0.1,1);
    movePosition(mvgr_manipulator,3); 
    movePosition(mvgr_manipulator,4);
    moveCartesian(mvgr_manipulator,0,0,-0.12,1);
    
    step(steps_pub);
    fRef = ft_massage_mean_5.wrench.force.z;
    touch(mvgr_manipulator,&ft_massage_mean_5.wrench.force.z,&fRef,-0.5,+0.5,0.02);
    
    step(steps_pub);
    spiral(mvgr_manipulator,0,0.005,&ft_massage_mean_conv_5.data,&ft_massage_mean_conv_50.data,-0.5,+0.4);
    
    step(steps_pub);
    upright(mvgr_manipulator,0.271,0.003,0.005);
    
    step(steps_pub);
    fRef = ft_massage_mean_5.wrench.force.z;
    touch(mvgr_manipulator,&ft_massage_mean_5.wrench.force.z,&fRef,-3,+1,0.02);
    
    step(steps_pub);
    fRef = 0;
    spiral(mvgr_manipulator,0,0.001,&ft_massage_mean_5.wrench.force.z,&fRef,-1,+200);
    
    step(steps_pub);
    movejogstart();
    fitjog(jog_pub,mvgr_manipulator,&ft_massage,0.4,0.02,0.001,0.0002,0.02);
    movejogstop();
    
    step(steps_pub);
    gazeboDetach("robot","gripper_right_inner_finger","wrs_pin","pin_link");
    moveGripper(mvgr_gripper, 0.0);
      
      // Bearinghousing
      
     movePosition(mvgr_manipulator,5); 
     moveCartesian(mvgr_manipulator,0,0,-0.1,1);
     moveGripper(mvgr_gripper, 0.685);
     gazeboAttach("robot","gripper_right_inner_finger","wrs_bearinghousing","bearinghousing_link");
     moveCartesian(mvgr_manipulator,0,0,+0.1,1);
     movePosition(mvgr_manipulator,3);
     movePosition(mvgr_manipulator,6);
     moveCartesian(mvgr_manipulator,0,0,-0.115,1);
     fRef = ft_massage_mean_5.wrench.force.z;
     touch(mvgr_manipulator,&ft_massage_mean_5.wrench.force.z,&fRef,-5,+1,0.02);
     fRef = ft_massage_mean_5.wrench.force.z;
     spiral(mvgr_manipulator,0,0.02,&ft_massage_mean_5.wrench.force.z,&fRef,-1,+5);
     upright(mvgr_manipulator,0.269,0.018,0.005);
//      touch(mvgr_manipulator,-3,+1);
    movejogstart();
    
    step(steps_pub);
    touchjog(jog_pub,mvgr_manipulator,&ft_massage_mean_5.wrench.force.z,&ft_massage_mean_50.wrench.force.z,-5, +0.2,0.02);
    movejogstop();
    
    step(steps_pub);
    spiral(mvgr_manipulator,178,0.0001,&ft_massage_mean_5.wrench.force.z,&ft_massage_mean_50.wrench.force.z,-1,+200);
    
    step(steps_pub);
     movejogstart();
     fitjog(jog_pub,mvgr_manipulator,&ft_massage,0.5,0.015,0.001,0.0002,0.02);
     movejogstop();
     gazeboDetach("robot","gripper_right_inner_finger","wrs_bearinghousing","bearinghousing_link");
     moveGripper(mvgr_gripper, 0.0);
     

      
    return 0;
  }
  
  
  
