#ifndef FUNCTIONS_H_INCLUDED
#define FUNCTIONS_H_INCLUDED

// ROS
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//moveit
#include <moveit_msgs/ExecuteTrajectoryAction.h>


void callback_status(const moveit_msgs::ExecuteTrajectoryActionFeedbackConstPtr& msg);

/**
 * Gives out an Spike in form of a changeing std_msgs::Int8. Use to indicate a finished Step of your program in an graph.
 *  * Input: 	pub = publisher with an advertised <std_msgs::Int8> and a Topic that gets tracked
 */
void step(ros::Publisher pub);

/**
 * Open/Closes the Gripper to the set position.
 * Input: 	group = Movegroup of Gripper
 * 		value = requested positionvalue
 */
void moveGripper(moveit::planning_interface::MoveGroupInterface& group, float value);

/**
 * Fixes the order of given trajectory, by checking if the time is continues. If not deletes the second infringing waypoint.
 * https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=308080#post-id-308080
 * Input: 	trajectory = trajectory that is going to be checked/fixed
 */
void fixtrajectorytime(moveit_msgs::RobotTrajectory &trajectory);

/**
 * Move the Robot with cartesian movement (trajectory planing) to given xyz location. The Speed at the TCP is not constat, since the Robot works with jointspeeds. Uses pos_based_pos_traj_controller. 
 * Input: 	group = Movegroup of Robot
 * 		x y z = desired position in m from world center point.
 * 		speed = speed value
 */
void moveCartesian(moveit::planning_interface::MoveGroupInterface& group, float x, float y, float z, float speed);

/**
 * Changes the active ros controller
 * Input: 	start = Controller to start
 * 		stop =  Controller to stop
 */
void changeController(std::string start, std::string stop);

/**
 * moves the Robot slowly by cartesian Movement (trajectory planing) in the -z direction. Stops when fIn reaches fREF+fLOW or fREF+fUP. 
 * Input: 	group = Movegroup of Robot
 * 		fIN = Input Force 
 * 		fREF =  Reference Force, can be constant or updating 
 * 		fLOW = Lower Force Limit to fREF eg. (-3 = -3N)
 * 		fUP = Upper Forece Limit to fREF eg. (+5 = 5N)
 * 		depth = How deep the robot should touch in m, before stopping (maybe no surface to touch).
 * Output:	Error = true - no surface touched
 * 			false - no error
 */
bool touch(moveit::planning_interface::MoveGroupInterface& group,double *fIN, double *fREF, float fLOW, float fUP, float depth);

/**
 * moves the Robot slowly in the -z direction by using trajectory planing . Stops when fIn reaches fREF+fLOW or fREF+fUP. 
 * Input: 	group = Movegroup of Robot
 * 		fIN = Input Force 
 * 		fREF =  Reference Force, can be constant or updating 
 * 		fLOW = Lower Force Limit to fREF eg. (-3 = -3N)
 * 		fUP = Upper Forece Limit to fREF eg. (+5 = 5N)
 * 		depth = How deep the robot should touch in m, before stopping (maybe no surface to touch).
 * Output:	Error = true - no surface touched
 * 			false - no error
 */
bool touchjog(ros::Publisher pub, moveit::planning_interface::MoveGroupInterface& group, double *fIN, double *fREF, float fLOW, float fUP, float depth);

/**
 * moves the Robot slowly in an archimedean spiral. Stops when fIn reaches fREF+fLOW or fREF+fUP. 
 * Input: 	pub = Publisher to publish the <geometry_msgs::TwistStamped> for the jogserver.
 * 		group = Movegroup of Robot
 * 		offset = starting distance from centerpoint (offeset * b)
 * 		b = distance betwen rotations
 * 		fIN = Input Force 
 * 		fREF =  Reference Force, can be constant or updating 
 * 		fLOW = Lower Force Limit to fREF eg. (-3 = -3N)
 * 		fUP = Upper Forece Limit to fREF eg. (+5 = 5N)
 */
void spiral(moveit::planning_interface::MoveGroupInterface& group,int offset, float b,double *fIN, double *fREF, float fLOW, float fUP);

/**
 * Uprights the tilted Gripper
 * Input: 	group = Movegroup of Robot
 * 		gripperlength = Length of the gripper + object
 * 		objectradius = Radius of the Object at the contact point
 * 		savetyZ = ammount the gripper moves up to prevent possible collision in meter
 */
void upright(moveit::planning_interface::MoveGroupInterface& group, float gripperlength, float  objectradius, float savetyZ);

/**
 * Moves the Gripper down and places the object into the hole. It compensate for eventual forces in x and y dirction. It starts slow, if no XY correction
 * took place in 30 cycles it switches to the fast speed till an correction takes place.
 * Input: 	pub = Publisher to publish the <geometry_msgs::TwistStamped> for the jogserver.
 * 		group = Movegroup of Robot
 * 		ft_massage = input from the force sensor
 * 		force = max force that is allowed before the robot tries to compensate. 
 * 		depth = how deep the Object should be fitted.
 * 		speedXY = how fast the robot tries to compensate in XY direction.
 * 		speedZslow = the speed the Object gets fitted slowly.
 * 		speedZfast = the speed the Object gets fitted fast after 30 cycles.
 */
void fitjog(ros::Publisher pub,moveit::planning_interface::MoveGroupInterface& group, geometry_msgs::WrenchStamped *ft_massage, float force, float depth, float speedXY, float speedZslow, float speedZfast);

#endif
