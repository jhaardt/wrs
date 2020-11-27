/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_kdl.h>

#define _USE_MATH_DEFINES
class echoListener
{
public:

  tf::TransformListener tf;

  //constructor with name
  echoListener()
  {

  }

  ~echoListener()
  {

  }
  
private:

};

geometry_msgs::WrenchStamped ft_massage;
void callback_ft (const geometry_msgs::WrenchStampedConstPtr& msg){
   ft_massage = *msg;
}

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "tf_echo", ros::init_options::AnonymousName);

  ros::NodeHandle nh("~");
      ros::AsyncSpinner spinner(1);
    spinner.start();

  ros::Rate rate(200);

  //Instantiate a local listener
  echoListener echoListener;

  ros::Subscriber pick_place_ft =nh.subscribe("/wrench",1000,callback_ft);
  ros::Publisher ft_pub = nh.advertise<geometry_msgs::WrenchStamped>("/wrench_world", 200);
	 
  std::string source_frameid = std::string("world");
  std::string target_frameid = std::string("tool0_inertial");


  // Wait for up to one second for the first transforms to become avaiable. 
  echoListener.tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));

  geometry_msgs::WrenchStamped ft_massage_out; 
  KDL::Wrench    F;
  tf::Vector3 vf, vt;
  while(nh.ok())
    {
      try
      {
        tf::StampedTransform echo_transform;
        echoListener.tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
		  
	F.force[0]= ft_massage.wrench.force.x;
	F.force[1]= ft_massage.wrench.force.y;
	F.force[2]= ft_massage.wrench.force.z;
	F.torque[0]=ft_massage.wrench.torque.x;
	F.torque[1]=ft_massage.wrench.torque.y;
	F.torque[2]=ft_massage.wrench.torque.z;
	
	tf::vectorKDLToTF(F.force, vf);
	tf::vectorKDLToTF(F.torque, vt);
	
	for (unsigned int j=0; j<3; j++){
	  F.force[j] = echo_transform.getBasis()[j].dot(vf);
	  F.torque[j] = echo_transform.getBasis()[j].dot(vt);
        }

        
             ft_massage_out.wrench.force.x = -F.force[1];
                ft_massage_out.wrench.force.y = F.force[0];
                ft_massage_out.wrench.force.z = F.force[2];
              ft_massage_out.wrench.torque.x = -F.torque[1];
              ft_massage_out.wrench.torque.y = F.torque[0];
              ft_massage_out.wrench.torque.z = F.torque[2];
        
       ft_pub.publish(ft_massage_out);
      
		  
      }
      catch(tf::TransformException& ex)
      {
        std::cout << "Failure at "<< ros::Time::now() << std::endl;
        std::cout << "Exception thrown:" << ex.what()<< std::endl;
        std::cout << "The current list of frames is:" <<std::endl;
        std::cout << echoListener.tf.allFramesAsString()<<std::endl;
        
      }
      rate.sleep();
    }

  return 0;
}
