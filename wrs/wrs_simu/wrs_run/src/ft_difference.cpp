#include <ros/ros.h>
#include <std_msgs/Float64.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



class DifferenceNode
{
protected:
  ros::NodeHandle nh;
  ros::Publisher ft_diffrence, ft_diffrence_conv;
  ros::Subscriber ft_mean_in_5, ft_mean_in_50, ft_mean_in_conv_5, ft_mean_in_conv_50;
  double in_50, in_50_conv;

public:
  DifferenceNode() 
  {
    ft_mean_in_5 = nh.subscribe("/wrench_mean_5", 1000, &DifferenceNode::callback_5, this);
    ft_mean_in_50 = nh.subscribe("/wrench_mean_50", 1000, &DifferenceNode::callback_50, this);
    ft_diffrence = nh.advertise<std_msgs::Float64>("/wrench_differnce", 50);   
    
    ft_mean_in_conv_5 = nh.subscribe("/wrench_mean_conv_5", 1000, &DifferenceNode::callback_conv_5, this);
    ft_mean_in_conv_50 = nh.subscribe("/wrench_mean_conv_50", 1000, &DifferenceNode::callback_conv_50, this);
    ft_diffrence_conv = nh.advertise<std_msgs::Float64>("/wrench_differnce_conv", 50);  
  }
  
  void callback_5(const geometry_msgs::WrenchStampedPtr& msg_in)
  {
    std_msgs::Float64 out_msg_;
    out_msg_.data = msg_in->wrench.force.z - in_50;
    ft_diffrence.publish(out_msg_);
  }
  
  void callback_50(const geometry_msgs::WrenchStampedPtr& msg_in)
  {
    in_50 = msg_in->wrench.force.z;
  }
   
    void callback_conv_5(const std_msgs::Float64Ptr& msg_in)
  {
    std_msgs::Float64 out_msg_;
    out_msg_.data = msg_in->data - in_50_conv;
    ft_diffrence_conv.publish(out_msg_);
  }
  
  void callback_conv_50(const std_msgs::Float64Ptr& msg_in)
  {
    in_50_conv = msg_in->data;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ft_diffrence_node");
  DifferenceNode DifferenceNode;
  ros::spin();
  return 0;
}