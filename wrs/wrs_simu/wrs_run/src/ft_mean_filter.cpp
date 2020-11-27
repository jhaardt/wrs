//https://answers.ros.org/question/321117/using-filters-from-the-filter-package-inwith-ros-nodes/?answer=323696#post-id-323696

#include <ros/ros.h>
#include <filters/mean.h>
#include <std_msgs/Float64.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class MyMeanFilter: public filters::MeanFilter <double>
{
public:
  bool configure(int num_obs){
    number_of_observations_ = num_obs;
    data_storage_.reset(new filters::RealtimeCircularBuffer<double>(number_of_observations_, temp_));
    return true;
  }
};

class SimpleFtFilterNode
{
protected:
  ros::NodeHandle nh;
  ros::Publisher ft_mean_out;
  ros::Publisher ft_mean_out_2;
  ros::Publisher ft_mean_conv_out;
  ros::Publisher ft_mean_conv_out_2;
  ros::Subscriber ft_mean_in;
  ros::Subscriber ft_mean_conv_in;
  int num_obs_;
  MyMeanFilter mean_filter_x;
    MyMeanFilter mean_filter_y;
      MyMeanFilter mean_filter_z;
        MyMeanFilter mean_filter_r;
	  MyMeanFilter mean_filter_q;
	    MyMeanFilter mean_filter_t;
	      
  MyMeanFilter mean_filter_2_x;
  MyMeanFilter mean_filter_2_y;
  MyMeanFilter mean_filter_2_z;
  MyMeanFilter mean_filter_2_r;
  MyMeanFilter mean_filter_2_q;
  MyMeanFilter mean_filter_2_t;
  
  MyMeanFilter mean_filter_conv;
  MyMeanFilter mean_filter_conv_2;

public:
  SimpleFtFilterNode() 
  {
    ros::NodeHandle pnh_("~"); 
    ft_mean_in = nh.subscribe("/wrench_world", 1000, &SimpleFtFilterNode::callback, this);
    ft_mean_conv_in = nh.subscribe("/wrench_converted", 1000, &SimpleFtFilterNode::callback_conv, this);
    ft_mean_out = nh.advertise<geometry_msgs::WrenchStamped>("/wrench_mean_5", 50);
    pnh_.param<int>("number_of_observations", num_obs_, 5);
    mean_filter_x.configure(num_obs_);
    mean_filter_y.configure(num_obs_);
    mean_filter_z.configure(num_obs_);
    mean_filter_r.configure(num_obs_);
    mean_filter_q.configure(num_obs_);
    mean_filter_t.configure(num_obs_);
    
    ft_mean_out_2 = nh.advertise<geometry_msgs::WrenchStamped>("/wrench_mean_50", 5);
    pnh_.param<int>("number_of_observations", num_obs_, 150);
    mean_filter_2_x.configure(num_obs_);
    mean_filter_2_y.configure(num_obs_);
    mean_filter_2_z.configure(num_obs_);
    mean_filter_2_r.configure(num_obs_);
    mean_filter_2_q.configure(num_obs_);
    mean_filter_2_t.configure(num_obs_);
    
    ft_mean_conv_out = nh.advertise<std_msgs::Float64>("/wrench_mean_conv_5", 50);
    pnh_.param<int>("number_of_observations", num_obs_, 5);
    mean_filter_conv.configure(num_obs_);
    
    ft_mean_conv_out_2 = nh.advertise<std_msgs::Float64>("/wrench_mean_conv_50", 50);
    pnh_.param<int>("number_of_observations", num_obs_, 150);
    mean_filter_conv_2.configure(num_obs_);
  }
  
void callback(const geometry_msgs::WrenchStampedPtr& msg_in)
  {
    geometry_msgs::WrenchStamped out_msg_;
    mean_filter_x.update (msg_in->wrench.force.x, out_msg_.wrench.force.x);
    mean_filter_y.update (msg_in->wrench.force.y, out_msg_.wrench.force.y);
    mean_filter_z.update (msg_in->wrench.force.z, out_msg_.wrench.force.z);
    mean_filter_r.update (msg_in->wrench.torque.x, out_msg_.wrench.torque.x);
    mean_filter_q.update (msg_in->wrench.torque.y, out_msg_.wrench.torque.y);
    mean_filter_t.update (msg_in->wrench.torque.z, out_msg_.wrench.torque.z);
    ft_mean_out.publish(out_msg_);
    
    mean_filter_2_x.update (msg_in->wrench.force.x, out_msg_.wrench.force.x);
    mean_filter_2_y.update (msg_in->wrench.force.y, out_msg_.wrench.force.y);
    mean_filter_2_z.update (msg_in->wrench.force.z, out_msg_.wrench.force.z);
    mean_filter_2_r.update (msg_in->wrench.torque.x, out_msg_.wrench.torque.x);
    mean_filter_2_q.update (msg_in->wrench.torque.y, out_msg_.wrench.torque.y);
    mean_filter_2_t.update (msg_in->wrench.torque.z, out_msg_.wrench.torque.z);
    ft_mean_out_2.publish(out_msg_);
  }
  
  void callback_conv(const std_msgs::Float64Ptr& msg_in)
  {
    std_msgs::Float64 out_msg_;
    mean_filter_conv.update (msg_in->data, out_msg_.data);
    ft_mean_conv_out.publish(out_msg_);
    
    mean_filter_conv_2.update (msg_in->data, out_msg_.data);
    ft_mean_conv_out_2.publish(out_msg_);
  }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_filter_node");
  SimpleFtFilterNode simple_ft_filter_node;
  ros::spin();
  return 0;
}