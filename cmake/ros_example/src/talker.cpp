#include "ros_example/talker.hpp"


namespace example
{

Talker::Talker() :
  nh_private_{"~"}
{
  string_pub_ = nh_.advertise<std_msgs::String>("chatter", 1);
  hello_msg_.data = "hello";
}

void Talker::run(double publish_frequency)
{
  ros::Rate rate{publish_frequency};
  while (ros::ok())
  {
    ROS_WARN_ONCE("[talker] started talking");
    string_pub_.publish(hello_msg_);
    // ros::spinOnce(); // need this if you have subscribers
    rate.sleep();
  }
}

} // namespace example
