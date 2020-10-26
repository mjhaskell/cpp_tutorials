#include "ros_example/listener.hpp"


namespace example
{

Listener::Listener() :
  nh_private_{"~"},
  count_{0}
{
  string_sub_ = nh_.subscribe("chatter", 1, &Listener::chatterCallback, this);
  count_pub_ = nh_.advertise<std_msgs::UInt64>("count", 1);
}

void Listener::run()
{
  ros::spin();
}

void Listener::chatterCallback(const std_msgs::StringConstPtr &msg)
{
  ROS_WARN_ONCE("I heard: [%s]", msg->data.c_str());
  count_msg_.data = ++count_;
  count_pub_.publish(count_msg_);
}

} // namespace example
