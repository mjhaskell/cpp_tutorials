#ifndef LISTENER_HPP
#define LISTENER_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>


namespace example
{

class Listener
{
public:
  Listener();
  void run();

private:
  void chatterCallback(const std_msgs::StringConstPtr &msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber string_sub_;
  ros::Publisher count_pub_;
  std_msgs::UInt64 count_msg_;
  uint count_;
};

} // namespace example

#endif // LISTENER_HPP
