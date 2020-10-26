#ifndef TALKER_HPP
#define TALKER_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>


namespace example
{

class Talker
{
public:
  Talker();
  void run(double publish_frequency);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher string_pub_;
  std_msgs::String hello_msg_;
};

} // namespace example

#endif // TALKER_HPP
