#include <ros/ros.h>
#include "ros_example/talker.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh{"~"};

  example::Talker talker;
  double publish_freq;
  nh.param<double>("publish_frequency", publish_freq, 1.0);
  talker.run(publish_freq);

  return 0;
}
