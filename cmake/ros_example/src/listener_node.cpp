#include <ros/ros.h>
#include "ros_example/listener.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");

  example::Listener listener;
  listener.run();

  return 0;
}
