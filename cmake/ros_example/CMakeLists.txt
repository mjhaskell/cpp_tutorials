cmake_minimum_required(VERSION 3.0.2)
project(ros_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
  )


add_executable(talker
  src/talker_node.cpp
  src/talker.cpp
  include/ros_example/talker.hpp
  )
target_include_directories(talker PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener
  src/listener_node.cpp
  src/listener.cpp
  include/ros_example/listener.hpp
  )
target_include_directories(listener PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(listener ${catkin_LIBRARIES})
