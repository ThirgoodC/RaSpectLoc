#include "ros/ros.h"
#include "std_msgs/String.h"

#include <gazebo_msgs/ModelStates.h>
#include <logical_camera/logical_camera_plugin.hh>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void logicalCameraCallback(const std_msgs::String::ConstPtr& msg)
{


  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");


  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("gazebo/logical_camera", 1000, logicalCameraCallback);

  ros::spin();

  return 0;
}