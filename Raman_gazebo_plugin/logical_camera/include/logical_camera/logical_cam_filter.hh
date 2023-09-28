#pragma once

#include <sdf/sdf.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/logical_camera_image.pb.h>

#include <gazebo/rendering/rendering.hh>

#include <ros/ros.h>


class LogicalCameraPlugin{
public:


private:
  void OnImage();

};