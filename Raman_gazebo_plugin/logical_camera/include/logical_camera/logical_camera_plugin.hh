#pragma once

#include <sdf/sdf.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/logical_camera_image.pb.h>

#include <gazebo/rendering/rendering.hh>

#include <ros/ros.h>

namespace gazebo
{

class LogicalCameraPlugin : public SensorPlugin {
public:
  LogicalCameraPlugin();

  virtual ~LogicalCameraPlugin();

  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

private:
  void OnImage();

  ros::NodeHandle* rosnode;
  ros::Publisher   modelsPub;

  event::ConnectionPtr updateConnection;
  sensors::LogicalCameraSensorPtr sensorPtr;

};


}