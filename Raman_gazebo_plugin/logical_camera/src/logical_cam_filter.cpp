#include <gazebo_msgs/ModelStates.h>
#include <logical_camera/logical_camera_plugin.hh>

namespace log_cam_filter
{

LogicalCameraFilter::LogicalCameraFilter(ros::NodeHandle& nh) : nh_(nh)
{
    logCam_sub_ = nh_.subscribe("/gazebo/logical_camera", 1, &LogicalCameraFilter::logCamCallback, this);
}

LogicalCameraFilter::logCamCallback(const )

void LogicalCameraFilter::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
  // make sure that ros node for gazebo is already initialized.
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM(
        "A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // get the sensor locally
  this->sensorPtr = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(_sensor);

  // check of logical camera sensor exist
  if(!this->sensorPtr){
      gzerr << "Logical camera plugin requires a logical camera sensor.\n";
      return;
  }

  // bind to sensor update event 
  this->updateConnection = this->sensorPtr->ConnectUpdated(std::bind(&LogicalCameraPlugin::OnImage, this));

  // activate the sensor
  this->sensorPtr->SetActive(true);

  ROS_INFO("LogicalCameraPlugin loaded successfully.");

  //Creating ROS node
  //TODO set topic name via SDF + read the robot namespace 
  this->rosnode = new ros::NodeHandle("~");
  this->modelsPub = this->rosnode->advertise<gazebo_msgs::ModelStates>("logical_camera", 1, true);
}

void LogicalCameraPlugin::OnImage(){
  gazebo_msgs::ModelStates imageMsg;
  msgs::LogicalCameraImage logical_image;

  logical_image = this->sensorPtr->Image();

  //loop throught all the models with in the camera FOV to get their poses
  for(uint16_t i = 0; i < logical_image.model_size(); i++){
    imageMsg.name.emplace_back(logical_image.model(i).name());
    geometry_msgs::Pose_<std::allocator<void>> p;
    p.position.x = logical_image.model(i).pose().position().x();
    p.position.y = logical_image.model(i).pose().position().y();
    p.position.z = logical_image.model(i).pose().position().z();

    imageMsg.pose.emplace_back(p);
  }

  this->modelsPub.publish(imageMsg);  
}

}