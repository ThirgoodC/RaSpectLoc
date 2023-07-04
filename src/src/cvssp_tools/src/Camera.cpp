/*
 * Camera.cpp
 *
 *  Created on: 25 Apr 2017
 *      Author: footstool
 */

#include "cvssp_tools/Camera.h"

namespace cvssp_tools
{
Camera::Camera() :
    rays_labeled_(false), rays_ranged_(false)
{
}
Camera::Camera(const sensor_msgs::CameraInfo& camModel) :
    rays_labeled_(false), rays_ranged_(false)
{
  camModel_.fromCameraInfo(camModel);
}

/*Camera::Camera(const sensor_msgs::CameraInfo& camModel, const std::vector<cv::Point2d>& pts2d) :
    labeled(false), ranged(false)
{
  camModel_.fromCameraInfo(camModel);
  generateRays(pts2d);
}

Camera::Camera(const cv::Mat& image, const cv::Mat& depth, const cv::Mat& labels, const tf::Pose& pose,
               const sensor_msgs::CameraInfo& camModel, const std::vector<cv::Point2d>& pts2d) :
    image_(image), depth_(depth), labels_(labels), pose_(pose), labeled(false), ranged(false)
{
  camModel_.fromCameraInfo(camModel);
  generateRays(pts2d);
  populateLabels();
  populateRange();
}*/

Camera::~Camera()
{
  // TODO Auto-generated destructor stub
}

void Camera::generateRays(const std::vector<cv::Point2d>& pts2d)
{
  Eigen::Matrix3d K;
  cv::cv2eigen(cv::Mat(camModel_.fullIntrinsicMatrix()), K);
  ROS_DEBUG_STREAM("\nK: " << K);

  Eigen::Matrix3d K_inv = (K).inverse();

  //Centre Ray (for angle estimation)
  Eigen::Vector3d Rc = K_inv
      * Eigen::Vector3d(camModel_.cx(), camModel_.cy(), 1.0);

  rays_.resize(pts2d.size());
  for (uint i = 0; i < pts2d.size(); i++)
  {
    //Set Pixel
    rays_[i].pix() = Eigen::Vector2d(pts2d[i].x, pts2d[i].y);

    //Set Direction
    rays_[i].dir() = K_inv * rays_[i].pix().homogeneous();

    //Set Label
    rays_[i].label() = 0;

    //Set Label
    rays_[i].range() = 0.0;

    //Set angle
    rays_[i].angle() = cvssp_tools::angle_between_rays(Rc, rays_[i].dir());

    ROS_DEBUG("Ray %3i: [%7.3f, %7.3f] -> [%7.3f, %7.3f, %7.3f] -> %7.3f -> %7.3f -> %u", i, rays_[i].pix().x(),
              rays_[i].pix().y(),
              rays_[i].dir().x(), rays_[i].dir().y(), rays_[i].dir().z(), rays_[i].range(), rays_[i].angle(),
              rays_[i].label());
  }

  //std::sort(rays_.begin(), rays_.end());
}

void Camera::updateRays()
{
  populateRayLabel();
  populateRayRange();
}

void Camera::drawRay(const tf::Pose& origin, const Eigen::Vector3d& ray, const double& scale,
                     visualization_msgs::Marker& m)
{
  m.header.frame_id = "camera_rgb_frame";
  m.header.stamp = ros::Time();
  m.ns = "rays";
  m.id = 0;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::MODIFY;
  m.points.resize(2);
  m.points[0].x = origin.getOrigin().x();
  m.points[0].y = origin.getOrigin().y();
  m.points[0].z = origin.getOrigin().z();
  m.points[1].x = origin.getOrigin().x() + ray.x() * scale;
  m.points[1].y = origin.getOrigin().y() + ray.y() * scale;
  m.points[1].z = origin.getOrigin().z() + ray.z() * scale;
  m.scale.x = 0.03;
  m.scale.y = 0.04;
  m.scale.z = 0.0;
  m.color.a = 1.0; // Don't forget to set the alpha!
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
}
void Camera::getRaysMsg(visualization_msgs::MarkerArray& markerArray, std_msgs::Header header)
{

  tf::Pose origin(tf::createIdentityQuaternion(), tf::Vector3(0.0, 0.0, 0.0));
  
  int downsample=1;
  markerArray.markers.clear();
  markerArray.markers.resize(rays_.size()/downsample);

  //Draw Centre Ray first (helps visualisation)
  Eigen::Matrix3d K;
  cv::cv2eigen(cv::Mat(camModel_.fullIntrinsicMatrix()), K);

//  Eigen::Vector3d r = K.inverse() * Eigen::Vector3d(camModel_.cx(), camModel_.cy(), 1.0);
//  drawRay(origin, r, 2.0, markerArray.markers[0]);
//  markerArray.markers[rays_.size()].header = header;
//  markerArray.markers[rays_.size()].color.g = 1.0;
//  markerArray.markers[rays_.size()].color.b = 1.0;
#pragma omp for
  for (uint i = 0; i < markerArray.markers.size(); i++)
  {
    drawRay(origin, rays_[i*downsample].dir(), rays_[i*downsample].range(), markerArray.markers[i]);
    markerArray.markers[i].color.r = (double)i / (double)rays_.size();
    markerArray.markers[i].header = header;
    markerArray.markers[i].id = i*downsample;
  }
}
void Camera::getRaysMsg(visualization_msgs::MarkerArray& markerArray, const double& scale)
{
  Eigen::Matrix3d R;
  tf::matrixTFToEigen(pose_.getBasis(), R);
  ROS_DEBUG_STREAM("R: \n"<<R);

  markerArray.markers.clear();
  markerArray.markers.resize(rays_.size() + 1);

  //Draw Centre Ray first (helps visualisation)
  Eigen::Matrix3d K;
  cv::cv2eigen(cv::Mat(camModel_.fullIntrinsicMatrix()), K);
  Eigen::Vector3d r = K.inverse()
      * Eigen::Vector3d(camModel_.cx(), camModel_.cy(), 1.0);
  drawRay(pose_, R * r, 2.0 * scale, markerArray.markers[0]);
  markerArray.markers[0].color.g = 1.0;
  markerArray.markers[0].color.b = 1.0;

  for (uint i = 1; i < rays_.size(); i++)
  {
    r = rays_[i].dir();
    r[1] = 0.0;
    drawRay(pose_, R * r, scale, markerArray.markers[i]);
    markerArray.markers[i].color.r = (double)i / (double)rays_.size();
    markerArray.markers[i].header.frame_id = "/camera_depth_frame";
    markerArray.markers[i].id = i;
  }
}
void Camera::populateRayLabel()
{
  assert(!label_.empty());

  for (uint i = 0; i < rays_.size(); i++)
  {
    rays_[i].label() = label_.at<uint8_t>(rays_[i].pix().y(), rays_[i].pix().x());
  }
  rays_labeled_ = true;
}
void Camera::populateRayRange()
{

  assert(!depth_.empty());

  for (uint i = 0; i < rays_.size(); i++)
  {
    rays_[i].range() = depth_.at<uint16_t>(rays_[i].pix().y(), rays_[i].pix().x());
    rays_[i].range() /= 1000.0; //To Meters
  }
  rays_ranged_ = true;
}
void Camera::getSemanticScanMsg(cvssp_tools::SemanticScan& msg)
{
  if (!rays_labeled_)
    populateRayLabel();
  if (!rays_ranged_)
    populateRayRange();

  msg.header.frame_id = "cam";
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();

  msg.range_min = 0.0;
  msg.range_max = 5.0;

  msg.ranges.resize(rays_.size());
  msg.labels.resize(rays_.size());
  msg.angles.resize(rays_.size());

  for (uint i = 0; i < rays_.size(); i++)
  {
    msg.ranges[i] = rays_[i].range();
    msg.labels[i] = rays_[i].label();
    msg.angles[i] = rays_[i].angle();

    ROS_DEBUG("Ray %3i: [%7.3f, %7.3f] -> [%7.3f, %7.3f, %7.3f] -> %7.3f -> %7.3f -> %u", i, rays_[i].pix().x(),
              rays_[i].pix().y(), rays_[i].dir().x(), rays_[i].dir().y(), rays_[i].dir().z(), rays_[i].range(),
              rays_[i].angle(), rays_[i].label());
  }
}

} /* namespace cvssp_tools */
