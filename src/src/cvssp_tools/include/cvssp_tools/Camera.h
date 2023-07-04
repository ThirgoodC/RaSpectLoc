/*
 * Camera.h
 *
 *  Created on: 25 Apr 2017
 *      Author: footstool
 */

#ifndef CVSSP_LOCALISER_SRC_CAMERA_H_
#define CVSSP_LOCALISER_SRC_CAMERA_H_

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf_conversions/tf_eigen.h>
#include <image_geometry/pinhole_camera_model.h>
#include <camera_info_manager/camera_info_manager.h>

#include <cvssp_tools/localiser_tools.h>
#include <cvssp_tools/SemanticScan.h>
#include <cvssp_tools/Ray.h>

namespace cvssp_tools
{

class Camera
{
private:
  image_geometry::PinholeCameraModel camModel_;

  cv::Mat color_;
  cv::Mat depth_;
  cv::Mat label_;

  std::vector<cvssp_tools::Ray> rays_;

  tf::Pose pose_;

  bool rays_labeled_, rays_ranged_;

  void drawRay(const tf::Pose& origin, const Eigen::Vector3d& ray, const double& scale, visualization_msgs::Marker& m);

public:
  Camera();
  Camera(const sensor_msgs::CameraInfo& camModel);
  /*Camera(const sensor_msgs::CameraInfo& camModel, const std::vector<cv::Point2d>& pts2d);
  Camera(const cv::Mat& image, const cv::Mat& depth, const cv::Mat& label, const tf::Pose& pose,
         const sensor_msgs::CameraInfo& camModel, const std::vector<cv::Point2d>& pts2d);*/
  virtual ~Camera();

  void generateRays(const std::vector<cv::Point2d>& pts2d);
  void updateRays();
  void getRaysMsg(visualization_msgs::MarkerArray& markerArray, std_msgs::Header header);
  void getRaysMsg(visualization_msgs::MarkerArray& markerArray, const double& scale);
  void getSemanticScanMsg(cvssp_tools::SemanticScan& msg);

  void populateRayLabel();
  void populateRayRange();

  bool isInit()
  {
    return camModel_.initialized();
  }

  cv::Mat& getColor()
  {
    return color_;
  }

  void setColor(const cv::Mat& image)
  {
    color_ = image;
  }

  cv::Mat& getLabel()
  {
    return label_;
  }

  void setLabel(const cv::Mat& label)
  {
    label_ = label;
  }

  const tf::Pose& getPose() const
  {
    return pose_;
  }

  void setPose(const tf::Pose& pose)
  {
    pose_ = pose;
  }

  const Ray& getRays(const int& i) const
  {
    return rays_[i];
  }

  const std::vector<Ray>& getRays() const
  {
    return rays_;
  }

  void setRays(const std::vector<Ray>& rays)
  {
    rays_ = rays;
  }

  void setRays(const std::vector<cv::Point2d>& pixels)
  {
    assert(false);
  }

  const image_geometry::PinholeCameraModel& getCamModel() const
  {
    return camModel_;
  }

  void setCamInfo(const sensor_msgs::CameraInfo& camInfo)
  {
    camModel_.fromCameraInfo(camInfo);
  }

  void setCamInfo(const image_geometry::PinholeCameraModel& camModel)
  {
    this->camModel_ = camModel;
  }

  const cv::Mat& getK() const
  {
    return cv::Mat(camModel_.fullIntrinsicMatrix());
  }
  const cv::Mat& getDepthConst() const
  {
    return depth_;
  }
  cv::Mat& getDepth()
  {
    return depth_;
  }

  void setDepth(const cv::Mat& depth)
  {
    depth_ = depth;
  }

  bool isLabeled() const
  {
    return rays_labeled_;
  }

  void setLabeled(bool labeled)
  {
    this->rays_labeled_ = labeled;
  }

  bool isRanged() const
  {
    return rays_ranged_;
  }

  void setRanged(bool ranged)
  {
    this->rays_ranged_ = ranged;
  }
};

} /* namespace cvssp_tools */

#endif /* CVSSP_LOCALISER_SRC_CAMERA_H_ */
