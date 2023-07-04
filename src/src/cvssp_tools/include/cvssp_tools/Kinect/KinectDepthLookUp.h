/*
 * KinectDepthLookUp.h
 *
 *  Created on: 19 Jan 2018
 *      Author: footstool
 */

#ifndef CVSSP_TOOLS_INCLUDE_CVSSP_TOOLS_KINECT_KINECTDEPTHLOOKUP_H_
#define CVSSP_TOOLS_INCLUDE_CVSSP_TOOLS_KINECT_KINECTDEPTHLOOKUP_H_

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>

namespace cvssp_tools
{

class KinectDepthLookUp
{
private:
  image_geometry::PinholeCameraModel model_;
  double range_max_;
public:
  KinectDepthLookUp();
  KinectDepthLookUp(image_geometry::PinholeCameraModel model);
  KinectDepthLookUp(image_geometry::PinholeCameraModel model, double range_max);
  virtual ~KinectDepthLookUp();

  void lookup(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg);
  void lookupWithFilter(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg);

  template<typename T>
    void convert(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg);

  template<typename T>
      void convertWithFilter(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg);

  const image_geometry::PinholeCameraModel& getModel() const
  {
    return model_;
  }

  void setModel(const image_geometry::PinholeCameraModel& model)
  {
    model_ = model;
  }

  double getRangeMax() const
  {
    return range_max_;
  }

  void setRangeMax(double rangeMax)
  {
    range_max_ = rangeMax;
  }
};

} /* namespace cvssp_tools */

#endif /* CVSSP_TOOLS_INCLUDE_CVSSP_TOOLS_KINECT_KINECTDEPTHLOOKUP_H_ */
