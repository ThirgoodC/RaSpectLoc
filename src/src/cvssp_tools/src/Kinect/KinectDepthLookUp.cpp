/*
 * KinectDepthLookUp.cpp
 *
 *  Created on: 19 Jan 2018
 *      Author: footstool
 */

#include <cvssp_tools/Kinect/KinectDepthLookUp.h>

namespace cvssp_tools
{
KinectDepthLookUp::KinectDepthLookUp() :
    model_(), range_max_(0.0)
{
  // TODO Auto-generated constructor stub

}
KinectDepthLookUp::KinectDepthLookUp(image_geometry::PinholeCameraModel model) :
    model_(model), range_max_(0.0)
{
  // TODO Auto-generated constructor stub

}
KinectDepthLookUp::KinectDepthLookUp(image_geometry::PinholeCameraModel model, double range_max) :
    model_(model), range_max_(range_max)
{
  // TODO Auto-generated constructor stub

}

KinectDepthLookUp::~KinectDepthLookUp()
{
  // TODO Auto-generated destructor stub
}

void KinectDepthLookUp::lookup(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg)
{
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    //printf("Encoding 16UC1\n");
    this->convert<uint16_t>(depth_msg, cloud_msg);
  }
  else
  {
    this->convert<float>(depth_msg, cloud_msg);
  }
}

void KinectDepthLookUp::lookupWithFilter(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg)
{
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    //printf("Encoding 16UC1\n");
    this->convertWithFilter<uint16_t>(depth_msg, cloud_msg);
  }
  else
  {
    this->convertWithFilter<float>(depth_msg, cloud_msg);
  }
}


// Handles float or uint16 depths
template<typename T>
  void KinectDepthLookUp::convert(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg)
  {
    // Use correct principal point from calibration
    float center_x = model_.cx();
    float center_y = model_.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = depth_image_proc::DepthTraits<T>::toMeters(T(1));
    float constant_x = unit_scaling / model_.fx();
    float constant_y = unit_scaling / model_.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

    /*std::cout << "Params: [" << constant_x << ", " << constant_y << ", " << center_x << ", " << center_y << ", "
     << unit_scaling << "]" << std::endl;*/

    const T* depth_orig = reinterpret_cast<const T*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(T);

    for (int v = 0; v < (int)(cloud_msg->height * cloud_msg->width); ++v, ++iter_x, ++iter_y, ++iter_z)
    {

      //todo: interpolate?
      int d_x = (int)std::round(*iter_x);
      int d_y = (int)std::round(*iter_y);
      const T* depth_row = depth_orig + (d_y * row_step);
      T depth = depth_row[d_x];

      // Missing points denoted by NaNs
      if (!depth_image_proc::DepthTraits<T>::valid(depth))
      {
        if (range_max_ != 0.0)
        {
          depth = depth_image_proc::DepthTraits<T>::fromMeters(range_max_);
        }
        else
        {
          *iter_x = *iter_y = *iter_z = bad_point;
          //continue;
        }
      }

      // Fill in XYZ
      *iter_x = (*iter_x - center_x) * depth * constant_x;
      *iter_y = (*iter_y - center_y) * depth * constant_y;
      *iter_z = depth_image_proc::DepthTraits<T>::toMeters(depth);

      /*std::cout << "2D: [" << d_x << ", " << d_y << ", " << depth << "]" << std::endl;
       std::cout << "3D: [" << *iter_x << ", " << *iter_y << ", " << *iter_z << "]" << std::endl;*/
    }
  }

// Handles float or uint16 depths
template<typename T>
  void KinectDepthLookUp::convertWithFilter(const sensor_msgs::ImageConstPtr& depth_msg,
                                            sensor_msgs::PointCloud2::Ptr& cloud_msg)
  {
    // Use correct principal point from calibration
    float center_x = model_.cx();
    float center_y = model_.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = depth_image_proc::DepthTraits<T>::toMeters(T(1));
    float constant_x = unit_scaling / model_.fx();
    float constant_y = unit_scaling / model_.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

    /*std::cout << "Params: [" << constant_x << ", " << constant_y << ", " << center_x << ", " << center_y << ", "
     << unit_scaling << "]" << std::endl;*/

    const T* depth_orig = reinterpret_cast<const T*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(T);

    T fail_depth;
    float hip_x;
    float hip_y;
    float hip_z;

    float tmp_x;
    float tmp_y;
    float tmp_z;

    float dist;
    for (int v = 0; v < (int)(cloud_msg->height * cloud_msg->width); ++v, ++iter_x, ++iter_y, ++iter_z)
    {

      if ( (v % 18) == 0)
      {
        fail_depth = *(iter_z + 8);
        hip_x = (*(iter_x + 8) - center_x) * fail_depth * constant_x;
        hip_y = (*(iter_y + 8) - center_y) * fail_depth * constant_y;
        hip_z = depth_image_proc::DepthTraits<T>::toMeters(fail_depth);
      }

      //todo: interpolate?
      int d_x = (int)std::round(*iter_x);
      int d_y = (int)std::round(*iter_y);
      const T* depth_row = depth_orig + (d_y * row_step);
      T depth = depth_row[d_x];

      // Missing points denoted by NaNs
      if (!depth_image_proc::DepthTraits<T>::valid(depth))
      {
        depth = fail_depth;
        if (!depth_image_proc::DepthTraits<T>::valid(depth))
        {
          if (range_max_ != 0.0)
          {
            depth = depth_image_proc::DepthTraits<T>::fromMeters(range_max_);
          }
          else
          {
            *iter_x = *iter_y = *iter_z = bad_point;
            //continue;
          }
        }
      }

      *iter_x = (*iter_x - center_x) * depth * constant_x;
      *iter_y = (*iter_y - center_y) * depth * constant_y;
      *iter_z = depth_image_proc::DepthTraits<T>::toMeters(depth);

      // Fill in XYZ
      /*tmp_x = (*iter_x - center_x) * depth * constant_x;
      tmp_y = (*iter_y - center_y) * depth * constant_y;
      tmp_z = depth_image_proc::DepthTraits<T>::toMeters(depth);

      dist = std::sqrt(
          (hip_x - tmp_x) * (hip_x - tmp_x) + (hip_y - tmp_y) * (hip_y - tmp_y)
              + (hip_z - tmp_z) * (hip_z - tmp_z));

      if (dist > 1.5)
      {
        std::cout<<"Failed Depth Test: "<<dist<<std::endl;
        depth = fail_depth;
        if (!depth_image_proc::DepthTraits<T>::valid(depth))
        {
          if (range_max_ != 0.0)
          {
            depth = depth_image_proc::DepthTraits<T>::fromMeters(range_max_);
          }
          else
          {
            *iter_x = *iter_y = *iter_z = bad_point;
            //continue;
          }
        }
        std::cout<<"Recovered"<<std::endl;
        *iter_x = (*iter_x - center_x) * depth * constant_x;
        *iter_y = (*iter_y - center_y) * depth * constant_y;
        *iter_z = depth_image_proc::DepthTraits<T>::toMeters(depth);

      }else{

        *iter_x=tmp_x;
        *iter_y=tmp_y;
        *iter_z=tmp_z;

      }*/

      /*std::cout << "2D: [" << d_x << ", " << d_y << ", " << depth << "]" << std::endl;
       std::cout << "3D: [" << *iter_x << ", " << *iter_y << ", " << *iter_z << "]" << std::endl;*/
    }
  }
} /* namespace cvssp_tools */
