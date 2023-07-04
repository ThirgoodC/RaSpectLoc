/*
 * localiser_tools.h
 *
 *  Created on: 28 Apr 2017
 *      Author: footstool
 */

#ifndef CVSSP_LOCALISER_INCLUDE_CVSSP_LOCALISER_LOCALISER_TOOLS_H_
#define CVSSP_LOCALISER_INCLUDE_CVSSP_LOCALISER_LOCALISER_TOOLS_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>

#include <image_geometry/pinhole_camera_model.h>
#include <camera_info_manager/camera_info_manager.h>

#define NDEBUG

namespace cvssp_tools
{
inline double magnitude_of_ray(const cv::Point3d& ray)
{
  return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
}
inline double angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2)
{
  double dot_product = ray1.x * ray2.x + ray1.y * ray2.y + ray1.z * ray2.z;
  double magnitude1 = magnitude_of_ray(ray1);
  double magnitude2 = magnitude_of_ray(ray2);
  return acos(dot_product / (magnitude1 * magnitude2));
}
inline double magnitude_of_ray(const Eigen::Vector3d& ray)
{
  return sqrt(pow(ray.x(), 2.0) + pow(ray.y(), 2.0) + pow(ray.z(), 2.0));
}
inline double angle_between_rays(const Eigen::Vector3d& ray1, const Eigen::Vector3d& ray2)
{
  double dot_product = ray1.x() * ray2.x() + ray1.y() * ray2.y() + ray1.z() * ray2.z();
  double magnitude1 = magnitude_of_ray(ray1);
  double magnitude2 = magnitude_of_ray(ray2);
  return acos(dot_product / (magnitude1 * magnitude2));
}
inline void generateScanLine(const int& width, const int& height, const image_geometry::PinholeCameraModel& camModel,
                             std::vector<cv::Point2d>& pixels)
{
  pixels.resize(width);
  for (uint i = 0; i < width; i++)
  {
    pixels[i] = camModel.rectifyPoint(cv::Point2d(i, camModel.cy()));
  }

}

inline void generate2DSamples(const int& width, const int& height, const int& numSamples,
                              std::vector<cv::Point2d>& pixels)
{
  int nSamples = numSamples;
  if (width * height < numSamples)
  {
    ROS_ERROR("Requested samples exceed pixels, returning all!");
    nSamples = width * height;
  }

  //Estimate Step Size for uniform grid
  int numWidthSamples = std::floor(std::sqrt(((double)nSamples * (double)width) / ((double)height)));
  int numHeightSamples = std::floor(std::sqrt(((double)nSamples * (double)height) / ((double)width)));
  int actualNumSamples = numWidthSamples * numHeightSamples;

  assert(actualNumSamples <= nSamples);

  int widthStep = width / numWidthSamples;
  int heightStep = height / numHeightSamples;

  pixels.resize(actualNumSamples);
  for (int i = 0; i < numWidthSamples; i++)
  {
    for (int j = 0; j < numHeightSamples; j++)
    {
      //ROS_INFO("Pix [%i, %i, %i]:, [%i, %i]",i, j, (i*numHeightSamples)+j,  (i * widthStep),(j * heightStep));
      pixels[(i * numHeightSamples) + j].x = (i * widthStep) + j % widthStep;
      pixels[(i * numHeightSamples) + j].y = (j * heightStep) + heightStep / 2;
    }
  }

  if (pixels.size() != nSamples)
  {
    ROS_DEBUG("Requested %u, but only %ui provided.", nSamples, pixels.size());
  }

#ifndef NDEBUG
  ROS_INFO("Image: [%i, %i]", width, height);
  ROS_INFO("Requested: %i", numSamples);
  ROS_INFO("Got: %i", pixels.size());
  ROS_INFO("Samples: [%i, %i]", numWidthSamples, numHeightSamples);
  cv::Mat displaySamples=cv::Mat::zeros(cv::Size(width, height), CV_8UC1);
  for (int i = 0; i < actualNumSamples; i++)
  {
    displaySamples.at<uint8_t>(pixels[i].y, pixels[i].x) = 255;
  }
  cv::imshow("Samples", displaySamples);
  int k = cv::waitKey(100);
  if (k == 27)
  exit(0);
#endif
}

inline std::vector<cv::Point2d> generate2DSamples(const int& width, const int& height, const int& numSamples)
{
  int nSamples = numSamples;
  if (width * height < numSamples)
  {
    ROS_ERROR("Requested samples exceed pixels, returning all!");
    nSamples = width * height;
  }

  std::vector<cv::Point2d> pixels;

  //Estimate Step Size for uniform grid
  int numWidthSamples = std::floor(std::sqrt(((double)nSamples * (double)width) / ((double)height)));
  int numHeightSamples = std::floor(std::sqrt(((double)nSamples * (double)height) / ((double)width)));
  int actualNumSamples = numWidthSamples * numHeightSamples;

  assert(actualNumSamples <= nSamples);

  int widthStep = width / numWidthSamples;
  int heightStep = height / numHeightSamples;

  pixels.resize(actualNumSamples);
  for (int i = 0; i < numWidthSamples; i++)
  {
    for (int j = 0; j < numHeightSamples; j++)
    {
      //ROS_INFO("Pix [%i, %i, %i]:, [%i, %i]",i, j, (i*numHeightSamples)+j,  (i * widthStep),(j * heightStep));
      pixels[(i * numHeightSamples) + j].x = (i * widthStep) + j % widthStep;
      pixels[(i * numHeightSamples) + j].y = (j * heightStep) + heightStep / 2;
    }
  }

  if (pixels.size() != nSamples)
  {
    ROS_DEBUG("Requested %u, but only %ui provided.", nSamples, pixels.size());
  }

#ifndef NDEBUG
  ROS_INFO("Image: [%i, %i]", width, height);
  ROS_INFO("Requested: %i", numSamples);
  ROS_INFO("Got: %i", pixels.size());
  ROS_INFO("Samples: [%i, %i]", numWidthSamples, numHeightSamples);
  cv::Mat displaySamples=cv::Mat::zeros(cv::Size(width, height), CV_8UC1);
  for (int i = 0; i < actualNumSamples; i++)
  {
    displaySamples.at<uint8_t>(pixels[i].y, pixels[i].x) = 255;
  }
  cv::imshow("Samples", displaySamples);
  int k = cv::waitKey(100);
  if (k == 27)
  exit(0);
#endif
  return pixels;
}

} /*namespace cvssp_tools*/

#endif /* CVSSP_LOCALISER_INCLUDE_CVSSP_LOCALISER_LOCALISER_TOOLS_H_ */
