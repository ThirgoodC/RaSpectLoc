/*
 * KinectReader.cpp
 *
 *  Created on: 5 Mar 2018
 *      Author: om0007
 */

#include "cvssp_tools/Kinect/KinectReader.h"
#include "cvssp_tools/opencv_tools.h"

#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <camera_calibration_parsers/parse.h>

namespace cvssp_tools
{

KinectReader::KinectReader()
{
  // TODO Auto-generated constructor stub

}

KinectReader::~KinectReader()
{
  // TODO Auto-generated destructor stub
}

} /* namespace cvssp_tools */
cv::Mat convertToMono16NoOpenCV(cv::Mat& image)
{
  cv::Mat unpacked = cv::Mat::zeros(image.size(), CV_16UC1);

  ROS_INFO_STREAM("Unpacking: "<<image.size() <<" - "<<cvssp_tools::type2str(image.type()));
  ROS_INFO_STREAM("Into     : "<<unpacked.size() <<" - "<<cvssp_tools::type2str(unpacked.type()));
  ROS_INFO_STREAM("Test: ("<<image.rows<<","<<image.cols<<")");

  uint8_t *input = (uint8_t*)(image.data);
  uint16_t *output = (uint16_t*)(unpacked.data);
  for (int j = 0, l = 0; j < image.rows; j++, l++)
  {
    for (int i = 0, k = 0; i < image.cols; i +=3, k ++)
    {
      uint8_t b  = input[image.step * j + i]; //& 0x00FF;
      uint8_t g  = input[image.step * j + i + 1];
      uint8_t r  = input[image.step * j + i + 2];

      unpacked.at<unsigned short>(j, k) = b+(g*256);
    }
  }
  return unpacked;
}
cv::Mat convertToMono16(cv::Mat& image)
{
  cv::Mat unpacked = cv::Mat::zeros(image.size(), CV_16UC1);

  ROS_INFO_STREAM("Unpacking: "<<image.size() <<" - "<<cvssp_tools::type2str(image.type()));
  ROS_INFO_STREAM("Into     : "<<unpacked.size() <<" - "<<cvssp_tools::type2str(unpacked.type()));
  ROS_INFO_STREAM("Test: ("<<image.rows<<","<<image.cols<<")");

  uint8_t *input = (uint8_t*)(image.data);
  uint8_t *output = (uint8_t*)(unpacked.data);
  for (int j = 0, l = 0; j < image.rows; j++, l++)
  {
    for (int i = 0, k = 0; i < 2 * image.cols; i += 2, k += 3)
    {
      output[unpacked.step * j + i + 0] = input[image.step * l + k]; //& 0x00FF;
      output[unpacked.step * j + i + 1] = input[image.step * l + k + 1];
    }
  }
  return unpacked;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_reader", ros::init_options::AnonymousName);

  ros::NodeHandle nh;

  cvssp_tools::KinectReader callbacks;

  cv::Mat cv_color;
  cv::Mat cv_depth;
  cv::Mat cv_depth_fixed;
  cv::Mat cv_depth_fixed_math;

  sensor_msgs::CameraInfo cam_info;
  std::string cam_name("camera");
  camera_calibration_parsers::readCalibration("/home/chris/Desktop/test/depth0000.yaml", cam_name, cam_info);
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(cam_info);

  ros::Publisher color_pub = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_color", 1);
  ros::Publisher depth_pub = nh.advertise<sensor_msgs::Image>("/camera/depth_registered/image_raw", 1);
  ros::Publisher depth_fixed_pub = nh.advertise<sensor_msgs::Image>("/camera/depth_registered/image_fixed", 1);
  ros::Publisher depth_fixed_math_pub = nh.advertise<sensor_msgs::Image>("/camera/depth_registered/image_fixed_math", 1);
  ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/depth_registered/camera_info", 1);

  ros::Time time = ros::Time::now();

  sensor_msgs::ImagePtr msg_color;
  sensor_msgs::ImagePtr msg_depth;
  sensor_msgs::ImagePtr msg_depth_fixed;
  sensor_msgs::ImagePtr msg_depth_fixed_math;
  char k;

  std::string path = "/home/chris/Desktop/test/";
  int count = 0;
  do
  {
    std::stringstream color_path;
    color_path << std::setfill('0') << std::setw(4);
    color_path << path << "color"<< std::setfill('0') << std::setw(4)<<count<<".png";
    ROS_INFO_STREAM("Opening: "<<color_path.str());
    cv_color = cv::imread(color_path.str());

    std::stringstream depth_path;
    depth_path << std::setfill('0') << std::setw(4);
    depth_path << path << "depth"<< std::setfill('0') << std::setw(4)<<count<<".png";
    ROS_INFO_STREAM("Opening: "<<depth_path.str());
    cv_depth = cv::imread(depth_path.str());


    cv_depth_fixed = convertToMono16(cv_depth);
    cv_depth_fixed_math = convertToMono16NoOpenCV(cv_depth);

    cv::imshow("Color", cv_color);
    cv::imshow("Depth", cv_depth);
    cv::imshow("DepthFixed", cv_depth_fixed);
    cv::imshow("DepthFixed", cv_depth_fixed_math);


    msg_color       = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, cv_color).toImageMsg();
    msg_depth       = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, cv_depth).toImageMsg();
    msg_depth_fixed = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO16, cv_depth_fixed).toImageMsg();
    msg_depth_fixed_math = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO16, cv_depth_fixed).toImageMsg();

    std_msgs::Header header;
    header.frame_id="map";
    header.seq=count;
    header.stamp=ros::Time::now();

    msg_color->header = header;
    msg_depth->header = header;
    msg_depth_fixed->header = header;
    msg_depth_fixed_math->header = header;

    color_pub.publish(msg_color);
    depth_pub.publish(msg_depth);
    depth_fixed_pub.publish(msg_depth_fixed);
    depth_fixed_math_pub.publish(msg_depth_fixed_math);
    cam_info_pub.publish(cam_info);

    ros::spinOnce();

    if(count == 100){
      count=0;
    }
    count++;

  } while (cv::waitKey(100) != 'q');

  //ros::spin();
}
