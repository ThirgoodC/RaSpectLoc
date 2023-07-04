/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <bitset>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <boost/format.hpp>

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <tf/transform_listener.h>

#include <cvssp_tools/Kinect/KinectSubscriber.h>
#include <cvssp_tools/opencv_tools.h>

namespace cvssp_tools
{

boost::format c_format;
boost::format d_format;
bool save_all_image, save_image_service;
std::string color_encoding;
std::string depth_encoding;
bool request_start_end;

bool service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  save_image_service = true;
  return true;
}

/** Class to deal with which callback to call whether we have CameraInfo or not
 */
class KinectSaver : public KinectSubscriber
{
private:
  ros::ServiceServer save;
  ros::ServiceServer srv_start;
  ros::ServiceServer srv_end;

public:
  KinectSaver() :
      is_first_image_(true), has_camera_info_(false), count_(0)
  {
    ros::NodeHandle private_nh_("~");
    std::string format_string;
    private_nh_.param("path", path_, std::string("."));
    private_nh_.param("filename_format", format_string, std::string("%04i.%s"));
    private_nh_.param("color_encoding", color_encoding, std::string("bgr8"));
    private_nh_.param("depth_encoding", depth_encoding, std::string("16UC1"));
    private_nh_.param("save_all_image", save_all_image, true);
    private_nh_.param("request_start_end", request_start_end, false);
    private_nh_.param("save_pose", save_pose_, true);
    private_nh_.param("pose_filename", pose_filename_, std::string("pose.txt"));
    private_nh_.param("pad", pad_, false);
    private_nh_.param("repack_depth", repack_depth_, false);
    private_nh_.param("save_rpy", save_rpy_, false);
    private_nh_.param("live", live_, false);
    private_nh_.param("resize_image",resize_image_,false);

    private_nh_.param("map_frame", map_frame_, std::string("map"));
    private_nh_.param("cam_frame", cam_frame_, std::string("camera_link"));

    ROS_INFO("Transforming from: %s to: %s", map_frame_.c_str(), cam_frame_.c_str());

    c_format.parse((path_ + "/color" + format_string));
    d_format.parse((path_ + "/depth" + format_string));

    if (save_pose_)
    {
      if(!live_){
        pose_file_.open(path_ + "/" + pose_filename_);
      }
    }

    save = private_nh_.advertiseService("save", cvssp_tools::service);

    if (request_start_end && !save_all_image)
      ROS_WARN("'request_start_end' is true, so overwriting 'save_all_image' as true");

    srv_start = private_nh_.advertiseService("start", &KinectSaver::callbackStartSave, this);
    srv_end = private_nh_.advertiseService("end", &KinectSaver::callbackEndSave, this);

  }

  ~KinectSaver()
  {
    if (save_pose_)
    {
      if(!live_)
        pose_file_.close();
    }
  }

  //Overloaded from KinectSubscriber
  void postProcessImages()
  {
    callbackWithCameraInfo(this->getImgMsgRgb(), this->getImgMsgDepth(), this->getInfoMsg());
  }

  bool callbackStartSave(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    ROS_DEBUG("Received start saving request");
    start_time_ = ros::Time::now();
    end_time_ = ros::Time(0);

    res.success = true;
    return true;
  }

  bool callbackEndSave(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    ROS_DEBUG("Received end saving request");
    end_time_ = ros::Time::now();

    res.success = true;
    return true;
  }

  void callbackWithCameraInfo(const sensor_msgs::ImageConstPtr& color_image_msg,
                              const sensor_msgs::ImageConstPtr& depth_image_msg,
                              const sensor_msgs::CameraInfoConstPtr& info)
  {
    has_camera_info_ = true;

    if (!save_image_service && request_start_end)
    {
      if (start_time_ == ros::Time(0))
        return;
      else if (start_time_ > info->header.stamp)
        return;  // wait for message which comes after start_time
      else if ((end_time_ != ros::Time(0)) && (end_time_ < info->header.stamp))
        return;  // skip message which comes after end_time
    }

    // save the image
    std::string color_filename;
    if (!saveImage(color_image_msg, color_encoding, c_format, color_filename, false))
      return;

    std::string depth_filename;
    if (!saveImage(depth_image_msg, depth_encoding, d_format, depth_filename, true))
      return;

    //Save pose
    if (save_pose_ && !savePose())
      return;

    // save the CameraInfo
    if (info)
    {
      ROS_DEBUG("Saving Cam Info!");
      depth_filename = depth_filename.replace(depth_filename.rfind("."), depth_filename.length(), ".yaml");
      camera_calibration_parsers::writeCalibration(depth_filename, "camera", *info);
    }

    count_++;
  }
private:
  bool savePose()
  {
    if(live_)
      pose_file_.open(path_ + "/" + pose_filename_);

    tf::StampedTransform transform;
    try
    {

      listener_.lookupTransform(map_frame_, cam_frame_, ros::Time(0), transform);

      if (!save_rpy_)
      {

        //Save to file
        pose_file_ << transform.getOrigin().getX() << " " << transform.getOrigin().getY() << " "
            << transform.getOrigin().getZ() << " " << transform.getRotation().getX() << " "
            << transform.getRotation().getY() << " " << transform.getRotation().getZ() << " "
            << transform.getRotation().getW() << std::endl;
        ROS_DEBUG_STREAM(
            transform.getOrigin().getX() <<" "<<transform.getOrigin().getY() <<" "<<transform.getOrigin().getZ()<<" "<< transform.getRotation().getX() <<" "<<transform.getRotation().getY() <<" "<<transform.getRotation().getZ()<<" "<<transform.getRotation().getW());

      }
      else
      {
        double r, p, y;
        transform.getBasis().getEulerYPR(y, p, r);
        //Save to file
        pose_file_ << transform.getOrigin().getX() << std::endl << transform.getOrigin().getY() << std::endl << transform.getOrigin().getZ() << std::endl
                   << r << std::endl << p << std::endl << y << std::endl;
        ROS_DEBUG_STREAM(
            transform.getOrigin().getX() <<" "<<transform.getOrigin().getY() <<" "<<transform.getOrigin().getZ()<<" "<< r <<" "<<p <<" "<<y);

      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }

    if(live_)
      pose_file_.close();

    return true;
  }
  void padImage(cv::Mat& image)
  {
    if (pad_)
    {
      int pixelPad = (image.cols - image.rows) / 2;
      copyMakeBorder(image, image, pixelPad, pixelPad, 0, 0, cv::BORDER_CONSTANT, 0);
    }
  }
  void repackImage(cv::Mat& image)
  {
    if (repack_depth_)
    {
      cv::Mat repacked = cv::Mat::zeros(image.size(), CV_8UC3);

      ROS_DEBUG_STREAM("Repacking: "<<image.size() <<" - "<<type2str(image.type()));
      ROS_DEBUG_STREAM("Into     : "<<repacked.size() <<" - "<<type2str(repacked.type()));
      ROS_DEBUG_STREAM("Test: ("<<image.rows<<","<<image.cols<<")");

      uint32_t *debug = (uint32_t*)(image.data);
      uint8_t *input = (uint8_t*)(image.data);
      uint8_t *output = (uint8_t*)(repacked.data);
      for (int j = 0, l = 0; j < image.rows; j++, l++)
      {
        for (int i = 0, k = 0; i < 2 * image.cols; i += 2, k += 3)
        {
          output[repacked.step * l + k + 0] = input[image.step * j + i];  //& 0x00FF;
          output[repacked.step * l + k + 1] = input[image.step * j + i + 1];
          output[repacked.step * l + k + 2] = 0;

          //output[repacked.step * j + i + 1]    = input[image.step * j + i + 1];// & 0xFF00;
          //output[repacked.step * j + i + 2]    = 255;
          //output[repacked.step * j + i + 1]    = input[image.step * j + i ];
          //output[repacked.step * j + i + 2]    = input[image.step * j + i +1];
          /*ROS_DEBUG_STREAM(" 8-bit Channels: " << std::bitset<8> (output[repacked.step * j + i + 2]) << "," << std::bitset<8>(output[repacked.step * j + i + 1]) << "," << std::bitset<8>(output[repacked.step * j + i]));
           ROS_DEBUG_STREAM(" 8-bit Channels: " << std::bitset<8> (input[image.step * j + i ] & 0x00FF) << std::bitset<8>(input[image.step * j + i ] & 0xFF00));
           ROS_DEBUG_STREAM("16-bit Channel : " << std::bitset<16>(debug[image.step * j + i]));
           ROS_DEBUG_STREAM("16-bit Number : " << debug[image.step * j + i]);*/
        }
      }
      /*cv::imshow("Repacked",repacked);
       cv::imshow("Image", image);
       cv::waitKey(100);*/
      cv::swap(repacked, image);
    }
    else
    {
      ROS_DEBUG_STREAM("Not Repacking: "<<type2str(image.type()));
    }
  }

  void resizeImage(cv::Mat& image){
    if (resize_image_)
    {
      cv::Mat imSmall;
      cv::resize(image,imSmall,cv::Size(),0.5,0.5,cv::INTER_NEAREST);
      cv::swap(image,imSmall);
    }
  }
  bool saveImage(const sensor_msgs::ImageConstPtr& image_msg, const std::string& encoding, boost::format& g_format,
                 std::string &filename, bool isDepth)
  {
    cv::Mat image;
    try
    {
      image = cv_bridge::toCvShare(image_msg, encoding)->image;
    }
    catch (cv_bridge::Exception)
    {
      ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
      return false;
    }

    if (!image.empty())
    {
      if (isDepth)
      {
        repackImage(image);
      }

      //Pad Image
      padImage(image);
      resizeImage(image);

      try
      {
        filename = (g_format).str();
      }
      catch (...)
      {
        g_format.clear();
      }
      try
      {
        filename = (g_format % count_).str();
      }
      catch (...)
      {
        g_format.clear();
      }
      try
      {
        filename = (g_format % count_ % "jpg").str();
      }
      catch (...)
      {
        g_format.clear();
      }

      if (save_all_image || save_image_service)
      {
        cv::imwrite(filename, image);
        ROS_DEBUG("Saved image %s", filename.c_str());

        save_image_service = false;
      }
      else
      {
        return false;
      }
    }
    else
    {
      ROS_WARN("Couldn't save image, no data!");
      return false;
    }
    return true;
  }

private:
  bool is_first_image_;
  bool has_camera_info_;
  size_t count_;
  ros::Time start_time_;
  ros::Time end_time_;
  tf::TransformListener listener_;
  bool save_pose_;
  std::string pose_filename_;
  std::ofstream pose_file_;
  std::string path_;
  bool pad_;
  bool repack_depth_;
  bool save_rpy_;
  bool live_;
  bool resize_image_;
  std::string map_frame_;
  std::string cam_frame_;
};
}

using namespace cvssp_tools;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_saver", ros::init_options::AnonymousName);

  cvssp_tools::KinectSaver callbacks;

  ros::spin();
}
