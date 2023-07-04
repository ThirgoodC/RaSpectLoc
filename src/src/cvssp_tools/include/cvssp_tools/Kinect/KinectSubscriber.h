/*
 * KinectSubscriber.h
 *
 *  Created on: 11 May 2017
 *      Author: footstool
 */

#ifndef CVSSP_TOOLS_SRC_KINECTSUBSCRIBER_H_
#define CVSSP_TOOLS_SRC_KINECTSUBSCRIBER_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>        // OpenCV
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <image_transport/image_transport.h>
#define USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER 1
#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
#  include <image_transport/subscriber_filter.h>
#else
#  include <sensor_msgs/Image.h>
#  include <message_filters/subscriber.h>
#endif

#include <cvssp_tools/Camera.h>
#include <cvssp_tools/SemanticScan.h>

namespace cvssp_tools
{

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
  typedef image_transport::SubscriberFilter ImageSubscriber;
#else
  typedef message_filters::Subscriber< sensor_msgs::Image > ImageSubscriber;
#endif

  class MyImageSubscriber : public image_transport::SubscriberFilter
{
public:
  void newMessage(const boost::shared_ptr<sensor_msgs::Image const> &msg)
  {
    this->signalMessage(msg);
  }
};

class KinectSubscriber : public cvssp_tools::Camera
{
private:

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  ros::Subscriber cam_info_sub_;

  MyImageSubscriber subscriber_depth_;
  MyImageSubscriber subscriber_rgb_;
  message_filters::Synchronizer<MySyncPolicy> sync_;

  double time_accum;

  sensor_msgs::ImageConstPtr img_msg_rgb_;
  sensor_msgs::ImageConstPtr img_msg_depth_;
  sensor_msgs::CameraInfoConstPtr info_msg_;
  cv_bridge::CvImagePtr img_ptr_rgb;
  cv_bridge::CvImagePtr img_ptr_depth;
  std_msgs::Header image_header_;
  std_msgs::Header depth_header_;

public:
  KinectSubscriber();
  virtual ~KinectSubscriber();

  // Callbacks
  void camInfoCb(const sensor_msgs::CameraInfoConstPtr& msg);
  virtual void postProcessInfo()
  {
    //Generate 2D stuff
    std::vector<cv::Point2d> pts2d;
    cvssp_tools::generateScanLine(this->getCamModel().fullResolution().width,
                                  this->getCamModel().fullResolution().height, this->getCamModel(), pts2d);
    this->generateRays(pts2d);
  }
  void imageSyncCb(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth);
  virtual void postProcessImages()
  {
  }

  const std::string& getDepthTopic()
  {
    return subscriber_depth_.getTopic();
  }

  const std::string& getRGBTopic()
  {
    return subscriber_rgb_.getTopic();
  }

  const std_msgs::Header& getDepthHeader() const
  {
    return depth_header_;
  }

  void setDepthHeader(const std_msgs::Header& depthHeader)
  {
    depth_header_ = depthHeader;
  }

  const std_msgs::Header& getImageHeader() const
  {
    return image_header_;
  }

  void setImageHeader(const std_msgs::Header& imageHeader)
  {
    image_header_ = imageHeader;
  }

  const cv_bridge::CvImagePtr& getImgPtrDepth() const
  {
    return img_ptr_depth;
  }

  void setImgPtrDepth(const cv_bridge::CvImagePtr& imgPtrDepth)
  {
    img_ptr_depth = imgPtrDepth;
  }

  const cv_bridge::CvImagePtr& getImgPtrRgb() const
  {
    return img_ptr_rgb;
  }

  void setImgPtrRgb(const cv_bridge::CvImagePtr& imgPtrRgb)
  {
    img_ptr_rgb = imgPtrRgb;
  }

  const sensor_msgs::ImageConstPtr& getImgMsgDepth() const
  {
    return img_msg_depth_;
  }

  void setImgMsgDepth(const sensor_msgs::ImageConstPtr& imgMsgDepth)
  {
    img_msg_depth_ = imgMsgDepth;
  }

  const sensor_msgs::ImageConstPtr& getImgMsgRgb() const
  {
    return img_msg_rgb_;
  }

  void setImgMsgRgb(const sensor_msgs::ImageConstPtr& imgMsgRgb)
  {
    img_msg_rgb_ = imgMsgRgb;
  }

  const sensor_msgs::CameraInfoConstPtr& getInfoMsg() const
  {
    return info_msg_;
  }

  void setInfoMsg(const sensor_msgs::CameraInfoConstPtr& infoMsg)
  {
    info_msg_ = infoMsg;
  }

  void signalDepthMsg(const sensor_msgs::ImagePtr& imgMsgDepth){
    subscriber_depth_.newMessage(imgMsgDepth);

  }

  void signalRgbMsg(const sensor_msgs::ImageConstPtr& imgMsgRgb){
    subscriber_rgb_.newMessage(imgMsgRgb);

  }

};

} /* namespace cvssp_tools */

#endif /* CVSSP_TOOLS_SRC_KINECTSUBSCRIBER_H_ */
