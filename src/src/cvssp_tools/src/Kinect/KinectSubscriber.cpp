/*
 * KinectSubscriber.cpp
 *
 *  Created on: 11 May 2017
 *      Author: footstool
 */

#include "cvssp_tools/Kinect/KinectSubscriber.h"

namespace cvssp_tools
{

KinectSubscriber::KinectSubscriber() :
    it_(nh_), sync_(MySyncPolicy(1000), subscriber_rgb_, subscriber_depth_), time_accum(0.0)
{

#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
  subscriber_rgb_.subscribe(it_, nh_.resolveName("rgb_image"), 1,image_transport::TransportHints("compressed"));
  subscriber_depth_.subscribe(it_, nh_.resolveName("depth_image"), 1,image_transport::TransportHints("compressedDepth"));
#else
  subscriber_depth_.subscribe(nh_, nh_.resolveName("depth_image"), 1);
  subscriber_rgb_.subscribe(nh_, nh_.resolveName("rgb_image"), 1);
#endif

  sync_.registerCallback(boost::bind(&KinectSubscriber::imageSyncCb, this, _1, _2));

  cam_info_sub_ = nh_.subscribe(nh_.resolveName("cam_info"), 1, &KinectSubscriber::camInfoCb, this);

  this->setImgMsgDepth(nullptr);
  this->setImgMsgRgb(nullptr);
  this->setImgPtrDepth(nullptr);
  this->setImgPtrRgb(nullptr);

  ROS_INFO("Image: %s", subscriber_rgb_.getTopic().c_str());
  ROS_INFO("Depth: %s", subscriber_depth_.getTopic().c_str());
}

KinectSubscriber::~KinectSubscriber()
{
  // TODO Auto-generated destructor stub
}
void KinectSubscriber::camInfoCb(const sensor_msgs::CameraInfoConstPtr& msg)
{
  if (!this->isInit())
  {
    this->setInfoMsg(msg);
    this->setCamInfo(*msg);
    postProcessInfo();
    ROS_INFO("Got CamInfo: [%5.2f %5.2f %5.2f %5.2f]", this->getCamModel().fx(), this->getCamModel().fy(), this->getCamModel().cx(), this->getCamModel().cy());
  }
  else
  {
    ROS_INFO("CamInfo Unsubscribing!");
    cam_info_sub_.shutdown();
  }
}

void KinectSubscriber::imageSyncCb(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth)
{
  this->setImgMsgDepth(msg_depth);
  this->setImgMsgRgb(msg_rgb);

  try
  {
    img_ptr_depth = cv_bridge::toCvCopy(*msg_depth);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }
  try
  {
    img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  this->setDepth(img_ptr_depth->image);
  this->setDepthHeader(msg_depth->header);
  this->setColor(img_ptr_rgb->image);
  this->setImageHeader(msg_rgb->header);

  ROS_DEBUG_STREAM("Image Callback!");
  ros::Time start = ros::Time::now();
  postProcessImages();

  ROS_DEBUG_STREAM("Time: "<<(ros::Time::now()-start).toSec());
  double dur = (ros::Time::now() - start).toSec();
  double alpha = 0.01;
  time_accum = (alpha * dur) + (1.0 - alpha) * time_accum;
  ROS_DEBUG("IterTime: %f", time_accum);
}

} /* namespace cvssp_tools */
