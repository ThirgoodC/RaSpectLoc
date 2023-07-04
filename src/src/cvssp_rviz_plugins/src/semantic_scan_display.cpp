/*
 * semantic_scan_display.cpp
 *
 *  Created on: 27 Mar 2018
 *      Author: om0007
 */

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <ros/time.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <laser_geometry/laser_geometry.h>

#include "rviz/default_plugin/point_cloud_common.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/properties/int_property.h"
#include "rviz/validate_floats.h"
// #include <visualization_manager.h>

#include "semantic_scan_display.h"

namespace cvssp_rviz_plugins
{

SemanticScanDisplay::SemanticScanDisplay() :
    point_cloud_common_(new rviz::PointCloudCommon(this)), projector_(new laser_geometry::LaserProjection())
{
  // queue_size_property_ = new rviz::IntProperty(
  //     "Queue Size", 10, "Advanced: set the size of the incoming SemanticScan message queue. "
  //     " Increasing this is useful if your incoming TF data is delayed significantly "
  //     "from your SemanticScan data, but it can greatly increase memory usage if the messages are big.",
  //     this, SLOT(updateQueueSize()));

  // PointCloudCommon sets up a callback queue with a thread for each
  // instance.  Use that for processing incoming messages.

  // update_nh_.setCallbackQueue(point_cloud_common_->getCallbackQueue()); //Figure this out <-- deals with the callback queue
  std::string package_path = ros::package::getPath("cvssp_rviz_plugins");
  std::string label_colours_path = package_path + "/media/colours/sun.png";
  label_colours_ = cv::imread(label_colours_path, 1);
}

SemanticScanDisplay::~SemanticScanDisplay()
{
  SemanticScanDisplay::unsubscribe();
  delete point_cloud_common_;
  delete projector_;
}

void SemanticScanDisplay::onInitialize()
{
  update_nh_.setCallbackQueue(context_->getThreadedQueue());

  queue_size_property_ = new rviz::IntProperty(
    "Queue Size", 10, "Advanced: set the size of the incoming SemanticScan message queue. "
    " Increasing this is useful if your incoming TF data is delayed significantly "
    "from your SemanticScan data, but it can greatly increase memory usage if the messages are big.",
    this, SLOT(updateQueueSize()));

  MFDClass::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);
}

void SemanticScanDisplay::updateQueueSize()
{
  tf_filter_->setQueueSize((uint32_t)queue_size_property_->getInt());
}

void SemanticScanDisplay::processMessage(const cvssp_tools::SemanticScanConstPtr& scan)
{
  sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);

  std::string frame_id = scan->header.frame_id;

  // Compute tolerance necessary for this scan
  ros::Duration tolerance(scan->time_increment * scan->ranges.size());
  if (tolerance > filter_tolerance_)
  {
    filter_tolerance_ = tolerance;
    tf_filter_->setTolerance(filter_tolerance_);
  }

  try
  {
    sensor_msgs::LaserScanPtr laserScan(new sensor_msgs::LaserScan);
    laserScan->header = scan->header;
    // laserScan->header.frame_id = "map";
    laserScan->angle_increment = scan->angle_increment;
    laserScan->angle_max = scan->angle_max;
    laserScan->angle_min = scan->angle_min;
    laserScan->range_max = scan->range_max;
    laserScan->range_min = scan->range_min;
    laserScan->ranges = scan->ranges; //Shallow copy?
    laserScan->scan_time = scan->scan_time;
    laserScan->time_increment = scan->time_increment;

    // ROS_WARN("Will this work?:: %s", fixed_frame_.toStdString());

    // std::cout << "What is this: " << fixed_frame_.toStdString() << std::endl;

    projector_->transformLaserScanToPointCloud(fixed_frame_.toStdString(), 
                                               *laserScan,
                                               *cloud,
                                               *context_->getTF2BufferPtr()
                                               );


    // projector_->transformLaserScanToPointCloud(fixed_frame_.toStdString(), *scan, *cloud, *tf, -1.0,
    //                                            laser_geometry::channel_option::Intensity);

    //Add Colour
    int offset = 12;
    int field_size = cloud->fields.size();
    cloud->fields.resize(field_size + 1);
    cloud->fields[field_size].name = "rgb";
    cloud->fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
    cloud->fields[field_size].offset = offset;
    cloud->fields[field_size].count = 1;
    int idx_colour = field_size;

    double range_cutoff = scan->range_max;

    sensor_msgs::PointCloud2Iterator<uint8_t> out_r(*cloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> out_g(*cloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> out_b(*cloud, "b");
    for (uint i = 0; i < scan->ranges.size(); i++)
    {
      if (scan->ranges[i] <= range_cutoff && scan->ranges[i] >= scan->range_min)
      {
        cv::Vec3b colour = label_colours_.at<cv::Vec3b>(scan->labels[i]);
        *out_r = colour[2];
        *out_g = colour[1];
        *out_b = colour[0];

        ++out_r;
        ++out_g;
        ++out_b;
      }
    }

  }
  catch (tf::TransformException& e)
  {
    ROS_DEBUG(
        "SemanticScan [%s]: failed to transform scan: %s.  This message should not repeat (tolerance should now be set on our tf::MessageFilter).",
        qPrintable( getName() ), e.what());
    return;
  }

  point_cloud_common_->addMessage(cloud);
}

void SemanticScanDisplay::update(float wall_dt, float ros_dt)
{
  point_cloud_common_->update(wall_dt, ros_dt);
}

void SemanticScanDisplay::reset()
{
  MFDClass::reset();
  point_cloud_common_->reset();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cvssp_rviz_plugins::SemanticScanDisplay, rviz::Display)
