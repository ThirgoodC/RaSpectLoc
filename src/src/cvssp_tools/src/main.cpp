#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>

#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <ros/callback_queue.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <set>
#include <std_msgs/UInt8.h>
#include <std_srvs/SetBool.h>
#include <string>
#include <tf_conversions/tf_eigen.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <vector>

#include <cvssp_tools/RTabMapData.h>

int main(int argc, char *argv[])
{
  ROS_INFO_STREAM("Starting Semantic Localisation Node.");

  // Initialize ROS
  ros::init(argc, argv, "SemanticLocalisation");

  ros::NodeHandle nh;

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(nh.resolveName("RTabMap/Pose"), 1);
  ros::Publisher data_pose_pub = nh.advertise<geometry_msgs::PoseArray>(nh.resolveName("RTabMap/Data"), 1);
  ros::Publisher rays_pub = nh.advertise<visualization_msgs::MarkerArray>(nh.resolveName("RTabMap/Rays"), 1);

  ros::Publisher scan_pub = nh.advertise<cvssp_tools::SemanticScan>(nh.resolveName("RTabMap/Scan"), 1);

  std::string dataPath;
  if (!ros::param::get("~dataPath", dataPath))
  {
    ROS_ERROR("No Data Path!");
    return 1;
  }

  cvssp_tools::RTabMapData data(dataPath);
  do
  {

    data_pose_pub.publish(data.getPosesMsg());
    ros::spinOnce();

#ifndef NDEBUG
    cv::namedWindow("RGB");
    cv::namedWindow("Depth");
    cv::namedWindow("Sem");
#endif
    for (int i = 70; (i < data.getNumCams()) && ros::ok(); i)
    {
      visualization_msgs::MarkerArray m;
      data.getCam(i).getRaysMsg(m, 1.0);
      rays_pub.publish(m);

      //Load Data (because lasy loading)
      cv::Mat rgb = data.getRGBImage(i);
      cv::Mat dph = data.getDepthImage(i);
      cv::Mat lbl = data.getSemImage(i);

#ifndef NDEBUG
      ROS_INFO("Loading Image: %i", i);
      cv::imshow("RGB", rgb);
      cv::imshow("Depth", dph);
      cv::imshow("Sem", lbl);
      int k = cv::waitKey(30);
      if (k == 27 || !ros::ok())
      {
        ros::shutdown();
        break;
      }
#endif
      data_pose_pub.publish(data.getPosesMsg());

      cvssp_tools::SemanticScan scanMsg;
      data.getCam(i).getSemanticScanMsg(scanMsg);
      scanMsg.header.seq = i;
      scan_pub.publish(scanMsg);

      ros::Rate(10).sleep();
      ros::spinOnce();

      //TODO: Publish Transform, images and cam_info
      //rosrun tf static_transform_publisher 9.75 9.5 0 -2.15 -0.4 0.05 map data 100
    }

    ros::Rate(10).sleep();
  } while (ros::ok());
}

