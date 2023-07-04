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

#include <nav_msgs/Path.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "pcl_ros/point_cloud.h"
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

#include <geometry_msgs/PoseArray.h>

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

#include <cvssp_tools/Camera.h>
#include <cvssp_tools/localiser_tools.h>

typedef tf::Stamped<tf::Pose> PoseStamped;

void PoseVectorTFToPoseArrayMsg(const std::vector<PoseStamped>& poses, geometry_msgs::PoseArray& msg)
{
  msg.poses.resize(poses.size());
  for (uint i = 0; i < poses.size(); i++)
  {
    tf::poseTFToMsg((tf::Pose)poses[i], msg.poses[i]);
  }
}
void PoseVectorTFToPathMsg(const std::vector<PoseStamped>& poses, nav_msgs::Path& msg)
{
  msg.poses.resize(poses.size());
  for (uint i = 0; i < poses.size(); i++)
  {
    tf::poseStampedTFToMsg(poses[i], msg.poses[i]);
  }
}
void applyGlobalTf(const tf::Transform& T, std::vector<PoseStamped>& poses)
{
  for (uint i = 0; i < poses.size(); i++)
  {
    poses[i] = PoseStamped(T * poses[i], poses[i].stamp_, poses[i].frame_id_);
  }
}
void applyLocalTf(const tf::Transform& T, std::vector<PoseStamped>& poses)
{
  for (uint i = 0; i < poses.size(); i++)
  {
    poses[i] = PoseStamped(poses[i] * T, poses[i].stamp_, poses[i].frame_id_);
  }
}
void ReadPosesFromFile(const std::string& dataPath, std::vector<PoseStamped>& poses)
{
  std::ifstream file;
  file.open(dataPath.c_str());

  int ctr = 0;
  double time, x, y, z, qx, qy, qz, qw;
  while (!file.eof())
  {
    //1495719972.380147037 0.0218887 5.2675 0 0 0 -0.531428 0.847103
    file >> time >> x >> y >> z >> qx >> qy >> qz >> qw;
    ROS_DEBUG("L: %4i S: %20.9f T:[%7.4f,%7.4f, %7.4f] R:[%7.4f, %7.4f, %7.4f, %7.4f]", ctr++, time, x, y, z, qx, qy,
              qz, qw);
    poses.push_back(
        PoseStamped(tf::Pose(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x, y, z)), ros::Time(time), "map"));
  }
  file.close();
}
void interpolate(const PoseStamped& start, const double& stepSize, const PoseStamped& end,
                 std::vector<PoseStamped>& path)
{

  int steps = rint(start.getOrigin().distance(end.getOrigin()) / stepSize) - 1;

  if (steps > 1)
  {

    path.resize(steps);

    double stepRatio = 1.0 / (double)steps;
    tf::Vector3 vdir = start.getOrigin().lerp(end.getOrigin(), stepRatio) - start.getOrigin();
    tf::Quaternion qdir = start.getRotation().slerp(end.getRotation(), stepRatio) - start.getRotation();

    path[0].setOrigin(start.getOrigin() + vdir);
    path[0].setRotation(start.getRotation() + qdir);
    for (int i = 1; i < steps; i++)
    {
      path[i].setOrigin(path[i - 1].getOrigin() + start.getOrigin().lerp(end.getOrigin(), stepRatio) - start.getOrigin());
      path[i].setRotation(path[i - 1].getRotation() + start.getRotation().slerp(end.getRotation(), stepRatio) - start.getRotation());
    }
  }
}
void interpolate(std::vector<PoseStamped>& poses, double minDist)
{
  for (uint i = 0; i < poses.size() - 1; i++)
  {
    std::vector<PoseStamped> intPoses;
    interpolate(poses[i], minDist, poses[i + 1], intPoses);
    poses.insert(poses.begin() + i, intPoses.begin(), intPoses.end());
    i += (intPoses.size());
  }
}
void GetMsgs(const std::string& dataPath, const std::string& frame_id, geometry_msgs::PoseArray& pose_msg,
             nav_msgs::Path& path_msg, bool inter=false, tf::Transform globalTf = tf::Transform::getIdentity(), tf::Transform localTf =
                 tf::Transform::getIdentity())
{

  std::vector<PoseStamped> poses;
  ReadPosesFromFile(dataPath, poses);
  applyGlobalTf(globalTf, poses);
  applyLocalTf(localTf, poses);

  if(inter){
    ROS_INFO("PreInt: %i", poses.size());
    interpolate(poses, 0.05);
    ROS_INFO("PostInt: %d", poses.size());
  }

  pose_msg.header.frame_id = frame_id;
  pose_msg.header.stamp = ros::Time::now();
  PoseVectorTFToPoseArrayMsg(poses, pose_msg);

  path_msg.header.frame_id = frame_id;
  path_msg.header.stamp = ros::Time::now();
  PoseVectorTFToPathMsg(poses, path_msg);
}
int main(int argc, char *argv[])
{
  // Initialize ROS
  ros::init(argc, argv, "PoseVisualiser");

  ros::NodeHandle nh;

  ros::Publisher gt_path_pub = nh.advertise<nav_msgs::Path>(nh.resolveName("GT/Path"), 1);
  ros::Publisher gt_pose_pub = nh.advertise<geometry_msgs::PoseArray>(nh.resolveName("GT/Poses"), 1);

  ros::Publisher amcl_path_pub = nh.advertise<nav_msgs::Path>(nh.resolveName("AMCL/Path"), 1);
  ros::Publisher amcl_pose_pub = nh.advertise<geometry_msgs::PoseArray>(nh.resolveName("AMCL/Poses"), 1);

  ros::Publisher data_path_pub = nh.advertise<nav_msgs::Path>(nh.resolveName("MCL/Path"), 1);
  ros::Publisher data_pose_pub = nh.advertise<geometry_msgs::PoseArray>(nh.resolveName("MCL/Poses"), 1);

  std::string dataPath;
  if (!ros::param::get("~dataPath", dataPath))
  {
    ROS_ERROR("No Data Path!");
    return 1;
  }

  std::string amclPath;
  if (!ros::param::get("~amclPath", amclPath))
  {
    ROS_ERROR("No AMCL Path!");
    return 1;
  }

  std::string gtPath;
  if (!ros::param::get("~gtPath", gtPath))
  {
    ROS_ERROR("No Ground Truth Path!");
    return 1;
  }

  tf::Transform gtTf(tf::Quaternion(0.500, -0.500, 0.500, 0.500), tf::Vector3(0.0, 0.0, 0.0));

  //Rtab Rate 1Hz
  tf::Matrix3x3 R(-0.225, -0.974, 0.0, 0.974, -0.225, 0.0, -0.000, 0.000, 1.0);
  tf::Vector3 t(-0.019, 0.089, 0.0);

  // Rtab Rate 5Hz
  /*tf::Matrix3x3 R(-0.317, -0.949,  0.000,
   0.949, -0.317,  0.000,
   0.000,  0.000,  1.000);
   tf::Vector3 t(-0.043,-0.045,0.0);*/
  tf::Quaternion q;
  R.getRotation(q);
  geometry_msgs::PoseArray gt_pose_msg;
  nav_msgs::Path gt_path_msg;
  GetMsgs(gtPath, "map", gt_pose_msg, gt_path_msg, true, tf::Transform(q, tf::Vector3(t)), gtTf);

  geometry_msgs::PoseArray data_pose_msg;
  nav_msgs::Path data_path_msg;
  GetMsgs(dataPath, "map", data_pose_msg, data_path_msg);

  geometry_msgs::PoseArray amcl_pose_msg;
  nav_msgs::Path amcl_path_msg;
  GetMsgs(amclPath, "map", amcl_pose_msg, amcl_path_msg);

  //tf::Transform(tf::Quaternion(0.0,0.0,0.135767,0.990741),tf::Vector3(0.341724, 0.045332, 0)).inverse()

  do
  {
    gt_path_pub.publish(gt_path_msg);
    gt_pose_pub.publish(gt_pose_msg);

    data_path_pub.publish(data_path_msg);
    data_pose_pub.publish(data_pose_msg);

    amcl_path_pub.publish(amcl_path_msg);
    amcl_pose_pub.publish(amcl_pose_msg);

    ros::spinOnce();
    ros::Rate(1).sleep();

  } while (ros::ok());

}

