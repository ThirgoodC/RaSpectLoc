/*
 * RTabMapData.h
 *
 *  Created on: 25 Apr 2017
 *      Author: footstool
 */

#ifndef CVSSP_LOCALISER_SRC_RTABMAPDATA_H_
#define CVSSP_LOCALISER_SRC_RTABMAPDATA_H_

#include <ros/ros.h>
#include <ros/common.h>
#include <ros/console.h>
#include <geometry_msgs/PoseArray.h>

#include "opencv2/highgui/highgui.hpp"

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstdlib>
#include <string>

#include <camera_calibration_parsers/parse_yml.h>
#include <camera_calibration_parsers/parse.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

#include <cvssp_tools/Camera.h>
#include <cvssp_tools/localiser_tools.h>

namespace cvssp_tools
{

class RTabMapData
{

public:
  RTabMapData(std::string& path)
  {
    if ((*path.rbegin()) != '/')
      path.push_back('/');

    posePath = path + "poses.txt";
    calibPath = path + "calibration.yaml";
    rgbPath = path + "rgb/";
    semPath = path + "semantic/";
    depthPath = path + "depth/";

    ROS_INFO("Reading from Path: %s", path.c_str());

    std::vector<tf::Pose> poses;
    if (!loadPoses(poses))
    {
      ROS_ERROR("Error loading Poses!");
      exit(1);
    }

    int numPoses = poses.size();
    int numRGBImages = countFilesInDir(rgbPath);
    int numDepthImages = countFilesInDir(depthPath);
    int numSemImages = countFilesInDir(semPath);

    if (numRGBImages != numPoses)
    {
      ROS_ERROR("Error loading RGB Images!");
      ROS_ERROR("Got %i", numRGBImages);
      ROS_ERROR("Expected %i", numPoses);
      exit(1);
    }
    if (numDepthImages != numPoses)
    {
      ROS_ERROR("Error loading Depth Images!");
      ROS_ERROR("Got %i", numDepthImages);
      ROS_ERROR("Expected %i", numPoses);
      exit(1);
    }
    if (numSemImages != numPoses)
    {
      ROS_ERROR("Error loading Semantic Images!");
      ROS_ERROR("Got %i", numSemImages);
      ROS_ERROR("Expected %i", numPoses);
      exit(1);
    }
    loadImageNames();

    //Get Calib Params
    std::string cam_name;
    sensor_msgs::CameraInfo camInfo;
    bool read = camera_calibration_parsers::readCalibration(calibPath, cam_name, camInfo);

    double scale = 0.75; //Between Normal and Semantic Images
    camInfo.width *= scale;
    camInfo.height *= scale;

    camInfo.K[0] *= scale;
    camInfo.K[2] *= scale;
    camInfo.K[4] *= scale;
    camInfo.K[5] *= scale;

    camInfo.P[0] *= scale;
    camInfo.P[2] *= scale;
    camInfo.P[4] *= scale;
    camInfo.P[5] *= scale;

    //Assign Camera's Common Elements
    std::vector<cv::Point2d> pts2d = generate2DSamples(camInfo.width, camInfo.height,
                                                       0.01 * (camInfo.width * camInfo.height));
    cameras.resize(numPoses, cvssp_tools::Camera(camInfo));

    //Set Poses and Rays
    for(uint i=0;i<cameras.size();i++) {
      cameras[i].setPose(poses[i]);
      cameras[i].generateRays(pts2d);
    }
  }
  virtual ~RTabMapData()
  {
  }

  geometry_msgs::PoseArray getPosesMsg()
  {
    geometry_msgs::PoseArray msg;
    msg.header.frame_id = "data";
    msg.header.stamp = ros::Time::now();
    msg.header.seq = 0;
    msg.poses.resize(cameras.size());
    for (uint i = 0; i < cameras.size(); i++)
    {
      tf::poseTFToMsg(cameras[i].getPose(), msg.poses[i]);
    }
    return msg;
  }

  int getNumCams()
  {
    return cameras.size();
  }

  cv::Mat& getImage(cv::Mat& image, const std::string& path, const int& frame=0)
    {

      if (image.empty())
      {
        bool success = readImage(image, path);
        if (!success)
        {
          ROS_ERROR("Failed to load image: %i!", frame);
          exit(1);
        }
      int w = cameras[frame].getCamModel().cameraInfo().width;
      int h = cameras[frame].getCamModel().cameraInfo().height;
      cv::resize(image, image, cv::Size(w, h), 0.0, 0.0, cv::INTER_AREA);
      }

      return image;
    }
  /* cv::Mat& getImage(const int& frame, std::vector<cv::Mat>& images, std::vector<std::string>& paths)
  {

    if (images[frame].empty())
    {
      bool success = readImage(images[frame], paths[frame]);
      if (!success)
      {
        ROS_ERROR("Failed to load image: %i!", frame);
        exit(1);
      }
    }

    return images[frame];
   }*/
  cv::Mat& getRGBImage(const int& frame)
  {
    return getImage(cameras[frame].getColor(), rgbImagePaths[frame],frame);
  }
  cv::Mat& getDepthImage(const int& frame)
  {
    return getImage(cameras[frame].getDepth(), depthImagePaths[frame],frame);
  }
  cv::Mat& getSemImage(const int& frame)
  {
    return getImage(cameras[frame].getLabel(), semImagePaths[frame],frame);
  }

  cvssp_tools::Camera& getCam(const int& frame)
  {
    return cameras[frame];
  }

private:
  std::string posePath;

  std::string rgbPath;
  std::string semPath;
  std::string depthPath;
  std::string calibPath;

  std::vector<cvssp_tools::Camera> cameras;

  std::vector<std::string> rgbImagePaths;
  std::vector<std::string> depthImagePaths;
  std::vector<std::string> semImagePaths;

  struct sortByImageNumber
  {
    bool operator ()(const std::string & a, const std::string & b)
    {
      return std::atoi(a.substr(a.find_last_of('/') + 1, a.find_last_of('.') - a.find_last_of('/')).c_str())
          < std::atoi(b.substr(b.find_last_of('/') + 1, b.find_last_of('.') - b.find_last_of('/')).c_str()); // or some custom code
    }
  };

  std::vector<std::string> getFileList(const std::string& path)
  {
    std::vector<std::string> fileList;
    if (!path.empty())
    {
      namespace fs = boost::filesystem;

      fs::path apk_path(path);
      fs::recursive_directory_iterator end;

      for (fs::recursive_directory_iterator i(apk_path); i != end; ++i)
      {
        const fs::path cp = (*i);
        fileList.push_back(cp.string());
      }
    }
    std::sort(fileList.begin(), fileList.end(), sortByImageNumber());
    return fileList;
  }

  void loadImageNames()
  {

    rgbImagePaths = getFileList(rgbPath);
    depthImagePaths = getFileList(depthPath);
    semImagePaths = getFileList(semPath);
  }

  bool loadPoses(std::vector<tf::Pose>& poses)
  {
    std::ifstream poseFile;
    poseFile.open(posePath.c_str());
    if (!poseFile.good())
      return false;


    //tf::Quaternion toROSStd(tf::createQuaternionFromRPY(-1.5707,0.0,-1.5707));
    tf::Matrix3x3 toROSStd(0, -1, 0,
                           0, 0,-1,
                           1, 0, 0);
    while (!poseFile.eof())
    {

      double T[3][4];
      poseFile >> T[0][0] >> T[0][1] >> T[0][2] >> T[0][3];
      poseFile >> T[1][0] >> T[1][1] >> T[1][2] >> T[1][3];
      poseFile >> T[2][0] >> T[2][1] >> T[2][2] >> T[2][3];

      tf::Matrix3x3 R(T[0][0], T[0][1], T[0][2], T[1][0], T[1][1], T[1][2], T[2][0], T[2][1], T[2][2]);

      //Rotate to ROS Standard
      //tf::Quaternion q;
      //R.getRotation(q);
      //R.setRotation(toROSStd*q);

      R=R*toROSStd.inverse();

      if (poseFile.eof())
        continue;

      poses.push_back(tf::Pose(R, tf::Vector3(T[0][3], T[1][3], T[2][3])));

    }

    return true;
  }

  bool readImage(cv::Mat& image, const std::string& imPath)
  {
    image = cv::imread(imPath);

    if (image.empty())
      return false;

    return true;
  }
  int countFilesInDir(const std::string& filename)
  {
    boost::filesystem::path path(filename);
    return std::count_if(boost::filesystem::directory_iterator(path), boost::filesystem::directory_iterator(),
                         static_cast<bool (*)(const boost::filesystem::path&)>(boost::filesystem::is_regular_file));}
                       };

                     }
                     /* namespace cvssp_tools */

#endif /* CVSSP_LOCALISER_SRC_RTABMAPDATA_H_ */
