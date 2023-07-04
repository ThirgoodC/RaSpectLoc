/*
 * OfflineCamera.h
 *
 *  Created on: 30 Jul 2018
 *      Author: Oscar Mendez <o.mendez@surrey.ac.uk>
 */

#ifndef CVSSP_TOOLS_INCLUDE_CVSSP_TOOLS_OFFLINECAMERA_H_
#define CVSSP_TOOLS_INCLUDE_CVSSP_TOOLS_OFFLINECAMERA_H_

#include <cvssp_tools/Camera.h>

namespace cvssp_tools
{

class OfflineCamera : public Camera
{
private:
  std::string color_path_;
  std::string depth_path_;
  std::string label_path_;

public:
  OfflineCamera();
  virtual ~OfflineCamera();

  const std::string& getColorPath() const
  {
    return color_path_;
  }

  void setColorPath(const std::string& colorPath)
  {
    color_path_ = colorPath;
  }

  const std::string& getDepthPath() const
  {
    return depth_path_;
  }

  void setDepthPath(const std::string& depthPath)
  {
    depth_path_ = depthPath;
  }

  const std::string& getLabelPath() const
  {
    return label_path_;
  }

  void setLabelPath(const std::string& labelPath)
  {
    label_path_ = labelPath;
  }

  cv::Mat& loadImage(cv::Mat& image, const std::string& path, int flags=1){
    if(image.empty() && !path.empty()){
      image = cv::imread(path,flags);
    }
    return image;
  }

  cv::Mat& getColorImage()
  {
    return loadImage(getColor(),getColorPath());
  }

  cv::Mat& getLabelImage()
  {
    return loadImage(getLabel(),getLabelPath(),cv::IMREAD_ANYDEPTH);
  }
  cv::Mat& getDepthImage()
  {
    return loadImage(getDepth(),getDepthPath(),cv::IMREAD_ANYDEPTH);
  }
};

} /* namespace cvssp_tools */

#endif /* CVSSP_TOOLS_INCLUDE_CVSSP_TOOLS_OFFLINECAMERA_H_ */
