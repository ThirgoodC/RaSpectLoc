/*
 * Ray.h
 *
 *  Created on: 30 Jul 2018
 *      Author: Oscar Mendez <o.mendez@surrey.ac.uk>
 */

#ifndef CVSSP_TOOLS_SRC_RAY_H_
#define CVSSP_TOOLS_SRC_RAY_H_

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace cvssp_tools
{

class Ray
{
  Eigen::Vector2d pix_;
  Eigen::Vector3d dir_;
  double angle_;
  double range_;
  uint8_t label_;

public:
  Ray() :
      angle_(0.0), range_(0.0), label_(0)
  {

  }
  Eigen::Vector2d& pix()
  {
    return pix_;
  }

  Eigen::Vector3d& dir()
  {
    return dir_;
  }

  double& angle()
  {
    return angle_;
  }

  double& range()
  {
    return range_;
  }

  uint8_t& label()
  {
    return label_;
  }
  bool operator<(const Ray& other) const
  {
    return angle_ < other.angle_;
  }

};

} /* namespace cvssp_tools */

#endif /* CVSSP_TOOLS_SRC_RAY_H_ */
