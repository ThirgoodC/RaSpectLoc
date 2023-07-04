/*
 * semantic_scan_display.h
 *
 *  Created on: 27 Mar 2018
 *      Author: om0007
 */

#ifndef CVSSP_TOOLS_PLUGINS_RVIZ_SEMANTIC_SCAN_DISPLAY_H_
#define CVSSP_TOOLS_PLUGINS_RVIZ_SEMANTIC_SCAN_DISPLAY_H_

#include <cvssp_tools/SemanticScan.h>

#include "opencv2/core.hpp"
#include <opencv2/imgcodecs.hpp>
#include "rviz/message_filter_display.h"

namespace laser_geometry
{
class LaserProjection;
}

namespace rviz
{

class IntProperty;
class PointCloudCommon;
}
namespace cvssp_rviz_plugins
{

/** @brief Visualizes a Semantic scan, received as a cvssp_tools::SemanticScan. */
class SemanticScanDisplay: public rviz::MessageFilterDisplay<cvssp_tools::SemanticScan>
{
Q_OBJECT
public:
  SemanticScanDisplay();
  ~SemanticScanDisplay();

  virtual void reset();

  virtual void update( float wall_dt, float ros_dt );

private Q_SLOTS:
  void updateQueueSize();

protected:
  /** @brief Do initialization. Overridden from MessageFilterDisplay. */
  virtual void onInitialize();

  /** @brief Process a single message.  Overridden from MessageFilterDisplay. */
  virtual void processMessage( const cvssp_tools::SemanticScanConstPtr& scan );

  rviz::IntProperty* queue_size_property_;

  rviz::PointCloudCommon* point_cloud_common_;

  laser_geometry::LaserProjection* projector_;
  ros::Duration filter_tolerance_;
  cv::Mat label_colours_;
};

} // namespace rviz

#endif /* CVSSP_TOOLS_PLUGINS_RVIZ_SEMANTIC_SCAN_DISPLAY_H_ */
