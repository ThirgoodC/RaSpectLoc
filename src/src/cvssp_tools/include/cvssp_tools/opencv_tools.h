/*
 * opencv_tools.h
 *
 *  Created on: 6 Mar 2018
 *      Author: om0007
 */

#ifndef CVSSP_TOOLS_INCLUDE_CVSSP_TOOLS_OPENCV_TOOLS_H_
#define CVSSP_TOOLS_INCLUDE_CVSSP_TOOLS_OPENCV_TOOLS_H_

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_calibration_parsers/parse.h>


namespace cvssp_tools{
	std::string type2str(int type) {
		std::string r;

	  uchar depth = type & CV_MAT_DEPTH_MASK;
	  uchar chans = 1 + (type >> CV_CN_SHIFT);

	  switch ( depth ) {
		case CV_8U:  r = "8U"; break;
		case CV_8S:  r = "8S"; break;
		case CV_16U: r = "16U"; break;
		case CV_16S: r = "16S"; break;
		case CV_32S: r = "32S"; break;
		case CV_32F: r = "32F"; break;
		case CV_64F: r = "64F"; break;
		default:     r = "User"; break;
	  }

	  r += "C";
	  r += (chans+'0');

	  return r;
	}
}


#endif /* CVSSP_TOOLS_INCLUDE_CVSSP_TOOLS_OPENCV_TOOLS_H_ */
