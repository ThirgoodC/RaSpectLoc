/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Local map GUI functions
 * Author: Andrew Howard
 * Date: 18 Jan 2003
 * CVS: $Id: map_draw.c 7057 2008-10-02 00:44:06Z gbiggs $
 **************************************************************************/
#include <cstdlib>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "map.h"

void map_draw_cspace_opencv(map_t *map)
{

  int rows = map->size_y;
  int cols = map->size_x;
  cv::Mat cmap(cv::Mat::zeros(rows, cols, CV_8UC1));

  // Draw occupancy
  for (int j = 0; j < rows; j++)
  {
    for (int i = 0; i < cols; i++)
    {
      map_cell_t *cell = map->cells + MAP_INDEX(map, i, j);
      cmap.at<uint8_t>((rows-1)-j, i) = 255 * cell->occ_dist /  map->max_occ_dist;
    }
  }

  cv::imwrite("/home/footstool/dist_depth.png",cmap);
/*
  cv::imshow("CSpace", cmap);
  cv::waitKey(0);*/
}

void map_draw_label_cspace_opencv(map_t *map, int label)
{

  int rows = map->size_y;
  int cols = map->size_x;
  cv::Mat cmap(cv::Mat::zeros(rows, cols, CV_8UC1));

  int idx=LabelToIndex(label);

  // Draw occupancy
  for (int j = 0; j < rows; j++)
  {
    for (int i = 0; i < cols; i++)
    {
      map_cell_t *cell = map->cells + MAP_INDEX(map, i, j);
      cmap.at<uint8_t>((rows-1)-j, i) = 255 * cell->label_dist[idx] /  map->max_label_dist;
    }
  }
  cv::imwrite(SSTR("/home/footstool/dist_label_"<<label<<".png"),cmap);
/*
  cv::imshow(("CSpace"+label), cmap);
  cv::waitKey(0);*/
}
