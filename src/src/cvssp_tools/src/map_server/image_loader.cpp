/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This file contains helper functions for loading images as maps.
 *
 * Author: Brian Gerkey
 */

#include <cstring>
#include <stdexcept>

#include <stdlib.h>
#include <stdio.h>

// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>

#include "cvssp_tools/map_server/image_loader.h"
#include <tf/tf.h>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace cvssp_tools
{
namespace map_server
{

void loadMapFromFile(cvssp_tools::GetMap::Response* resp, const char* fname, double res, bool negate, double occ_th,
                     double free_th, double* origin, MapMode mode)
{
  SDL_Surface* img;

  unsigned char* pixels;
  unsigned char* p;
  unsigned char value;
  int rowstride, n_channels, avg_channels;
  unsigned int i, j;
  int k;
  double occ;
  int alpha;
  int color_sum;
  double color_avg;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  if (!(img = IMG_Load(fname)))
  {
    std::string errmsg = std::string("failed to open image file \"") + std::string(fname) + std::string("\"");
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  resp->map.info.width = img->w;
  resp->map.info.height = img->h;
  resp->map.info.resolution = res;
  resp->map.info.origin.position.x = *(origin);
  resp->map.info.origin.position.y = *(origin + 1);
  resp->map.info.origin.position.z = 0.0;
  tf::Quaternion q;
  q.setRPY(0, 0, *(origin + 2));
  resp->map.info.origin.orientation.x = q.x();
  resp->map.info.origin.orientation.y = q.y();
  resp->map.info.origin.orientation.z = q.z();
  resp->map.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  resp->map.data.resize(resp->map.info.width * resp->map.info.height);
  resp->map.label.resize(resp->map.info.width * resp->map.info.height);

  // Get values that we'll need to iterate through the pixels
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  if (mode == TRINARY || !img->format->Amask)
    avg_channels = n_channels;
  else
    avg_channels = n_channels - 1;

  // Copy pixel data into the map structure
  pixels = (unsigned char*)(img->pixels);
  for (j = 0; j < resp->map.info.height; j++)
  {
    for (i = 0; i < resp->map.info.width; i++)
    {
      // Compute mean of RGB for this pixel
      p = pixels + j * rowstride + i * n_channels;
      color_sum = 0;
      for (k = 0; k < avg_channels; k++)
        color_sum += *(p + (k));
      color_avg = color_sum / (double)avg_channels;

      if (n_channels == 1)
        alpha = 1;
      else
        alpha = *(p + n_channels - 1);

      if (negate)
        color_avg = 255 - color_avg;

      if (mode == RAW)
      {
        value = color_avg;
        resp->map.data[MAP_IDX(resp->map.info.width, i, resp->map.info.height - j - 1)] = value;
        continue;
      }

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      occ = (255 - color_avg) / 255.0;

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if (occ > occ_th)
        value = +100;
      else if (occ < free_th)
        value = 0;
      else if (mode == TRINARY || alpha < 1.0)
        value = -1;
      else
      {
        double ratio = (occ - free_th) / (occ_th - free_th);
        value = 99 * ratio;
      }

      resp->map.data[MAP_IDX(resp->map.info.width, i, resp->map.info.height - j - 1)] = value;

      //Get Labels
      int r = *(p + (0));
      int g = *(p + (1));
      int b = *(p + (2));

      int8_t label = 0; //NULL Label
      if((r==0) && (g == 0) && (b == 0)) 
        label = 1; //WALL
      else if ((r == 0) && (g == 0) && (b == 255))
        label = 8; //DOOR
      else if ((r == 255) && (g == 0) && (b == 0))
        label = 9; //WINDOW
      else if ((r == 246) && (g == 0) && (b == 223))
        label = 10; //PAINTED_WOOD
      else if ((r == 246) && (g == 223) && (b == 0))
        label = 11; //METAL
      else if ((r == 149) && (g == 62) && (b == 0))
        label = 12;//PLASTIC
      else if ((r == 0) && (g == 255) && (b == 255))
        label = 98; //STAIRS
      else if ((r == 74) && (g == 149) && (b == 93))
        label = 99; //MISC
      resp->map.label[MAP_IDX(resp->map.info.width, i, resp->map.info.height - j - 1)] = label;
#ifdef VIS_SEM_MAP
      ROS_DEBUG("R: %i, G: %i, B: %i", r, g, b);
      if (label == 1)
        ROS_DEBUG("WALL!");

      else if (label == 8)
        ROS_DEBUG("DOOR!");

      else if (label == 9)
        ROS_DEBUG("WINDOW!");

      else if (label == 10)
        ROS_DEBUG("P_WOOD!");

      else if (label == 11)
        ROS_DEBUG("METAL!");

      else if (label == 12)
        ROS_DEBUG("PLASTIC!");

      else if (label == 98)
        ROS_DEBUG("STAIRS!");
        
      else if (label == 99)
        ROS_DEBUG("MISC!");
#endif
    }
  }

  SDL_FreeSurface(img);
}

}
}
