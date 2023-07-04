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
 * Desc: Range routines
 * Author: Andrew Howard
 * Date: 18 Jan 2003
 * CVS: $Id: map_range.c 1347 2003-05-05 06:24:33Z inspectorg $
 **************************************************************************/

#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "map.h"

void swap(int* x, int* y)
{
  int* tmp;
  tmp = x;
  x = y;
  y = tmp;
}

// Extract a single range reading from the map.  Unknown cells and/or
// out-of-bound cells are treated as occupied, which makes it easy to
// use Stage bitmap files.
double map_calc_range(map_t *map, double ox, double oy, double oa, double max_range, int* label)
{
  // Bresenham raytracing
  int x0, x1, y0, y1;
  int x, y;
  int xstep, ystep;
  char steep;
  int tmp;
  int deltax, deltay, error, deltaerr;

  x0 = MAP_GXWX(map, ox);
  y0 = MAP_GYWY(map, oy);

  x1 = MAP_GXWX(map, ox + max_range * cos(oa));
  y1 = MAP_GYWY(map, oy + max_range * sin(oa));

  if (abs(y1 - y0) > abs(x1 - x0))
    steep = 1;
  else
    steep = 0;

  if (steep)
  {
    tmp = x0;
    x0 = y0;
    y0 = tmp;

    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }

  deltax = abs(x1 - x0);
  deltay = abs(y1 - y0);
  error = 0;
  deltaerr = deltay;

  (*label) = -1;

  x = x0;
  y = y0;

  if (x0 < x1)
    xstep = 1;
  else
    xstep = -1;
  if (y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  if (steep)
  {
    if (!MAP_VALID(map, y, x) || map->cells[MAP_INDEX(map, y, x)].occ_state > -1)
    {
      if (MAP_VALID(map, y, x))
        (*label) = map->cells[MAP_INDEX(map, y, x)].label;
      return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
    }
  }
  else
  {
    if (!MAP_VALID(map, x, y) || map->cells[MAP_INDEX(map, x, y)].occ_state > -1)
    {
      if (MAP_VALID(map, x, y))
        (*label) = map->cells[MAP_INDEX(map, x, y)].label;
      return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
    }
  }

  while (x != (x1 + xstep * 1))
  {
    x += xstep;
    error += deltaerr;
    if (2 * error >= deltax)
    {
      y += ystep;
      error -= deltax;
    }

    if (steep)
    {
      if (!MAP_VALID(map, y, x) || map->cells[MAP_INDEX(map, y, x)].occ_state > -1)
      {
        if (MAP_VALID(map, y, x))
          (*label) = map->cells[MAP_INDEX(map, y, x)].label;
        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
      }
    }
    else
    {
      if (!MAP_VALID(map, x, y) || map->cells[MAP_INDEX(map, x, y)].occ_state > -1)
      {
        if (MAP_VALID(map, x, y))
          (*label) = map->cells[MAP_INDEX(map, x, y)].label;
        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
      }
    }
  }
  return max_range;
}
double calc_range(map_t *map, double ox, double oy, double oa, double max_range, int* mi, int* mj, int* ml)
{
  double distThreshold = 0.000001;
  double step_coeff = 0.999;

  int x0 = MAP_GXWX(map, ox);
  int y0 = MAP_GYWY(map, oy);

  double ray_direction_x = cosf(oa);
  double ray_direction_y = sinf(oa);

  int px = 0;
  int py = 0;

  double t = 0.0;
  double d;
  int map_valid;
  while (t < max_range)
  {
    px = x0 + ray_direction_x * MAP_GXWX(map, t);
    py = y0 + ray_direction_y * MAP_GYWY(map, t);

    map_valid=MAP_VALID(map, px, py);
    if (!map_valid || map->cells[MAP_INDEX(map, px, py)].occ_state > -1)
    {
      int xi = px - x0;
      int yi = py - y0;

      if (map_valid)
        (*ml) = map->cells[MAP_INDEX(map, px, py)].label;

      *mi = px;
      *mj = py;

      //printf("Working: %f [%i %i %i]\n",sqrtf(xi * xi + yi * yi),*mi,*mj,*ml);

      return sqrtf(xi * xi + yi * yi) * map->scale;
    }
    d = map->cells[MAP_INDEX(map, px, py)].occ_dist;
    t += ((d * step_coeff) > map->scale ? (d * step_coeff) : map->scale);
  }

  return max_range;
}

double map_calc_pos(map_t *map, double ox, double oy, double oa, double max_range, int* mi, int* mj, int* ml)
{
  // Bresenham raytracing
  int x0, x1, y0, y1;
  int xstep, ystep;
  int x, y;
  char steep;
  int tmp;
  int deltax, deltay, error, deltaerr;

  x0 = MAP_GXWX(map, ox);
  y0 = MAP_GYWY(map, oy);

  x1 = MAP_GXWX(map, ox + max_range * cos(oa));
  y1 = MAP_GYWY(map, oy + max_range * sin(oa));

  (*ml) = -1;

  if (abs(y1 - y0) > abs(x1 - x0))
    steep = 1;
  else
    steep = 0;

  if (steep)
  {
    tmp = x0;
    x0 = y0;
    y0 = tmp;

    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }

  deltax = abs(x1 - x0);
  deltay = abs(y1 - y0);
  error = 0;
  deltaerr = deltay;

  x = x0;
  y = y0;

  if (x0 < x1)
    xstep = 1;
  else
    xstep = -1;
  if (y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  if (steep)
  {
    if (!MAP_VALID(map, y, x) || map->cells[MAP_INDEX(map, y, x)].occ_state > -1)
    {
      if (MAP_VALID(map, y, x))
        (*ml) = map->cells[MAP_INDEX(map, y, x)].label;

      //Swap both back (so return is correct and distance is same)
      *mi = y;
      *mj = x;

      return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
    }
  }
  else
  {
    if (!MAP_VALID(map, x, y) || map->cells[MAP_INDEX(map, x, y)].occ_state > -1)
    {
      if (MAP_VALID(map, x, y))
        (*ml) = map->cells[MAP_INDEX(map, x, y)].label;

      *mi = x;
      *mj = y;

      return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
    }
  }

  while (x != (x1 + xstep * 1))
  {
    x += xstep;
    error += deltaerr;
    if (2 * error >= deltax)
    {
      y += ystep;
      error -= deltax;
    }

    if (steep)
    {
      if (!MAP_VALID(map, y, x) || map->cells[MAP_INDEX(map, y, x)].occ_state > -1)
      {
        if (MAP_VALID(map, y, x))
          (*ml) = map->cells[MAP_INDEX(map, y, x)].label;

        *mi = y;
        *mj = x;

        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
      }
    }
    else
    {
      if (!MAP_VALID(map, x, y) || map->cells[MAP_INDEX(map, x, y)].occ_state > -1)
      {
        if (MAP_VALID(map, x, y))
          (*ml) = map->cells[MAP_INDEX(map, x, y)].label;

        *mi = x;
        *mj = y;

        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
      }
    }
  }
  return max_range;
}
