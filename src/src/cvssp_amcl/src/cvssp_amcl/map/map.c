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
 * Desc: Global map (grid-based)
 * Author: Andrew Howard
 * Date: 6 Feb 2003
 * CVS: $Id: map.c 1713 2003-08-23 04:03:43Z inspectorg $
 **************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "map.h"

// Create a new map
map_t *map_alloc(void)
{
  map_t *map;

  map = (map_t*)malloc(sizeof(map_t));

  // Assume we start at (0, 0)
  map->origin_x = 0;
  map->origin_y = 0;

  // Make the size odd
  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;

  // Allocate storage for main map
  map->cells = (map_cell_t*) NULL;

  return map;
}

// Destroy a map
void map_free(map_t *map)
{
  free(map->cells);
  free(map);
  return;
}

// Get the cell at the given point
map_cell_t *map_get_cell(map_t *map, double ox, double oy, double oa)
{
  int i, j;
  map_cell_t *cell;

  i = MAP_GXWX(map, ox);
  j = MAP_GYWY(map, oy);

  if (!MAP_VALID(map, i, j))
    return NULL;

  cell = map->cells + MAP_INDEX(map, i, j);
  return cell;
}

int has_collision(map_t *map, int x, int y, double* distToDoor)
{
  if (!MAP_VALID(map, x, y))
  {
    (*distToDoor) = map->max_label_dist;
    return 1;
  }
  else if (map->cells[MAP_INDEX(map, x, y)].label == WALL)
  {
    (*distToDoor) = map->cells[MAP_INDEX(map, x, y)].label_dist[1];
    return 1;
  }
  else if (map->cells[MAP_INDEX(map, x, y)].label == DOOR)
  {
    (*distToDoor) = map->cells[MAP_INDEX(map, x, y)].label_dist[0];
    return 1;
  }
  // else if (map->cells[MAP_INDEX(map, x, y)].label == METAL)
  // {
  //   (*distToDoor) = map->cells[MAP_INDEX(map, x, y)].label_dist[4];
  //   return 1;
  // }
  return 0;
}

int map_check_collision(map_t *map, double ox, double oy, double oa, double dx, double dy, double da,
                        double* distToDoor)
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

  x1 = MAP_GXWX(map, ox + dx);
  y1 = MAP_GYWY(map, oy + dy);

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
    if (has_collision(map, y, x, distToDoor)) return 0;
  }
  else
  {
    if (has_collision(map, x, y, distToDoor)) return 0;
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
      if (has_collision(map, y, x, distToDoor)) return 0;
    }
    else
    {
      if (has_collision(map, x, y, distToDoor)) return 0;
    }
  }

  if (steep)
  {
    if (MAP_VALID(map, y, x))
      (*distToDoor) = map->cells[MAP_INDEX(map, y, x)].label_dist[1];
    else
      (*distToDoor) = map->max_label_dist;
  }
  else
  {
    if (MAP_INDEX(map, x, y))
      (*distToDoor) = map->cells[MAP_INDEX(map, x, y)].label_dist[1];
    else
      (*distToDoor) = map->max_label_dist;
  }

    if (steep)
  {
    if (MAP_VALID(map, y, x))
      (*distToDoor) = map->cells[MAP_INDEX(map, y, x)].label_dist[0];
    else
      (*distToDoor) = map->max_label_dist;
  }
  else
  {
    if (MAP_INDEX(map, x, y))
      (*distToDoor) = map->cells[MAP_INDEX(map, x, y)].label_dist[0];
    else
      (*distToDoor) = map->max_label_dist;
  }
  //     if (steep)
  // {
  //   if (MAP_VALID(map, y, x))
  //     (*distToDoor) = map->cells[MAP_INDEX(map, y, x)].label_dist[4];
  //   else
  //     (*distToDoor) = map->max_label_dist;
  // }
  // else
  // {
  //   if (MAP_INDEX(map, x, y))
  //     (*distToDoor) = map->cells[MAP_INDEX(map, x, y)].label_dist[4];
  //   else
  //     (*distToDoor) = map->max_label_dist;
  // }

  return 1;
}

