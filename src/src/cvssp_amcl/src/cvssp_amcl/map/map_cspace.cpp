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

#include <queue>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "map.h"
#include <ros/console.h>

int LabelToIndex(int label)
{
  int idx = -1;
  if (label == WALL)
    idx = 0;
  if (label == DOOR)
    idx = 1;
  if (label == WINDOW)
    idx = 2;
  if (label == PAINTED_WOOD)
    idx = 3;
  if (label == METAL)
    idx = 4;
  if (label == PLASTIC)
    idx = 5;
  // switch(label)
  // {
  //   case BRICK:
  //     idx = 0;
  //     break;
  //   case LAMINATED_WOOD:
  //     idx = 1;
  //     break;
  //   case PAINTED_WOOD:
  //     idx = 2;
  //     break;
  //   case GLASS:
  //     idx = 3;
  //     break;
  //   case METAL:
  //     idx = 4;
  //     break;
  //   case PLASTIC:
  //     idx = 5;
  //     break;
  //   default:
  //     idx = -1;
  // }
  return idx;
}

class CellData
{
public:
  map_t* map_;
  int type_;
  unsigned int i_, j_;
  unsigned int src_i_, src_j_;
};

class CachedDistanceMap
{
public:
  CachedDistanceMap(double scale, double max_dist) :
      distances_(NULL), scale_(scale), max_dist_(max_dist)
  {
    cell_radius_ = max_dist / scale;
    distances_ = new double *[cell_radius_ + 2];
    for (int i = 0; i <= cell_radius_ + 1; i++)
    {
      distances_[i] = new double[cell_radius_ + 2];
      for (int j = 0; j <= cell_radius_ + 1; j++)
      {
        distances_[i][j] = sqrt(i * i + j * j);
      }
    }
  }
  ~CachedDistanceMap()
  {
    if (distances_)
    {
      for (int i = 0; i <= cell_radius_ + 1; i++)
        delete[] distances_[i];
      delete[] distances_;
    }
  }
  double** distances_;
  double scale_;
  double max_dist_;
  int cell_radius_;
};

bool operator<(const CellData& a, const CellData& b)
{
  // if (a.type_ == WALL)
  //   return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].label_dist[0]
  //       > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].label_dist[0];
  // else if (a.type_ == DOOR)
  //   return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].label_dist[1]
  //       > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].label_dist[1];
  // else if (a.type_ == WINDOW)
  //   return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].label_dist[2]
  //       > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].label_dist[2];
  // else
  //   return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist;

  if (a.type_ == WALL)
    return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].label_dist[0]
        > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].label_dist[0];

  else if (a.type_ == DOOR)
    return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].label_dist[1]
        > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].label_dist[1];

  else if (a.type_ == PAINTED_WOOD)
    return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].label_dist[2]
        > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].label_dist[2];

  else if (a.type_ == WINDOW)
    return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].label_dist[3]
        > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].label_dist[3];

  else if (a.type_ == METAL)
    return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].label_dist[4]
        > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].label_dist[4];
        
  else if (a.type_ == PLASTIC)
    return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].label_dist[5]
        > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].label_dist[5];

  else
    return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist 
        > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist;
}

CachedDistanceMap*
get_distance_map(double scale, double max_dist)
{
  static CachedDistanceMap* cdm = NULL;

  if (!cdm || (cdm->scale_ != scale) || (cdm->max_dist_ != max_dist))
  {
    if (cdm)
      delete cdm;
    cdm = new CachedDistanceMap(scale, max_dist);
  }

  return cdm;
}

void enqueue(map_t* map, unsigned int i, unsigned int j, unsigned int src_i, unsigned int src_j, int type,
             std::priority_queue<CellData>& Q, CachedDistanceMap* cdm, unsigned char* marked)
{
  if (marked[MAP_INDEX(map, i, j)])
    return;

  unsigned int di = abs((int)(i - src_i));
  // unsigned int di = (i - src_i) < 0 ? -di : di; 
  // unsigned int dj = (j - src_j) < 0 ? -dj : dj; 
  unsigned int dj = abs((int)(j - src_j));
  double distance = cdm->distances_[di][dj];

  if (distance > cdm->cell_radius_)
    return;

  if (type == -1)
    map->cells[MAP_INDEX(map, i, j)].occ_dist = distance * map->scale;
  else
    map->cells[MAP_INDEX(map, i, j)].label_dist[LabelToIndex(type)] = distance * map->scale;

  CellData cell;
  cell.type_ = type;
  cell.map_ = map;
  cell.i_ = i;
  cell.j_ = j;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;

  Q.push(cell);

  marked[MAP_INDEX(map, i, j)] = 1;
}

// Update the cspace distance values
void map_update_cspace(map_t *map, int label, double max_dist)
{
  // ROS_INFO("Wuz gud fam");
  
  if (label == -1)
    ROS_INFO("Updating Depth Distance Map: %f", max_dist);
  else
    ROS_INFO("Updating Label %i Distance Map: %f", label, max_dist);

  int idx = LabelToIndex(label);

  unsigned char* marked;
  std::priority_queue<CellData> Q;

  marked = new unsigned char[map->size_x * map->size_y];
  memset(marked, 0, sizeof(unsigned char) * map->size_x * map->size_y);

  if (label == -1){
    map->max_occ_dist = max_dist;
    map->occ_ctr = 0;
  }
  else
  {
    map->max_label_dist = max_dist;
    map->label_ctr[idx] = 0;
  }

  CachedDistanceMap* cdm = get_distance_map(map->scale, max_dist);
  
  // Enqueue all the obstacle cells
  CellData cell;
  cell.type_ = label;
  cell.map_ = map;
  for (int i = 0; i < map->size_x; i++)
  {
    cell.src_i_ = cell.i_ = i;
    for (int j = 0; j < map->size_y; j++)
    {
      bool occupied = false;
      if (label == -1)
        occupied = (map->cells[MAP_INDEX(map, i, j)].occ_state == +1);
      else
        occupied = (map->cells[MAP_INDEX(map, i, j)].label == label);

      if (occupied)
      {
        if (label == -1){
          map->cells[MAP_INDEX(map, i, j)].occ_dist = 0.0;
          map->occ_ctr++;
        }else
        {
          map->cells[MAP_INDEX(map, i, j)].label_dist[idx] = 0.0;
          map->label_ctr[idx]++;
        }
        cell.src_j_ = cell.j_ = j;
        marked[MAP_INDEX(map, i, j)] = 1;
        Q.push(cell);
      }
      else
      {
        if (label == -1)
          map->cells[MAP_INDEX(map, i, j)].occ_dist = max_dist;
        else
          map->cells[MAP_INDEX(map, i, j)].label_dist[idx] = max_dist;
      }
    }
  }
  
  while (!Q.empty())
  {
    CellData current_cell = Q.top(); 
    
    if (current_cell.i_ > 0)
      enqueue(map, current_cell.i_ - 1, current_cell.j_, current_cell.src_i_, current_cell.src_j_, label, Q, cdm,
              marked);
    if (current_cell.j_ > 0)
      enqueue(map, current_cell.i_, current_cell.j_ - 1, current_cell.src_i_, current_cell.src_j_, label, Q, cdm,
              marked);
    if ((int)current_cell.i_ < map->size_x - 1)
      enqueue(map, current_cell.i_ + 1, current_cell.j_, current_cell.src_i_, current_cell.src_j_, label, Q, cdm,
              marked);
    if ((int)current_cell.j_ < map->size_y - 1)
      enqueue(map, current_cell.i_, current_cell.j_ + 1, current_cell.src_i_, current_cell.src_j_, label, Q, cdm,
              marked);

    Q.pop();
  }
  
  
  
  if (idx == -1){
    ROS_INFO("Got %i occupied Cells", map->occ_ctr);
  }else{
    ROS_INFO("Got %i of label %i", map->label_ctr[idx], label);
  }

  
  delete[] marked;

  
}

void map_update_label_space(map_t *map, double max_label_dist)
{
  /* map_update_label(map, max_label_dist, WALL);
   map_update_label(map, max_label_dist, DOOR);
   map_update_label(map, max_label_dist, WINDOW);*/

  
  
  // map_update_cspace(map, WALL, max_label_dist);
  // map_update_cspace(map, DOOR, max_label_dist);
  // map_update_cspace(map, WINDOW, max_label_dist);
  map_update_cspace(map, WALL, max_label_dist);
  map_update_cspace(map, DOOR, max_label_dist);
  map_update_cspace(map, WINDOW, max_label_dist);
  map_update_cspace(map, PAINTED_WOOD, max_label_dist);
  map_update_cspace(map, METAL, max_label_dist);
  map_update_cspace(map, PLASTIC, max_label_dist);
}
void map_update_label(map_t *map, double max_label_dist, int label)
{
  ROS_INFO("Updating Label %i Distance Map: %f", label, max_label_dist);
  int i, j;
  int ni, nj;
  int s;
  double d;
  map_cell_t *cell, *ncell;

  //OSCAR: TODO: This is horrible, do this in a better way
  int idx(LabelToIndex(label));
  // if (label == WALL)
  //   idx = 0;
  // if (label == DOOR)
  //   idx = 1;
  // if (label == WINDOW)
  //   idx = 2;


  map->max_label_dist = max_label_dist;
  s = (int)ceil(map->max_label_dist / map->scale);

  // Reset the distance values
  for (j = 0; j < map->size_y; j++)
  {
    for (i = 0; i < map->size_x; i++)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      cell->label_dist[idx] = map->max_label_dist;
    }
  }

  // Find all the occupied cells and update their neighbours
  for (j = 0; j < map->size_y; j++)
  {
    for (i = 0; i < map->size_x; i++)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      if (cell->label != label)
        continue;

      cell->label_dist[idx] = 0;

      // Update adjacent cells
      for (nj = -s; nj <= +s; nj++)
      {
        for (ni = -s; ni <= +s; ni++)
        {
          if (!MAP_VALID(map, i + ni, j + nj))
            continue;

          ncell = map->cells + MAP_INDEX(map, i + ni, j + nj);
          d = map->scale * sqrt(ni * ni + nj * nj);

          if (d < ncell->label_dist[idx])
            ncell->label_dist[idx] = d;
        }
      }
    }
  }

  return;
}

#if 0
// TODO: replace this with a more efficient implementation.  Not crucial,
// because we only do it once, at startup.
void map_update_cspace(map_t *map, double max_occ_dist)
{
  int i, j;
  int ni, nj;
  int s;
  double d;
  map_cell_t *cell, *ncell;

  map->max_occ_dist = max_occ_dist;
  s = (int) ceil(map->max_occ_dist / map->scale);

  // Reset the distance values
  for (j = 0; j < map->size_y; j++)
  {
    for (i = 0; i < map->size_x; i++)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      cell->occ_dist = map->max_occ_dist;
    }
  }

  // Find all the occupied cells and update their neighbours
  for (j = 0; j < map->size_y; j++)
  {
    for (i = 0; i < map->size_x; i++)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      if (cell->occ_state != +1)
      continue;

      cell->occ_dist = 0;

      // Update adjacent cells
      for (nj = -s; nj <= +s; nj++)
      {
        for (ni = -s; ni <= +s; ni++)
        {
          if (!MAP_VALID(map, i + ni, j + nj))
          continue;

          ncell = map->cells + MAP_INDEX(map, i + ni, j + nj);
          d = map->scale * sqrt(ni * ni + nj * nj);

          if (d < ncell->occ_dist)
          ncell->occ_dist = d;
        }
      }
    }
  }

  return;
}

#endif
