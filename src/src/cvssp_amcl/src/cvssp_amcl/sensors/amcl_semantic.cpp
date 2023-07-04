/*
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
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL Semantic routines
// Author: Oscar Mendez
// Date: 1 May 2017
//
///////////////////////////////////////////////////////////////////////////
#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <algorithm>
#include <string>
#include "raman_lut.h"
#include "amcl_semantic.h"

//REMOVE AFTER DEBUGGING
#include "wasserstein.h"

using namespace cvssp_amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLSemantic::AMCLSemantic(size_t max_beams, map_t* map, bool use_depth, std::string const& sim_func) :
    AMCLSensor()
{
  this->sim_func_type = hashit(sim_func);

  // if(this->sim_func_type != MODIFIED_EUCLIDEAN || this->sim_func_type != SAM)
  //   this->ramanUtil = raman_lut("/home/ct00659/Downloads/raman_alt_1/spectra_512.txt");
  // else
    this->ramanUtil = raman_lut("/home/ct00659/Downloads/raman_alt_1/spectra_sum_norm_256_blrm.txt");
    
  this->time = 0.0;

  this->use_depth = use_depth;

  this->max_beams = max_beams;
  this->map = map;

  return;
}

AMCLSemantic::~AMCLSemantic()
{
}

sim_func_t AMCLSemantic::hashit (std::string const& sim_func) 
{
  if (sim_func == "wasserstein") return WASSERSTEIN;
  if (sim_func == "kl-div") return KL_DIVERGENCE;
  if (sim_func == "SLK") return SPECTRAL_LINEAR_KERNEL;
  if (sim_func == "mod") return MODIFIED_EUCLIDEAN;
  if (sim_func == "peak") return PEAK_EUCLIDEAN;
  if (sim_func == "SAM") return SAM;

  return WASSERSTEIN;
}

void AMCLSemantic::SetModelBeam(double z_hit, double z_short, double z_max, double z_rand, double sigma_hit,
                                double lambda_short, double chi_outlier, double fudge_factor)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD;

  this->z_hit = z_hit;
  this->z_short = z_short;
  this->z_max = z_max;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->lambda_short = lambda_short;
  this->chi_outlier = chi_outlier;
  this->fudge_factor = fudge_factor;

  ROS_INFO("z_h: %f", this->z_hit);
  ROS_INFO("z_s: %f", this->z_short);
  ROS_INFO("z_m: %f", this->z_max);
  ROS_INFO("z_r: %f", this->z_rand);
  ROS_INFO("s_h: %f", this->sigma_hit);
  ROS_INFO("l_s: %f", this->lambda_short);
  ROS_INFO("c_o: %f", this->chi_outlier );
  ROS_INFO("f_f: %f", this->fudge_factor );
}

////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLSemantic::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  if (this->max_beams < 2)
    return false;

  // Apply the laser sensor model
  if (this->model_type == LASER_MODEL_BEAM)
    pf_update_sensor(pf, (pf_sensor_model_fn_t)BeamModel, data);
  else if (this->model_type == LASER_MODEL_LIKELIHOOD_FIELD)
    pf_update_sensor(pf, (pf_sensor_model_fn_t)LikelihoodFieldModel, data);
  else if (this->model_type == LASER_MODEL_LIKELIHOOD_FIELD_PROB)
    pf_update_sensor(pf, (pf_sensor_model_fn_t)LikelihoodFieldModelProb, data);
  else
    pf_update_sensor(pf, (pf_sensor_model_fn_t)BeamModel, data);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
double AMCLSemantic::BeamModel(AMCLSemanticData *data, pf_sample_set_t* set)
{
  //TODO: BEAM SKIPPING?

  AMCLSemantic *self;
  int i, j, step;
  double z;
  double pz, pz_i;
  double pl, pl_i;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  int map_label, obs_label;
  int num_steps;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;

  self = (AMCLSemantic*)data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    pz = 0.0;
    pl = 0.0;

    step = std::max((data->range_count - 1) / (self->max_beams - 1), 1);
    num_steps = (data->range_count / step);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];
      obs_label = data->labels[i];

      //OSCAR: Check for NaN (Should never happen because NANS are mapped to MAX in SemanticReceived)
      if (obs_range != obs_range)
      {
        ROS_WARN("NaN: Ignoring Range...");
        continue;
      }

      pz_i = 0.0;

      // Compute the range according to the map
      map_range = map_calc_range(self->map, pose.v[0], pose.v[1], pose.v[2] + obs_bearing, data->range_max, &map_label);

      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz_i += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if (z < 0)
        pz_i += self->z_short * self->lambda_short * exp(-self->lambda_short * obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      if (obs_range == data->range_max)
      {
        //OSCAR: Does this ever happen?
        //ROS_INFO("Max Range!"); YES. ALREADY CHECKED.
        pz_i += self->z_max * 1.0;
      }

      // Part 4: Random measurements
      if (obs_range < data->range_max)
        pz_i += self->z_rand * 1.0 / data->range_max;

      // TODO: outlier rejection for short readings

      pl_i = 0.0;
      //OSCAR: TODO: Labels
      // if (obs_label == map_label)
      // {
      //   pl_i += 1.0 / (double)num_steps;
      //   //ROS_INFO("Labels Match: %i (%f)", obs_label, pl_i);
      // }
      // else
      // {

      // }
      //Do this instead for spectra
      switch(self->sim_func_type)
      {
      case WASSERSTEIN:
        pl_i = self->ramanUtil.wasserstein_likelihood2(self->ramanUtil.spectra[obs_label], self->ramanUtil.spectra[map_label]);
        break;
      case SPECTRAL_LINEAR_KERNEL:
        pl_i = self->ramanUtil.spectralLinearKernel_likelihood(self->ramanUtil.spectra[obs_label], self->ramanUtil.spectra[map_label]);
        break;
      case KL_DIVERGENCE:
        pl_i = self->ramanUtil.kl_div(self->ramanUtil.spectra[obs_label], self->ramanUtil.spectra[map_label]);
        break;
      case MODIFIED_EUCLIDEAN:
        pl_i = self->ramanUtil.mod_euclidian(self->ramanUtil.spectra[obs_label], self->ramanUtil.spectra[map_label]);
        break;
      case PEAK_EUCLIDEAN:
        pl_i = self->ramanUtil.peak_euclidian(self->ramanUtil.spectra[obs_label], self->ramanUtil.spectra[map_label]);
        break;
      case SAM:
        pl_i = self->ramanUtil.calculate_sam(self->ramanUtil.spectra[obs_label], self->ramanUtil.spectra[map_label]);
        break;
      default:
        pl_i = self->ramanUtil.wasserstein_likelihood2(self->ramanUtil.spectra[obs_label], self->ramanUtil.spectra[map_label]);
        break;
      }
      // pl_i = self->ramanUtil.wasserstein_likilihood2(self->ramanUtil.spectra[obs_label], self->ramanUtil.spectra[map_label]);


      assert(pz_i <= 1.0);
      assert(pz_i >= 0.0);
//      assert(pl_i <= 1.0);
//      assert(pl_i >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      pz += (pz_i * pz_i * pz_i);
      pl += (pl_i * pl_i * pl_i);
    }

    p = ((self->wz * pz) + (self->wl * pl)) / (self->wz + self->wl);

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return (total_weight);
}

//UNUSED
void AMCLSemantic::SetModelLikelihoodField(double z_hit, double z_rand, double s_rand, double sigma_hit, double max_occ_dist,
                                           double max_label_dist, double lambda_short, double fudge_factor)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->s_rand = s_rand;
  this->z_short = 0.0; //This makes things worse, leave at zero
  this->sigma_hit = sigma_hit;
  this->lambda_short = lambda_short;
  this->fudge_factor = fudge_factor;

  ROS_INFO("z_h: %f", this->z_hit);
  ROS_INFO("z_r: %f", this->z_rand);
  ROS_INFO("s_h: %f", this->sigma_hit);
  ROS_INFO("l_s: %f", this->lambda_short);
  ROS_INFO("f_f: %f", this->fudge_factor );

  map_update_cspace(this->map, -1, max_occ_dist);
  map_update_label_space(this->map, max_label_dist);
  //assert((this->map->label_ctr[0]+this->map->label_ctr[1]+this->map->label_ctr[2])==this->map->occ_ctr);
}

void AMCLSemantic::SetModelLikelihoodFieldProb(double z_hit, double z_rand, double s_rand, double sigma_hit, double max_occ_dist,
                                               double max_label_dist, bool do_beamskip, double beam_skip_distance,
                                               double beam_skip_threshold, double beam_skip_error_threshold)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->s_rand = s_rand;
  this->sigma_hit = sigma_hit;
  this->do_beamskip = do_beamskip;
  this->beam_skip_distance = beam_skip_distance;
  this->beam_skip_threshold = beam_skip_threshold;
  this->beam_skip_error_threshold = beam_skip_error_threshold;
  map_update_cspace(this->map, -1, max_occ_dist);
  map_update_label_space(this->map, max_label_dist);
  //assert((this->map->label_ctr[0]+this->map->label_ctr[1]+this->map->label_ctr[2])==this->map->occ_ctr);
}

// double AMCLSemantic::LikelihoodFieldModel(AMCLSemanticData *data, pf_sample_set_t* set)
// {
//   AMCLSemantic *self;

//   self = (AMCLSemantic*)data->sensor;

//   for(int i(0); i < MATERIALS; ++i)
//   {
//     for(int j(0); j < MATERIALS; ++j)
//     {
//       self->wasserstein_likilihoods[i][j] = self->ramanUtil.wasserstein_likilihood(self->ramanUtil.spectra[i], self->ramanUtil.spectra[j]);
//     }
//   }

//   double total_weight = 0.0;

//   //Some Stats
//   int ctr = 0;
//   // if (label == WALL)
//     // idx = 0;
//   // if (label == DOOR)
//     // idx = 1;
//   // if (label == WINDOW)
//     // idx = 2;
//   // if (label == PAINTED_WOOD)
//     // idx = 3;
//   // if (label == METAL)
//     // idx = 4;
//   // if (label == PLASTIC)
//     // idx = 5;
//   // Add in the new materials 
//   double hits[6][2]; //Hits -> Matches, Misses
//   hits[0][0] = 0.0; // WALL
//   hits[0][1] = 0.0;
//   hits[1][0] = 0.0; // DOOR
//   hits[1][1] = 0.0;
//   hits[2][0] = 0.0; // WINDOW
//   hits[2][1] = 0.0;
//   hits[3][0] = 0.0; // PAINTED WOOD
//   hits[3][1] = 0.0;
//   hits[4][0] = 0.0; // METAL
//   hits[4][1] = 0.0;
//   hits[5][0] = 0.0; // PLASTIC
//   hits[5][1] = 0.0;

//   // Compute the sample weights
//  double lblTotal=(self->map->label_ctr[0] + 
//                   self->map->label_ctr[1] +
//                   self->map->label_ctr[2] +
//                   self->map->label_ctr[3] + 
//                   self->map->label_ctr[4] +
//                   self->map->label_ctr[5]);

//  double lblWeight [6];
//  lblWeight [0] = self->map->label_ctr[0] / lblTotal;
//  lblWeight [1] = self->map->label_ctr[1] / lblTotal;
//  lblWeight [2] = self->map->label_ctr[2] / lblTotal;
//  lblWeight [3] = self->map->label_ctr[4] / lblTotal;
//  lblWeight [4] = self->map->label_ctr[5] / lblTotal;
//  lblWeight [5] = self->map->label_ctr[6] / lblTotal;

//  double lblSigma [6];
//  lblSigma [0] = std::sqrt(1.0/(lblWeight[0]*self->fudge_factor));//0.5*std::sqrt(lblWeight[0]/2.0);
//  lblSigma [1] = std::sqrt(1.0/(lblWeight[1]*self->fudge_factor));//0.5*std::sqrt(lblWeight[1]/2.0);
//  lblSigma [2] = std::sqrt(1.0/(lblWeight[2]*self->fudge_factor));//0.5*std::sqrt(lblWeight[2]/2.0);
//  lblSigma [3] = std::sqrt(1.0/(lblWeight[3]*self->fudge_factor));//0.5*std::sqrt(lblWeight[2]/2.0);
//  lblSigma [4] = std::sqrt(1.0/(lblWeight[4]*self->fudge_factor));//0.5*std::sqrt(lblWeight[2]/2.0);
//  lblSigma [5] = std::sqrt(1.0/(lblWeight[5]*self->fudge_factor));//0.5*std::sqrt(lblWeight[2]/2.0);

//  double lblDenom [6];
//  lblDenom [0] = 2.0*std::pow(lblSigma[0],2.0);
//  lblDenom [1] = 2.0*std::pow(lblSigma[1],2.0);
//  lblDenom [2] = 2.0*std::pow(lblSigma[2],2.0);
//  lblDenom [3] = 2.0*std::pow(lblSigma[3],2.0);
//  lblDenom [4] = 2.0*std::pow(lblSigma[4],2.0);
//  lblDenom [5] = 2.0*std::pow(lblSigma[5],2.0);

//  double allNorm = ( 1.0 / std::sqrt(2.0*M_PI*std::pow(lblSigma[1],2.0)));

//  double lblNorm [6];
//  lblNorm [0] = 1.0; // (1.0 / std::sqrt( 2.0*M_PI*std::pow(lblSigma[0],2.0)))/allNorm;                               //1.0; //
//  lblNorm [1] = 1.0; // (1.0 / std::sqrt( 2.0*M_PI*std::pow(lblSigma[1],2.0)))/allNorm; //This is basically 1.        //1.0; //
//  lblNorm [2] = 1.0; // (1.0 / std::sqrt( 2.0*M_PI*std::pow(lblSigma[2],2.0)))/allNorm;                               //1.0; //
//  lblNorm [3] = 1.0; // (1.0 / std::sqrt( 2.0*M_PI*std::pow(lblSigma[2],2.0)))/allNorm;                               //1.0; //
//  lblNorm [4] = 1.0; // (1.0 / std::sqrt( 2.0*M_PI*std::pow(lblSigma[2],2.0)))/allNorm;                               //1.0; //
//  lblNorm [5] = 1.0; // (1.0 / std::sqrt( 2.0*M_PI*std::pow(lblSigma[2],2.0)))/allNorm;                               //1.0; //

// //  ROS_INFO_ONCE("Label Total: %f", lblTotal);
// //  ROS_INFO_ONCE("Fudge Factor: %f", lblTotal);
// //  ROS_INFO_ONCE("Wall Weight: %i (%f)", self->map->label_ctr[0], lblWeight[0]);
// //  ROS_INFO_ONCE("Door Weight: %i (%f)", self->map->label_ctr[1], lblWeight[1]);
// //  ROS_INFO_ONCE("Wdow Weight: %i (%f)", self->map->label_ctr[2], lblWeight[2]);
// //  ROS_INFO_ONCE("PAINTED_WOOD Weight: %i (%f)", self->map->label_ctr[3], lblWeight[3]);
// //  ROS_INFO_ONCE("METAL Weight: %i (%f)", self->map->label_ctr[4], lblWeight[4]);
// //  ROS_INFO_ONCE("PLASTIC Weight: %i (%f)", self->map->label_ctr[5], lblWeight[5]);

// //  ROS_INFO_ONCE("Wall Sigma: %f", lblSigma[0]);
// //  ROS_INFO_ONCE("Door Sigma: %f", lblSigma[1]);
// //  ROS_INFO_ONCE("Wdow Sigma: %f", lblSigma[2]);
// //  ROS_INFO_ONCE("PAINTED_WOOD Sigma: %f", lblSigma[3]);
// //  ROS_INFO_ONCE("METAL Sigma: %f", lblSigma[4]);
// //  ROS_INFO_ONCE("PLASTIC Sigma: %f", lblSigma[5]);

// //  ROS_INFO_ONCE("Wall Denom: %f", lblDenom[0]);
// //  ROS_INFO_ONCE("Door Denom: %f", lblDenom[1]);
// //  ROS_INFO_ONCE("Wdow Denom: %f", lblDenom[2]);
// //  ROS_INFO_ONCE("PAINTED_WOOD Denom: %f", lblDenom[3]);
// //  ROS_INFO_ONCE("METAL Denom: %f", lblDenom[4]);
// //  ROS_INFO_ONCE("PLASTIC Denom: %f", lblDenom[5]);

// //  ROS_INFO_ONCE("Wall Norm: %f", lblNorm[0]);
// //  ROS_INFO_ONCE("Door Norm: %f", lblNorm[1]);
// //  ROS_INFO_ONCE("Wdow Norm: %f", lblNorm[2]);
// //  ROS_INFO_ONCE("PAINTED_WOOD Norm: %f", lblNorm[3]);
// //  ROS_INFO_ONCE("METAL Norm: %f", lblNorm[4]);
// //  ROS_INFO_ONCE("PLASTIC Norm: %f", lblNorm[5]);


// #pragma omp parallel for num_threads(32) //schedule(dynamic, set->sample_count/16)
//   for (int j = 0; j < set->sample_count; j++)
//   {
//     int step;
//     double z, pz;
//     double l, pl;
//     double d, pd;
//     double p;
//     double obs_range, obs_bearing, obs_label;
//     int map_label;

//     pf_sample_t *sample;
//     pf_vector_t pose;
//     pf_vector_t hit;

//     sample = set->samples + j;
//     pose = sample->pose;

//     if (fabs(self->wz) < 0.0000001 && fabs(self->wl) < 0.0000001)
//     {
//       ROS_DEBUG("No Cost.");
//       sample->weight *= 1.0;
//       total_weight += sample->weight;
//       continue;
//     }

//     // Take account of the laser pose relative to the robot
//     pose = pf_vector_coord_add(self->laser_pose, pose);

//     pz = 1.0;
//     pl = 1.0;

//     // Pre-compute a couple of things
//     double z_hit_norm = std::sqrt(2 * M_PI * self->sigma_hit * self->sigma_hit);
//     double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
//     double z_rand_mult = 1.0 / data->range_max;
//     double d_rand_mult = 1.0 / self->ramanUtil.spectra[0].size();

//     step = (data->range_count - 1) / (self->max_beams - 1);

//     // Step size must be at least 1
//     if (step < 1)
//       step = 1;

//     for (int i = 0; i < data->range_count; i += step)
//     {
//       int mi, mj, ml = -1;
//       if (self->use_depth)
//       {
//         obs_range = data->ranges[i][0];

//         // This model ignores max range readings
//         if (obs_range >= data->range_max)
//           continue;

//         // Check for NaN
//         if (obs_range != obs_range)
//           continue;

//         // Compute the endpoint of the beam
//         hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
//         hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

//         // Convert to map grid coords.
//         mi = MAP_GXWX(self->map, hit.v[0]);
//         mj = MAP_GYWY(self->map, hit.v[1]);

//       }
//       else
//       {
//         obs_range = calc_range(self->map, pose.v[0], pose.v[1], pose.v[2] + obs_bearing, data->range_max, &mi, &mj,
//                                &ml);
//       }
//       obs_bearing = data->ranges[i][1];
//       obs_label = data->labels[i];

//       //OSCAR: This is messy, make it better
//       int idx = LabelToIndex(obs_label);
//       double l_hit_weight = 1.0;
//       //double l_hit_norm = 1.0;
//       if (idx != -1){
//         l_hit_weight = lblWeight[idx];
//        /* l_hit_denom = lblDenom[idx];
//         l_hit_norm = lblNorm[idx];*/
//       }

//       double pz_i = 0.0;
//       double pl_i = 0.0;
//       double pd_i = 0.0;

//       // Part 1: Get distance from the hit to closest obstacle.
//       // Off-map penalized as max distance
//       // Same for label (not in map)
//       bool mapValid = MAP_VALID(self->map, mi, mj);

//       if (!mapValid)
//         z = self->map->max_occ_dist;
//       else
//       {
//         z = self->map->cells[MAP_INDEX(self->map, mi, mj)].occ_dist;
//       }

//       //ROS_INFO("I: %i, J: %i, OZ: %f, MZ: %f",mi,mj,obs_range,z);
//       double min = 0.0;
//       int min_idx = 0;

//       if (!mapValid || idx == -1) //TODO: Something more clever with the labels (like person is always closer)
//         l = self->map->max_label_dist;
//       else
//       {
//         //Use spectra here
//         // l = self->map->cells[MAP_INDEX(self->map, mi, mj)].label_dist[idx];
//         // search num in inputArray from index 0 to elementCount-1 
//         for(int k = 0; k < 6; k++){
//             if(((self->map->cells[MAP_INDEX(self->map, mi, mj)].label_dist[k]) < min) && 
//                ((self->map->cells[MAP_INDEX(self->map, mi, mj)].label_dist[k]) > 0))
//             {
//                 min = self->map->cells[MAP_INDEX(self->map, mi, mj)].label_dist[k];
//                 min_idx = k;
//             }
//         }

//         // Cheap version here
//         // l = (min * 0.9) + (self->ramanUtil.wasserstein_likilihoods[idx][min_idx]);
//         //Probability version here.
//         l = self->wasserstein_likilihoods[idx][min_idx];
//       }
      
//       // Gaussian model (Depth)
//       // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
//       pz_i += (self->z_hit * z_hit_norm * exp(-(z * z) / z_hit_denom));
//       // Part 2: random measurements
//       pz_i += self->z_rand * z_rand_mult;

//       //Spectra likelihood 
//       pd_i += self->z_hit * exp(-(min * min)) / z_rand_mult;
//       pd_i += self->d_rand * d_rand_mult;
      
//       // Gaussian model (Label)
//       // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
//       if(idx != -1 ) pl_i += (self->z_hit * exp(-(l * l) / lblDenom[idx]) * lblNorm[idx]);

//       // Part 2: random measurements
//       pl_i += self->z_rand * z_rand_mult;

//       // Part 2: short reading from unexpected label (therefore, obstacle)
//       if (idx == -1)
//         pl_i += self->z_short * self->lambda_short * exp(-self->lambda_short * l);

//       assert(pz_i <= 1.0);
//       assert(pz_i >= 0.0);

//       // if (pl_i > 1.0)
//       // {
//       //   ROS_INFO("l: %f ", l);
//       //   ROS_INFO("idx: %i ", idx);
//       //   ROS_INFO("pl_i: %f", pl_i);
//       //   ROS_INFO("l_w: %f", l_hit_weight);
//       //   //ROS_INFO("l_n: %f", l_hit_norm);
//       //   ROS_INFO("z_h: %f", self->z_hit);
//       //   ROS_INFO("z_s: %f", self->z_short);
//       //   ROS_INFO("l_s: %f", self->lambda_short);
//       //   ROS_INFO("z_r: %f", self->z_rand);
//       //   ROS_INFO("r_m: %f", z_rand_mult);
//       // }

//       /*if (obs_label == ml)
//        {
//        hits[idx][0] += l;
//        }
//        else
//        {
//        hits[idx][1] += l;
//        }
//        ctr++;*/

//       assert(pl_i <= 1.0);
//       assert(pl_i >= 0.0);
//       //      p *= pz_i;
//       // here we have an ad-hoc weighting scheme for combining beam probs
//       // works well, though...

//       pz += (pz_i * pz_i * pz_i);
//       pl += (pl_i * pl_i * pl_i);
//       //Prob of spectra from distance likelihood
//       pd += (pd_i * pd_i * pd_i);

//     }
//     //ROS_INFO("P_L: %f", pl);
//     //Original
//     // p = ((self->wz * pz) + (self->wl * pl)) / (self->wz + self->wl);
//     //New
//     p = ((self->wz * pz) + (self->wl * pl) + (self->wd * pd)) / (self->wz + self->wl + self->wd);
//     sample->weight *= p;
// #pragma omp critical
//     {
//       total_weight += sample->weight;
//     }
//   }
//   /*ROS_INFO("Average Distances");
//    ROS_INFO("Wall Hits: %f ", hits[0][0] / (double )ctr);
//    ROS_INFO("Wall Miss: %f ", hits[0][1] / (double )ctr);
//    ROS_INFO("Door Hits: %f ", hits[1][0] / (double )ctr);
//    ROS_INFO("Door Miss: %f ", hits[1][1] / (double )ctr);
//    ROS_INFO("Wind Hits: %f ", hits[2][0] / (double )ctr);
//    ROS_INFO("Wind Miss: %f ", hits[2][1] / (double )ctr);*/
//   return (total_weight);
// }

double AMCLSemantic::LikelihoodFieldModel(AMCLSemanticData *data, pf_sample_set_t* set)
{
  AMCLSemantic *self;

  self = (AMCLSemantic*)data->sensor;

  // std::cout << data->labels[0] << std::endl;
  std::string fileout("/home/ct00659/Downloads/raman_alt_1/label_now.txt");
  std::ofstream outfile;
  outfile.open(fileout, std::ios::trunc);
  outfile << data->labels[0];
  outfile.close();
  //PRE-COMPUTE IF NEEDED
  // for(int i(0); i < MATERIALS; ++i)
  // {
  //   for(int j(0); j < MATERIALS; ++j)
  //   {
  //     self->wasserstein_likilihoods[i][j] = self->ramanUtil.wasserstein_likilihood(self->ramanUtil.spectra[i], self->ramanUtil.spectra[j]);
  //   }
  // }

  // for(int j(0); j < MATERIALS; ++j)
  // {
  //   self->ramanUtil.add_shot_noise(self->ramanUtil.spectra[j], self->noise_spectra[j]);
  // }

  double total_weight = 0.0;

  //Some Stats
  int ctr = 0;

  // Add in the new materials 
  double hits[6][2]; //Hits -> Matches, Misses
  hits[0][0] = 0.0; // WALL
  hits[0][1] = 0.0;
  hits[1][0] = 0.0; // DOOR
  hits[1][1] = 0.0;
  hits[2][0] = 0.0; // WINDOW
  hits[2][1] = 0.0;
  hits[3][0] = 0.0; // PAINTED WOOD
  hits[3][1] = 0.0;
  hits[4][0] = 0.0; // METAL
  hits[4][1] = 0.0;
  hits[5][0] = 0.0; // PLASTIC
  hits[5][1] = 0.0;

  // Compute the sample weights
 double lblTotal=(self->map->label_ctr[0] + 
                  self->map->label_ctr[1] +
                  self->map->label_ctr[2] +
                  self->map->label_ctr[3] + 
                  self->map->label_ctr[4] +
                  self->map->label_ctr[5]);

 double lblWeight [6];
 lblWeight [0] = self->map->label_ctr[0] / lblTotal;
 lblWeight [1] = self->map->label_ctr[1] / lblTotal;
 lblWeight [2] = self->map->label_ctr[2] / lblTotal;
 lblWeight [3] = self->map->label_ctr[4] / lblTotal;
 lblWeight [4] = self->map->label_ctr[5] / lblTotal;
 lblWeight [5] = self->map->label_ctr[6] / lblTotal;

 double lblSigma [6];
 lblSigma [0] = std::sqrt(1.0/(lblWeight[0]*self->fudge_factor));//0.5*std::sqrt(lblWeight[0]/2.0);
 lblSigma [1] = std::sqrt(1.0/(lblWeight[1]*self->fudge_factor));//0.5*std::sqrt(lblWeight[1]/2.0);
 lblSigma [2] = std::sqrt(1.0/(lblWeight[2]*self->fudge_factor));//0.5*std::sqrt(lblWeight[2]/2.0);
 lblSigma [3] = std::sqrt(1.0/(lblWeight[3]*self->fudge_factor));//0.5*std::sqrt(lblWeight[2]/2.0);
 lblSigma [4] = std::sqrt(1.0/(lblWeight[4]*self->fudge_factor));//0.5*std::sqrt(lblWeight[2]/2.0);
 lblSigma [5] = std::sqrt(1.0/(lblWeight[5]*self->fudge_factor));//0.5*std::sqrt(lblWeight[2]/2.0);

 double lblDenom [6];
 lblDenom [0] = 2.0*std::pow(lblSigma[0],2.0);
 lblDenom [1] = 2.0*std::pow(lblSigma[1],2.0);
 lblDenom [2] = 2.0*std::pow(lblSigma[2],2.0);
 lblDenom [3] = 2.0*std::pow(lblSigma[3],2.0);
 lblDenom [4] = 2.0*std::pow(lblSigma[4],2.0);
 lblDenom [5] = 2.0*std::pow(lblSigma[5],2.0);

 double allNorm = ( 1.0 / std::sqrt(2.0*M_PI*std::pow(lblSigma[1],2.0)));

 double lblNorm [6];
 lblNorm [0] = 1.0; // (1.0 / std::sqrt( 2.0*M_PI*std::pow(lblSigma[0],2.0)))/allNorm;                               //1.0; //
 lblNorm [1] = 1.0; // (1.0 / std::sqrt( 2.0*M_PI*std::pow(lblSigma[1],2.0)))/allNorm; //This is basically 1.        //1.0; //
 lblNorm [2] = 1.0; // (1.0 / std::sqrt( 2.0*M_PI*std::pow(lblSigma[2],2.0)))/allNorm;                               //1.0; //
 lblNorm [3] = 1.0; // (1.0 / std::sqrt( 2.0*M_PI*std::pow(lblSigma[2],2.0)))/allNorm;                               //1.0; //
 lblNorm [4] = 1.0; // (1.0 / std::sqrt( 2.0*M_PI*std::pow(lblSigma[2],2.0)))/allNorm;                               //1.0; //
 lblNorm [5] = 1.0; // (1.0 / std::sqrt( 2.0*M_PI*std::pow(lblSigma[2],2.0)))/allNorm;                               //1.0; //


//DEBUGGING STATEMENTS
//  ROS_INFO_ONCE("Label Total: %f", lblTotal);
//  ROS_INFO_ONCE("Fudge Factor: %f", lblTotal);
//  ROS_INFO_ONCE("Wall Weight: %i (%f)", self->map->label_ctr[0], lblWeight[0]);
//  ROS_INFO_ONCE("Door Weight: %i (%f)", self->map->label_ctr[1], lblWeight[1]);
//  ROS_INFO_ONCE("Wdow Weight: %i (%f)", self->map->label_ctr[2], lblWeight[2]);
//  ROS_INFO_ONCE("PAINTED_WOOD Weight: %i (%f)", self->map->label_ctr[3], lblWeight[3]);
//  ROS_INFO_ONCE("METAL Weight: %i (%f)", self->map->label_ctr[4], lblWeight[4]);
//  ROS_INFO_ONCE("PLASTIC Weight: %i (%f)", self->map->label_ctr[5], lblWeight[5]);

//  ROS_INFO_ONCE("Wall Sigma: %f", lblSigma[0]);
//  ROS_INFO_ONCE("Door Sigma: %f", lblSigma[1]);
//  ROS_INFO_ONCE("Wdow Sigma: %f", lblSigma[2]);
//  ROS_INFO_ONCE("PAINTED_WOOD Sigma: %f", lblSigma[3]);
//  ROS_INFO_ONCE("METAL Sigma: %f", lblSigma[4]);
//  ROS_INFO_ONCE("PLASTIC Sigma: %f", lblSigma[5]);

//  ROS_INFO_ONCE("Wall Denom: %f", lblDenom[0]);
//  ROS_INFO_ONCE("Door Denom: %f", lblDenom[1]);
//  ROS_INFO_ONCE("Wdow Denom: %f", lblDenom[2]);
//  ROS_INFO_ONCE("PAINTED_WOOD Denom: %f", lblDenom[3]);
//  ROS_INFO_ONCE("METAL Denom: %f", lblDenom[4]);
//  ROS_INFO_ONCE("PLASTIC Denom: %f", lblDenom[5]);

//  ROS_INFO_ONCE("Wall Norm: %f", lblNorm[0]);
//  ROS_INFO_ONCE("Door Norm: %f", lblNorm[1]);
//  ROS_INFO_ONCE("Wdow Norm: %f", lblNorm[2]);
//  ROS_INFO_ONCE("PAINTED_WOOD Norm: %f", lblNorm[3]);
//  ROS_INFO_ONCE("METAL Norm: %f", lblNorm[4]);
//  ROS_INFO_ONCE("PLASTIC Norm: %f", lblNorm[5]);


#pragma omp parallel for num_threads(20) //schedule(dynamic, set->sample_count/16)
  for (int j = 0; j < set->sample_count; j++)
  {
    int step;
    double z, pz;
    double l, pl;
    double s, ps;
    double p;
    double obs_range, obs_bearing, obs_label;
    int map_label;

    pf_sample_t *sample;
    pf_vector_t pose;
    pf_vector_t hit;

    sample = set->samples + j;
    pose = sample->pose;

    if (fabs(self->wz) < 0.0000001 && fabs(self->wl) < 0.0000001)
    {
      ROS_DEBUG("No Cost.");
      sample->weight *= 1.0;
      total_weight += sample->weight;
      continue;
    }

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    pz = 1.0;
    pl = 1.0;

    // Pre-compute a couple of things
    double z_hit_norm = std::sqrt(2 * M_PI * self->sigma_hit * self->sigma_hit);
    double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
    double z_rand_mult = 1.0 / data->range_max;
    // double s_rand_mult = 1.0 / self->ramanUtil.spectra[0].size();

    step = (data->range_count - 1) / (self->max_beams - 1);

    // Step size must be at least 1
    if (step < 1)
      step = 1;

    for (int i = 0; i < data->range_count; i += step)
    {
      int mi, mj, ml = -1;

      if (self->use_depth)
      {
        obs_range = data->ranges[i][0];

        // This model ignores max range readings
        if (obs_range >= data->range_max)
          continue;

        // Check for NaN
        if (obs_range != obs_range)
          continue;

        // Compute the endpoint of the beam
        hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
        hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

        // Convert to map grid coords.
        mi = MAP_GXWX(self->map, hit.v[0]);
        mj = MAP_GYWY(self->map, hit.v[1]);

      }
      else
      {
        obs_range = calc_range(self->map, pose.v[0], pose.v[1], pose.v[2] + obs_bearing, data->range_max, &mi, &mj,
                               &ml);
      }
      obs_bearing = data->ranges[i][1];
      obs_label = data->labels[i];

      //OSCAR: This is messy, make it better
      //CHRIS: Made this better
      int idx = LabelToIndex(obs_label);
      double l_hit_weight = 1.0;
      if (idx != -1){
        l_hit_weight = lblWeight[idx];
      }

      double pz_i = 0.0;
      double pl_i = 0.0;
      double ps_i = 0.0;

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      // Same for label (not in map)
      bool mapValid = MAP_VALID(self->map, mi, mj);
      double min(20.0);
      int min_idx = -1;

      if (!mapValid)
        z = self->map->max_occ_dist;
      else
      {
        z = self->map->cells[MAP_INDEX(self->map, mi, mj)].occ_dist;
        // min = 14;
        // min = self->map->cells[MAP_INDEX(self->map, mi, mj)].label_dist[0];
      }

      if (!mapValid || idx == -1) //TODO: Something more clever with the labels (like person is always closer)
        l = self->map->max_label_dist;
      else
      {       
        // Cheap RaSpectLoc version here
        // l = (min * 0.9) + (self->ramanUtil.wasserstein_likilihoods[idx][min_idx]);

        //Original SeDAR label distance value
        l = self->map->cells[MAP_INDEX(self->map, mi, mj)].label_dist[idx];

        //Find minimum distance to the nearest label
        for(int k = 0; k < MATERIALS; ++k)
        {
          if((self->map->cells[MAP_INDEX(self->map, mi, mj)].label_dist[k]) < min)
          {
            min = self->map->cells[MAP_INDEX(self->map, mi, mj)].label_dist[k];
            min_idx = k;
          }
        }

      }
      // Gaussian model (Depth)
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz_i += (self->z_hit * z_hit_norm * exp(-(z * z) / z_hit_denom));
      // Part 2: random measurements
      pz_i += self->z_rand * z_rand_mult;

      //Spectra likelihood 
      // pd_i += self->z_hit * exp(-(min * min)) / z_rand_mult; //gaussian floor
      // s = self->wasserstein_likilihoods[idx][min_idx]; //spectral likelihood precompute
      // s = self->ramanUtil.wasserstein_likilihood2((self->ramanUtil.spectra[idx]), self->ramanUtil.spectra[min_idx]); // wassser on the fly
      // std::vector<double> observed(self->ramanUtil.spectra[idx]);
      // (void)self->ramanUtil.add_shot_noise(observed, 0);
      // s = self->ramanUtil.spectralLinearKernel_likelihood(observed, self->ramanUtil.spectra[min_idx]); //spectral linear kernel
      // s = self->ramanUtil.wasserstein_likilihood2(self->ramanUtil.spectra[idx], self->ramanUtil.spectra[min_idx]); //spectral linear kernel
      
      // ROS_INFO_STREAM("MIN: " << min << "   idx_min: " << min_idx);

      // Gaussian model (Label)
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)

      // ROS_INFO_STREAM("IDX: " << idx << "min_idx: " << min_idx);
      if(idx != -1)
      {
        pl_i += (self->z_hit * exp(-(l * l) / lblDenom[idx]) * lblNorm[idx]);
        
        //Spectra likelihood here instead incase of idx out of bounds
        // ROS_INFO_STREAM("min_idx: " << min_idx << "  idx: " << idx << "   is_mapvalid: " << mapValid);
        if(mapValid)
        {
          // switch(self->sim_func_type)
          // {
          // case WASSERSTEIN:
          //   // s = self->ramanUtil.wasserstein_likilihood2(self->ramanUtil.spectra[idx], self->ramanUtil.spectra[min_idx]);
          //   s = self->ramanUtil.wasserstein_likelihood2(self->noise_spectra[idx], self->ramanUtil.spectra[min_idx]);            
          //   break;
          // case SPECTRAL_LINEAR_KERNEL:
          //   s = self->ramanUtil.spectralLinearKernel_likelihood(self->ramanUtil.spectra[idx], self->ramanUtil.spectra[min_idx]);
          //   break;
          // case KL_DIVERGENCE:
          //   s = self->ramanUtil.kl_div(self->ramanUtil.spectra[idx], self->ramanUtil.spectra[min_idx]);
          //   break;
          // case MODIFIED_EUCLIDEAN:
          //   s = self->ramanUtil.mod_euclidian(self->ramanUtil.spectra[idx], self->ramanUtil.spectra[min_idx]);
          //   break;
          // case PEAK_EUCLIDEAN:
          //   s = self->ramanUtil.peak_euclidian(self->ramanUtil.spectra[idx], self->ramanUtil.spectra[min_idx]);
          //   break;
          // case SAM:
          //   s = self->ramanUtil.calculate_sam(self->ramanUtil.spectra[idx], self->ramanUtil.spectra[min_idx]);
          //   break;
          // default:
          //   s = self->ramanUtil.wasserstein_likelihood2(self->ramanUtil.spectra[idx], self->ramanUtil.spectra[min_idx]);
          //   break;
          // }
          s = 0.00;
          // s = self->ramanUtil.wasserstein_likilihood2(self->ramanUtil.spectra[idx], self->ramanUtil.spectra[min_idx]); 
        }  
      
        else
          s = 0.01;
        ps_i += s;
      }
      // Part 2: random measurements
      pl_i += self->z_rand * z_rand_mult;
      // ps_i += self->s_rand * s_rand_mult;

      // Part 2: short reading from unexpected label (therefore, obstacle)
      if (idx == -1)
      {
        pl_i += self->z_short * self->lambda_short * exp(-self->lambda_short * l);
        ps_i += 0.01;
        // ps_i += self->z_short * self->lambda_short * exp(-self->lambda_short * s);      
      }
      assert(pz_i <= 1.0);
      assert(pz_i >= 0.0);
      // if (pl_i > 1.0)
      // {
      //   ROS_INFO("l: %f ", l);
      //   ROS_INFO("idx: %i ", idx);
      //   ROS_INFO("pl_i: %f", pl_i);
      //   ROS_INFO("l_w: %f", l_hit_weight);
      //   //ROS_INFO("l_n: %f", l_hit_norm);
      //   ROS_INFO("z_h: %f", self->z_hit);
      //   ROS_INFO("z_s: %f", self->z_short);
      //   ROS_INFO("l_s: %f", self->lambda_short);
      //   ROS_INFO("z_r: %f", self->z_rand);
      //   ROS_INFO("r_m: %f", z_rand_mult);
      // }

      // assert(pl_i <= 1.0);
      // assert(pl_i >= 0.0);
      // assert(ps_i <= 1.0);
      // assert(ps_i >= 0.0);

      //      p *= pz_i;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      pz += (pz_i * pz_i * pz_i);
      pl += (pl_i * pl_i * pl_i);
      //ad-hoc weighting for the spectral likelihood as well
      ps += (ps_i * ps_i * ps_i);

    }
    //ROS_INFO("P_L: %f", pl);
    //Original SeDAR
    // p = ((self->wz * pz) + (self->wl * pl)) / (self->wz + self->wl);
    //New S-SeDAR
    p = ((self->wz * pz) + (self->wl * pl) + (self->ws * ps)) / (self->wz + self->wl + self->ws);
    sample->weight *= p;
#pragma omp critical
    {
      total_weight += sample->weight;
    }
  }
  return (total_weight);
}

double AMCLSemantic::LikelihoodFieldModelProb(AMCLSemanticData *data, pf_sample_set_t* set)
{
  AMCLSemantic *self;
  int i, j, step;
  double z, pz;
  double log_p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLSemantic*)data->sensor;

  total_weight = 0.0;

  step = ceil((data->range_count) / static_cast<double>(self->max_beams));

  // Step size must be at least 1
  if (step < 1)
    step = 1;

  // Pre-compute a couple of things
  double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
  double z_rand_mult = 1.0 / data->range_max;

  double max_dist_prob = exp(-(self->map->max_occ_dist * self->map->max_occ_dist) / z_hit_denom);

  //Beam skipping - ignores beams for which a majoirty of particles do not agree with the map
  //prevents correct particles from getting down weighted because of unexpected obstacles
  //such as humans

  bool do_beamskip = self->do_beamskip;
  double beam_skip_distance = self->beam_skip_distance;
  double beam_skip_threshold = self->beam_skip_threshold;

  //we only do beam skipping if the filter has converged
  if (do_beamskip && !set->converged)
  {
    do_beamskip = false;
  }

  //we need a count the no of particles for which the beam agreed with the map
  int *obs_count = new int[self->max_beams]();

  //we also need a mask of which observations to integrate (to decide which beams to integrate to all particles)
  bool *obs_mask = new bool[self->max_beams]();

  int beam_ind = 0;

  //realloc indicates if we need to reallocate the temp data structure needed to do beamskipping
  bool realloc = false;

  if (do_beamskip)
  {
    if (self->max_obs < self->max_beams)
    {
      realloc = true;
    }

    if (self->max_samples < set->sample_count)
    {
      realloc = true;
    }

    if (realloc)
    {
      self->reallocTempData(set->sample_count, self->max_beams);
      fprintf(stderr, "Reallocing temp weights %d - %d\n", self->max_samples, self->max_obs);
    }
  }

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    log_p = 0;

    beam_ind = 0;

    for (i = 0; i < data->range_count; i += step, beam_ind++)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if (obs_range >= data->range_max)
      {
        continue;
      }

      // Check for NaN
      if (obs_range != obs_range)
      {
        continue;
      }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance

      if (!MAP_VALID(self->map, mi, mj))
      {
        pz += self->z_hit * max_dist_prob;
      }
      else
      {
        z = self->map->cells[MAP_INDEX(self->map, mi, mj)].occ_dist;
        if (z < beam_skip_distance)
        {
          obs_count[beam_ind] += 1;
        }
        pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      }

      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)

      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      assert(pz <= 1.0);
      assert(pz >= 0.0);

      // TODO: outlier rejection for short readings

      if (!do_beamskip)
      {
        log_p += log(pz);
      }
      else
      {
        self->temp_obs[j][beam_ind] = pz;
      }
    }
    if (!do_beamskip)
    {
      sample->weight *= exp(log_p);
      total_weight += sample->weight;
    }
  }

  if (do_beamskip)
  {
    int skipped_beam_count = 0;
    for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++)
    {
      if ((obs_count[beam_ind] / static_cast<double>(set->sample_count)) > beam_skip_threshold)
      {
        obs_mask[beam_ind] = true;
      }
      else
      {
        obs_mask[beam_ind] = false;
        skipped_beam_count++;
      }
    }

    //we check if there is at least a critical number of beams that agreed with the map
    //otherwise it probably indicates that the filter converged to a wrong solution
    //if that's the case we integrate all the beams and hope the filter might converge to
    //the right solution
    bool error = false;

    if (skipped_beam_count >= (beam_ind * self->beam_skip_error_threshold))
    {
      fprintf(
          stderr,
          "Over %f%% of the observations were not in the map - pf may have converged to wrong pose - integrating all observations\n",
          (100 * self->beam_skip_error_threshold));
      error = true;
    }

    for (j = 0; j < set->sample_count; j++)
    {
      sample = set->samples + j;
      pose = sample->pose;

      log_p = 0;

      for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++)
      {
        if (error || obs_mask[beam_ind])
        {
          log_p += log(self->temp_obs[j][beam_ind]);
        }
      }

      sample->weight *= exp(log_p);

      total_weight += sample->weight;
    }
  }

  delete[] obs_count;
  delete[] obs_mask;
  return (total_weight);
}

void AMCLSemantic::reallocTempData(int new_max_samples, int new_max_obs)
{
  if (temp_obs)
  {
    for (int k = 0; k < max_samples; k++)
    {
      delete[] temp_obs[k];
    }
    delete[] temp_obs;
  }
  max_obs = new_max_obs;
  max_samples = fmax(max_samples, new_max_samples);

  temp_obs = new double*[max_samples]();
  for (int k = 0; k < max_samples; k++)
  {
    temp_obs[k] = new double[max_obs]();
  }
}
