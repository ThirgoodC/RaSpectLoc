/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey et al.
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
///////////////////////////////////////////////////////////////////////////
//
// Desc: LASER sensor model for AMCL
// Author: Andrew Howard
// Date: 17 Aug 2003
// CVS: $Id: amcl_laser.h 6443 2008-05-15 19:46:11Z gerkey $
//
///////////////////////////////////////////////////////////////////////////
#ifndef AMCL_LASER_H
#define AMCL_LASER_H

#include "amcl_sensor.h"
#include "../map/map.h"
#include "raman_lut.h"

namespace cvssp_amcl
{

typedef enum
{
  LASER_MODEL_BEAM, LASER_MODEL_LIKELIHOOD_FIELD, LASER_MODEL_LIKELIHOOD_FIELD_PROB
} laser_model_t;

typedef enum
{
  WASSERSTEIN, KL_DIVERGENCE, SPECTRAL_LINEAR_KERNEL, MODIFIED_EUCLIDEAN, PEAK_EUCLIDEAN, SAM
} sim_func_t;

// Semantic sensor data
class AMCLSemanticData : public AMCLSensorData
{
public:
  AMCLSemanticData()
  {
    ranges = NULL;
  }
  ;
  virtual ~AMCLSemanticData()
  {
    delete[] ranges;
  }
  ;

public:
  int range_count;
  double range_max;
  double (*ranges)[2]; // Depth data (range, bearing tuples)
  int (*labels); // Semantic data (label)
};

// Laseretric sensor model
class AMCLSemantic : public AMCLSensor
{
  // Default constructor
public:
  AMCLSemantic(size_t max_beams, map_t* map, bool use_depth, std::string const& sim_func);

  virtual ~AMCLSemantic();



  void SetModelBeam(double z_hit, double z_short, double z_max, double z_rand, double sigma_hit, double labda_short,
                    double chi_outlier, double fudge_factor);

  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);

  // Set the laser's pose after construction
  void SetLaserPose(pf_vector_t& laser_pose)
  {
    this->laser_pose = laser_pose;
  }

  void SetWeightLabel(double& w)
  {
    this->wl = w;
  }
  void SetWeightDepth(double& w)
  {
    this->wz = w;
  }
  void SetWeightSpectra(double& w)
  {
    this->ws = w;
  }

  void SetUseDepth(bool d)
  {
    this->use_depth = d;
  }
  // raman_lut ramanUtil = raman_lut("/home/ct00659/Downloads/raman_scans_1/spectras.txt");
  raman_lut ramanUtil;

  // Determine the probability for the given pose
private:
  sim_func_t hashit(std::string const& sim_func);

  static double BeamModel(AMCLSemanticData *data, pf_sample_set_t* set);

  // Current data timestamp
  double time;

  // The semantic map
  map_t *map;

  // Laser offset relative to robot
  pf_vector_t laser_pose;

  // Max beams to consider
  int max_beams;

  // Laser model params
  //
  // Mixture params for the components of the model; must sum to 1
  double z_hit;
  double z_short;
  double z_max;
  double z_rand;

  //
  // Stddev of Gaussian model for laser hits.
  double sigma_hit;
  // Decay rate of exponential model for short readings.
  double lambda_short;
  // Threshold for outlier rejection (unused)
  double chi_outlier;

  //Oscar: Weights for Label vs. Depth
  bool use_depth;
  double wl;
  double wz;

  //Chris: Use spectra weights
  double ws;
  double wasserstein_likilihoods[MATERIALS][MATERIALS];
  std::vector<double> noise_spectra[MATERIALS];
  double s_rand; //used for spectra likelihood
  double fudge_factor;
  unsigned int similarity_function_idx;

  //UNUSED
  //UNUSED
  //UNUSED
private:
  laser_model_t model_type;
  sim_func_t sim_func_type;

  // Beam skipping parameters (used by LikelihoodFieldModelProb model)
private:
  bool do_beamskip;
private:
  double beam_skip_distance;
private:
  double beam_skip_threshold;
  //threshold for the ratio of invalid beams - at which all beams are integrated to the likelihoods
  //this would be an error condition
private:
  double beam_skip_error_threshold;

  //temp data that is kept before observations are integrated to each particle (requried for beam skipping)
private:
  int max_samples;
private:
  int max_obs;
private:
  double **temp_obs;

public:
  void SetModelLikelihoodField(double z_hit, double z_rand, double s_rand, double sigma_hit, double max_occ_dist,
                               double max_label_dist, double lambda_short, double fudge_factor);

  //a more probabilistically correct model - also with the option to do beam skipping
public:
  void SetModelLikelihoodFieldProb(double z_hit, double z_rand, double s_rand, double sigma_hit, double max_occ_dist,
                                   double max_label_dist, bool do_beamskip, double beam_skip_distance,
                                   double beam_skip_threshold, double beam_skip_error_threshold);

  // Determine the probability for the given pose
private:
  static double LikelihoodFieldModel(AMCLSemanticData *data, pf_sample_set_t* set);

  // Determine the probability for the given pose - more probablistic model
private:
  static double LikelihoodFieldModelProb(AMCLSemanticData *data, pf_sample_set_t* set);

private:
  void reallocTempData(int max_samples, int max_obs);
};

}

#endif
