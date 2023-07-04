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
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: Adaptive Monte-Carlo localization
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_sensor.h 6443 2008-05-15 19:46:11Z gerkey $
//
///////////////////////////////////////////////////////////////////////////

#ifndef RAMAN_LUT_H
#define RAMAN_LUT_H

#define MATERIALS 8

#include <ros/console.h>

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// Signal handling
#include <signal.h>
#include <fstream>
// #include <sstream>
#include <iomanip>
#include <iostream>

namespace cvssp_amcl
{

class raman_lut
{
public:
  raman_lut();
  raman_lut(std::string spectras);
  ~raman_lut();

  

  /**
   * @brief Find the wasserstein distance between the two spectra.
   */
  int wasserstein_likilihood(const std::vector<double> spectrum2, const bool use_nspec);
  double wasserstein_likelihood(const std::vector<double> spectrum1, std::vector<double> spectrum2);
  double wasserstein_likelihood2(const std::vector<double>& spectrum1, std::vector<double>& spectrum2);
  // int lsr_likilihood(const std::vector<double> spectrum2, const bool use_nspec);
  double spectralLinearKernel_likelihood(const std::vector<double>& spectrum1, const std::vector<double>& spectrum2);
  double kl_div(const std::vector<double>& spec1, std::vector<double>& spec2);
  double peak_euclidian(const std::vector<double>& spec1, std::vector<double>& spec2);
  double mod_euclidian(const std::vector<double>& spec1, std::vector<double>& spec2);
  double calculate_sam(const std::vector<double>& spectrum1, std::vector<double>& spectrum2);
  
  //CHRIS TODO: prob want to change this to a safer way to take off the stack
  std::vector< std::vector<double> > spectra;
  // std::vector< std::vector<double> > wavelengths;
  std::vector<float> wavelengths;
  std::vector< std::vector<double> > spectra2;
  // std::vector< std::vector<double> > peakings;
  std::vector < std::pair<float, float> > peaks;


private:
  void read_spectra(const std::string spectraFiles);
  void add_noise(const double stdev, const double mean);
  void add_noise(const double stdev, const double mean, std::vector<double>& spectra, int filenum);
  void add_shot_noise(std::vector<double>& query, int filenum);
  // void add_noise(std::vector<double>& query);

  // void bin_spectra();

  //For debugging purposes
  void write_to_file_spectra(std::vector<double>& nspectra);


  std::map<int, std::vector<float>> LUT;
};

}

#endif //RAMAN_LUT_H
