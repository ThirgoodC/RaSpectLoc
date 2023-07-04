#include "raman_lut.h"

#include "wasserstein.h"
#include "../sklearn_cpp/lsr.h"

#include <string>
#include <typeinfo>
#include <random>

#include <chrono>
using namespace std::chrono;

#define DEBUG 1

using namespace cvssp_amcl;

std::vector<double> weights;

raman_lut::raman_lut() {}

raman_lut::raman_lut(std::string spectrasPath)
{
    std::vector<std::string> spectraPaths; 
    // std::fstream file(spectraFile0, std::ios::in);
    std::string line, word;
    std::vector<std::vector<std::string>> content;
    // std::vector<std::string> row;

    std::fstream file(spectrasPath, std::ios::in);
    if(file.is_open())
    {
        while (getline(file, line))
        {
            std::stringstream str(line);
            while(getline(str, word))
                spectraPaths.push_back(word);
                std::cout << word << std::endl;
        }
    }
    else
    {
        std::cout<<"Could not open the file\n";
    }
    file.close();

    for(int k=0; k < spectraPaths.size(); ++k){
// #ifdef DEBUG
        read_spectra(spectraPaths[k]);
// #endif
    }

    weights = spectra[0];
    //set up the weights
    for(int i(0); i < weights.size(); ++i)
    {
        weights[i] = 2*i + 2;
    }
}

raman_lut::~raman_lut()
{}

void raman_lut::read_spectra(std::string spectraFiles)
{
    std::string line, word;
    std::vector<std::vector<std::string>> content;

    std::fstream file(spectraFiles, std::ios::in);
    
    if(file.is_open())
    {
        while(getline(file, line))
        {
            std::vector<std::string> row;

            std::stringstream str(line);

            while(getline(str, word, ','))
                row.push_back(word);

            content.push_back(row);
        }
    } else {
        std::cout<<"Could not open the file\n";
    }

    std::vector<double> data;
    for(int i=0;i<content.size();i++)
    {
        data.push_back(std::stod(content[i][1]));
        if(wavelengths.size() <= 1024)
        {
            wavelengths.push_back(std::stof(content[i][0]));
            // std::cout << content[i][0] << std::endl;
        }
    }

    for(auto x : wavelengths)
    {
        // std::cout << x << std::endl;
    }
    
    spectra.push_back(data);

    file.close();
    
}

int raman_lut::wasserstein_likilihood(std::vector<double> spectrum2, const bool use_nspec)
{
    std::vector<double> distances;

    std::vector<double> weights(spectrum2);
    for(int i(0); i < weights.size(); ++i)
    {
        weights[i] = 2*i + 2;
    }

    for(int j = 0; j < MATERIALS; ++j)
    {
        if (use_nspec)      
            distances.push_back(wasserstein(weights,spectra2[j],weights,spectrum2));
        else 
            distances.push_back((exp(-(wasserstein(weights,spectra[j],weights,spectrum2))/10)));


        // std::cout << "index : " << j << "  spectra check: " << distances[j] << std::endl; 
    } 

    // std::cout << "\nMax Element = "
        // << *max_element(distances.begin(), distances.end()) << std::endl;

    double min = *std::max_element(distances.begin(), distances.end());

    auto it = find(distances.begin(), distances.end(), min);
    
    int index(-1);
    
    if (it != distances.end()) 
    {
        index = it - distances.begin();
        std::cout << index << std::endl;
    }

    return index;
}

double raman_lut::wasserstein_likelihood2(const std::vector<double>& spectrum1, std::vector<double>& spectrum2)
{
    double likelihood(0.0);
    // std::cout << spectrum2[10] << std::endl;


    //add some noise 
    // add_shot_noise(spectrum2,0);

    //compute the likelihood between 0->1
    likelihood  = exp(-std::pow(wasserstein(weights,spectrum1,weights,spectrum2), 2)/500);

    // std::cout << spectrum2[10] << std::endl;


    return likelihood;
}

/**
 * @brief This function adds noise to all the currently stored spectra. It will
 * need to be updated to in the future to correctly add shot, dark and read noise
 * which is the sqrt of the number of events (could be the count)
 * 
 * @param stdev standard deviation of noise to be added
 * @param mean mean of the noise to be added
 */
void raman_lut::add_noise(const double stdev, const double mean)
{
    std::default_random_engine gen;
    std::normal_distribution<double> dist(mean,stdev);

    spectra2 = spectra;

    for(int i = 0; i < MATERIALS; ++i)
    {
        for(auto& x : spectra2[i])
        {
            x = x + dist(gen);
        }

        // std::copy(begin(spectra2[i]), end(spectra2[i]), std::ostream_iterator<double>(std::cout, " "));
        // std::copy(begin(spectra[i]), end(spectra[i]), spectra2[i]);

        std::cout << std::endl;
    }
}

/**
 * @brief overloaded function of the add-Noise function. Adds noise to the query vector of intensities
 * 
 * @param stdev standard deviation of noise
 * @param mean mean of the noise
 * @param query vector of intensities
 */
void raman_lut::add_noise(const double stdev, const double mean, std::vector<double>& query, int filenum)
{
    // std::default_random_engine gen;
    // std::normal_distribution<double> dist(mean,stdev);

    // // spectra2 = spectra;
    // for(auto& x : query)
    // {
    //     x = x + dist(gen);
    // }

    std::cout << "shit the bed" << std::endl;
    std::string attempt = "/home/ct00659/Documents/nshotspectra_fudge" + std::to_string(filenum) + ".csv";
    std::ofstream noisy_spec(attempt);
    int i = 0;

    for(auto& x : query)
    {
        // std::cout << std::to_string(wavelengths[0]) << std::endl;
        noisy_spec << /*std::to_string(wavelengths[i]) << "," <<*/ x << '\n';
        i++;
    }

    noisy_spec.close();
}

void raman_lut::add_shot_noise(std::vector<double>& query, int filenum)
{
    std::mt19937 rng(100);
    
    // spectra2 = spectra;
    for(auto& x : query)
    {
        std::uniform_int_distribution<int> gen(-(std::sqrt(x)), (std::sqrt(x)));
        x = x + 1*gen(rng);
    }

    // write_to_file_spectra(query);
    // std::string attempt = "/home/ct00659/Documents/nshotspectra_fudge" + std::to_string(filenum) + ".csv";
    // std::ofstream noisy_spec(attempt);
    // int i = 0;

    // for(auto& x : query)
    // {
    //     // std::cout << std::to_string(wavelengths[0]) << std::endl;
    //     noisy_spec << /*std::to_string(wavelengths[i]) << "," <<*/ x << '\n';
    //     i++;
    // }

    // noisy_spec.close();
}

void raman_lut::write_to_file_spectra(std::vector<double>& nspectra)
{
    // std::cout << "shit the bed";
    std::ofstream noisy_spec("/home/ct00659/Documents/nspectra0.txt");

    for(int i=0; i < nspectra.size(); ++i)
    {
        noisy_spec << wavelengths[i] << "," << nspectra[i] << '\n';
    }

    noisy_spec.close();
}

#include "../peak_finder/PeakFinder.h" 
using namespace PeakFinder;

// void  raman_lut::find_peaks(std::vector<double> spec)
// {
//     std::vector<int> out;
//     std::vector<float> specky(spec.begin(), spec.end());
//     PeakFinder::findPeaks((std::vector< float >)specky, out, false);

// 	if(out.size()==0)
// 	{
// 		std::cout<<"No peaks"<<std::endl;
// 	}

// 	std::cout<<"Maxima found:"<<std::endl;

// 	for(int i=0; i<out.size(); ++i)
// 		std::cout<< specky[out[i]] << " ";

// 	std::cout << std::endl;


// }

double raman_lut::peak_euclidian(const std::vector<double>& spec1, std::vector<double>& spec2)
{
    std::vector<int> out1, out2; // output peaks
    std::vector<float> floatSpec1(spec1.begin(), spec1.end()); //convert to a float vector //This is slow, use a condition in order to use floats in storage, or change source for find_peaks.
    std::vector<float> floatSpec2(spec2.begin(), spec2.end()); //convert to a float vector
    PeakFinder::findPeaks(floatSpec1, out1, false);
    PeakFinder::findPeaks(floatSpec2, out2, false);

    double maxQ1(0.0), binq1(0.0), maxQ2(0.0), binq2(0.0);
    double diff(0.0);
    bool use_maxq1(false), use_maxq2(false);

	if(out1.size()==0)
	{
		// std::cout << "No peaks max1 - maxq1:  " << maxQ1 <<std::endl;
        maxQ1 = (*std::max_element(spec1.begin(), spec1.end()));
        binq1 = std::distance(spec1.begin(), std::max_element(spec1.begin(), spec1.end()));
		std::cout << "No peaks max1 - maxq1:  " << maxQ1 << "  bin: " << binq1 << std::endl;
        use_maxq1 = true;
	}
	if(out2.size()==0)
	{
        maxQ2 = (*std::max_element(spec2.begin(), spec2.end()));
        binq2 = std::distance(spec2.begin(), std::max_element(spec2.begin(), spec2.end()));
		std::cout << "No peaks max1 - maxq1:  " << maxQ2 << "  bin: " << binq2 << std::endl;
        use_maxq2 = true;
	}

    //Check out the peaks found
	// for(int i=0; i<out1.size(); ++i)
    // {
	// 	std::cout << "1X-bin: " << out1[i] << "    1y-bin: " << floatSpec1[out1[i]] << std::endl;
    // }
    // for(int i=0; i<out2.size(); ++i)
    // {
	// 	std::cout << "2X-bin: " << out2[i] << "    2y-bin: " << floatSpec2[out2[i]] << std::endl;
    // }

    //Check if the number of peaks are the same
    if(out1.size() != out2.size()) //Not the same
    {
        if(use_maxq1 && use_maxq2) //The first and second spectrum has none
        {
            diff = std::sqrt(std::pow((binq1 - maxQ1), 2) - std::pow((binq2 - maxQ2), 2)); 
        }
        else if (use_maxq1) //query has none
        {
            diff = std::sqrt(std::pow((binq1 - maxQ1), 2) - std::pow((out2[0] - floatSpec2[out2[0]]), 2)); 
        }
        else if (use_maxq2) //second has none
        {
            diff = std::sqrt(std::pow((out1[0] - floatSpec1[out1[0]]), 2) - std::pow((binq2 - maxQ2), 2)); 
        }
        else //un-equal number of peaks
        {
            std::cout << "not a match" << std::endl;

            int min_idx =  out1.size() < out2.size() ? out1.size() : out2.size(); //count to the highest number
            // double penalty(out1.size() / out2.size())

            for(int i(0); i < min_idx; ++i)
            {
                //use a better penalty. Maybe the ratio of the size of the arrays
                diff += 3*(std::pow((out1[i] - floatSpec1[out1[i]]), 2) - std::pow((out2[i] - floatSpec2[out2[i]]), 2)); //Is this efficient?
                std::cout << "diff:  " << diff << std::endl;
            }
            
            diff = std::sqrt(std::abs(diff));
        }
    }
    else
    {
        // double diff(0);
        for(int i=0; i<out1.size(); ++i)
        {
            diff += std::pow((out1[i] - floatSpec1[out1[i]]), 2) - std::pow((out2[i] - floatSpec2[out2[i]]), 2); 
            std::cout << "diff:  " << diff << std::endl;
        }
        diff = std::sqrt(std::abs(diff));

        std::cout << "result:  " << diff << std::endl;
    }


	// std::cout << std::endl;

    return diff;
}

double raman_lut::spectralLinearKernel_likelihood(const std::vector<double>& spectrum1, 
                                                  const std::vector<double>& spectrum2)
{
    const int windowLen = 3;
    long long int sum = 0;

    for(int i = windowLen; i  < spectrum1.size() - windowLen; ++i)
    {
        long long int product = spectrum1[i] * spectrum2[i];

        // Window over data
        long long int windowSum = 0;
        for(int j = -(windowLen); j <= windowLen; ++j)
        {
            windowSum = windowSum + ((spectrum1[i] - spectrum1[i+j]) * (spectrum2[i] - spectrum2[i+j]));
            // std::cout << "sum val:" << windowSum << std::endl;

        }

        sum += (product + windowSum);
        // std::cout << "sum val:" << sum << std::endl;
    }
// std::cout << "sum val:" << sum << std::endl;
    // double likelihood =  1/(1+exp(-(std::pow(sum, 2))/exp(-(std::pow(6000000, 2)))));
    double likelihood =  1/ (1 + exp(-(std::pow(sum/1000, 2))/100000000));
    likelihood = (likelihood * 2) - 1;
    // std::cout << (-(sum)/7500000) << std::endl;
    // double likelihood =  1/(1+exp(-(sum))/10);
    // std::cout << (-(sum)/10) << std::endl;



    return likelihood;
}

/**
 * @brief Modified euclidean function from SOTA similarity paper.
 * This must be used with normailised spectra and baseline removed spectra
 * 
 * @param spec1 
 * @param spec2 
 * @return double 
 */
double raman_lut::mod_euclidian(const std::vector<double>& spec1, std::vector<double>& spec2)
{

    add_shot_noise(spec2,0);


    double maxQ(*std::max_element(spec1.begin(), spec1.end()));

    //find weight
    double weight = maxQ/(1-maxQ); 
    //This doesnt make sence for a normalised spectra 
    //Instead inverse it for the main function eg *1/w for condition 1, *w for condition 2 
    // std::cout << " mq: " << maxQ << std::endl;
    // std::cout << " w: " << weight << std::endl;
    double sqDist = 0.0;

    for(int i = 0; i < spec1.size(); ++i)
    {
        double dist = std::pow((spec2[i] - spec1[i]), 2);

        if((spec2[i] > spec1[i]) && (spec1[i] != 0)) //condution 1
            dist = (1/weight) * dist;
        else if((spec2[i] > spec1[i]) && (spec1[i] == 0)) //condtion 2
            dist = (weight) * dist;
        else
            dist = dist;

    
        // std::cout << " dist: " << dist << std::endl;
        sqDist += dist;
    }

    // double likelihood = std::sqrt(sqDist);

    // std::cout << " res: " << sqDist << std::endl;

    // return 1/(1+exp(-(sqDist * 1000000)));

    double likelihood  = exp(-(std::pow(std::sqrt(sqDist), 2))/0.5);

    return likelihood;    
}

double raman_lut::kl_div(const std::vector<double>& spec1, std::vector<double>& spec2)
{
    double res(0.0);

    add_shot_noise(spec2,0);

    for(int i(0); i < spec1.size(); ++i)
    {
        double val = spec1[i];
        double log_d(0.0);
        if(spec2[i] > 0 && spec2[i] != 0)
            log_d = log(spec1[i]/spec2[i]); 
        else
            log_d = -100; //This is terrible, please do this with epsilon when working

        res += (val * log_d);

    }
    double likelihood  = exp(-(std::pow(res, 2))/10);

    if(isnan(likelihood) || likelihood < 0.00001)
        likelihood = 0.0;

    return likelihood;
}

double raman_lut::calculate_sam(const std::vector<double>& spectrum1, std::vector<double>& spectrum2) 
{
    add_shot_noise(spectrum2,0);

    double numerator = 0.0;
    double denominator1 = 0.0;
    double denominator2 = 0.0;
    
    for (int i = 0; i < spectrum1.size(); i++) {
        numerator += spectrum1[i] * spectrum2[i];
        denominator1 += spectrum1[i] * spectrum1[i];
        denominator2 += spectrum2[i] * spectrum2[i];
    }
    
    double denominator = sqrt(denominator1) * sqrt(denominator2);
    double angle = acos(numerator / denominator);

    // std::cout << numerator / denominator << std::endl;
    double likelihood  = exp(-(std::pow(std::sqrt(angle), 2)/0.2));

    if (std::isnan(likelihood))
    {
        likelihood = 0.99;
    }
    if(likelihood == 1.0)
        likelihood = 0.99;
    // else if (lieklihood < 0.75)    
    return likelihood;
}