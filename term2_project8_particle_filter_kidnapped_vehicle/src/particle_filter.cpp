/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */
#define _USE_MATH_DEFINES
#include <math.h>
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

// declaration of a random engine
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	//num_particles = 8000; // 8000 - x.107 y.095 yaw.003 time just right (very slow ~80sec)
  //num_particles = 1000; // 1000 - x.106 y.099 yaw.003, much faster (~ 20sec), nearly as accurate as 8000 particles
  num_particles = 100;  //  100 - x.114 y.105 yaw.004, (~19sec), not as accurate but sufficient
  //num_particles = 50;   //   50 - x.117 y.155 yaw.004, (~16sec), not as accurate but sufficient
  //num_particles = 10;   //   10 - x.156 y.147 yaw.005, (~15sec), not as accurate but sufficient
  //num_particles = 5;   //     5 - x.211 y.187 yaw.007, (~15sec), not as accurate but sufficient



  is_initialized = true;
  Particle part_new;
  //Set standard deviations for x, y, and psi.
  //double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]

  // Create a normal (Gaussian) distribution for x.
	normal_distribution<double> dist_x(x, std[0]);
  // Create a normal (Gaussian) distribution for y.
	normal_distribution<double> dist_y(y, std[1]);
	// Create a normal (Gaussian) distribution for theta.
	normal_distribution<double> dist_theta(theta, std[2]);

	gen.seed(time(NULL));

	for (int i = 0; i < num_particles; ++i) {

    part_new.id = i;
    part_new.x = dist_x(gen);
    part_new.y = dist_y(gen);
    part_new.theta = dist_theta(gen);
    part_new.weight = 1.0;
    particles.push_back(part_new);
    weights.push_back(1.0);
	}
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	double x_new;
	double y_new;
	double theta_new;

	double dist_ = velocity*delta_t;
	double yaw_ = yaw_rate*delta_t;
	double coeff_;
	if(fabs(yaw_rate) > 0.0001){
    coeff_ = velocity/yaw_rate;
	}

	for (int i = 0; i < num_particles; ++i) {
    // calculate new position
    // yaw rate == 0.
		if (fabs(yaw_rate) < 0.0001){
      x_new = particles[i].x + dist_*cos(particles[i].theta);
      y_new = particles[i].y + dist_*sin(particles[i].theta);
      theta_new = particles[i].theta;
    }
    // yaw rate > 0.
    else{
      x_new = particles[i].x + coeff_*(sin(particles[i].theta + yaw_) - sin(particles[i].theta));
      y_new = particles[i].y + coeff_*(cos(particles[i].theta) - cos(particles[i].theta + yaw_));
      theta_new = particles[i].theta +yaw_;
     }
    // Create a normal (Gaussian) distribution for x.
    normal_distribution<double> dist_x(x_new, std_pos[0]);
    // Create a normal (Gaussian) distribution for y.
    normal_distribution<double> dist_y(y_new, std_pos[1]);
    // Create a normal (Gaussian) distribution for theta.
    normal_distribution<double> dist_theta(theta_new, std_pos[2]);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

  for (int i = 0; i < observations.size(); i++) {

    // get the current observation
    LandmarkObs lmObs = observations[i];

    // init minimum distance to value greater than 50m (radius of the sensor)
    double dMin_dist = 100.;

    // init id of landmark
    int iLandm_id = -1;

    for (int k = 0; k < predicted.size(); k++) {
      // get the current prediction
      LandmarkObs lmPre = predicted[k];

      // calculate distance between landmark observation and prediciton
      double dObs_pre_dist = dist(lmObs.x, lmObs.y, lmPre.x, lmPre.y);

      // find the landmark nearest the current observation
      if (dObs_pre_dist < dMin_dist) {
        dMin_dist = dObs_pre_dist;
        iLandm_id = lmPre.id;
      }
    }

    // set the observation's id to the nearest predicted landmark's id
    observations[i].id = iLandm_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

  // cout<<"num particles: "<<num_particles<<"\n";
  double dStd_x = std_landmark[0];
  double dStd_y = std_landmark[1];
  // precalculate parts for multivariate Gaussian
  //double dMVG_part1 = ( 1/(2*M_PI*dStd_x*dStd_y));
  double dMVG_part1 = 1;
  double dMVG_part2 = (2*pow(dStd_x, 2));
  double dMVG_part3 = (2*pow(dStd_y, 2));

  double dWeight_sum = 0;

  // for each particle...
  for (int i = 0; i < num_particles; i++) {
    // get the particle x, y coordinates
    double dPart_x = particles[i].x;
    double dPart_y = particles[i].y;
    double dPart_theta = particles[i].theta;
    // create a vector to hold the map landmark locations predicted to be within sensor range of the particle
    vector<LandmarkObs> lmPredictions;
    // for each map landmark...
    for (int m = 0; m < map_landmarks.landmark_list.size(); m++) {

      // get id and x,y coordinates
      double dLandm_x = map_landmarks.landmark_list[m].x_f;
      double dLandm_y = map_landmarks.landmark_list[m].y_f;
      int iLandm_id = map_landmarks.landmark_list[m].id_i;

      // identify landmarks within sensor range of the particle
      if (fabs(dist(dLandm_x, dLandm_y, dPart_x, dPart_y)) <= sensor_range) {
        // landmark within range then add to vector
        lmPredictions.push_back(LandmarkObs{ iLandm_id, dLandm_x, dLandm_y });
      }
    }

    // transform the observations from vehicle coordinates to map coordinates
    vector<LandmarkObs> lmTrans_os;
    for (int k = 0; k < observations.size(); k++) {
      double dTrans_x = observations[k].x * cos(dPart_theta) - observations[k].y * sin(dPart_theta) + dPart_x;
      double dTrans_y = observations[k].x * sin(dPart_theta) + observations[k].y * cos(dPart_theta) + dPart_y;
      lmTrans_os.push_back(LandmarkObs{ observations[k].id, dTrans_x, dTrans_y });
    }

    // finding nearest neighbour for transformed observations
    dataAssociation(lmPredictions, lmTrans_os);
/*
    vector<int> viLm_Associations;
    vector<double> vdLm_x;
    vector<double> vdLm_y;

    for(int r = 0; r< lmTrans_os.size(); r++){
      viLm_Associations.push_back(lmTrans_os[r].id);
      vdLm_x.push_back(lmTrans_os[r].x);
      vdLm_y.push_back(lmTrans_os[r].y);
    }

    SetAssociations(particles[i], viLm_Associations, vdLm_x , vdLm_y );
*/
    // reinit weight
    particles[i].weight = 1.0;



    for (int n = 0; n < lmTrans_os.size(); n++) {

      // current observation
      double dObs_x = lmTrans_os[n].x;
      double dObs_y = lmTrans_os[n].y;
      int iLandm_id = lmTrans_os[n].id;

      // placeholder for prediction coordinates
      double dPre_x;
      double dPre_y;

       // find corresponding prediction
      for (int q = 0; q < lmPredictions.size(); q++) {
        if (lmPredictions[q].id == iLandm_id) {
          dPre_x = lmPredictions[q].x;
          dPre_y = lmPredictions[q].y;
        }
      }

      // calculate weight for current observation
      //multivariate Gaussian
      double dObs_weight = dMVG_part1 * exp( -( pow(dPre_x-dObs_x,2)/dMVG_part2  + (pow(dPre_y-dObs_y,2)/dMVG_part3) ) );

      // final weight
      particles[i].weight *= dObs_weight;
    }
    weights[i] =particles[i].weight;
    dWeight_sum += particles[i].weight;
  }
  // for each particle...
  for (int i = 0; i < num_particles; i++){
    particles[i].weight /=dWeight_sum;
  }

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  vector<Particle> pNew_part;

  discrete_distribution<int> iDisc_dist(begin(weights),end(weights));


  for(int i = 0; i < num_particles; i++)
  {
    pNew_part.push_back(particles[iDisc_dist(gen)]);
  }
  particles = pNew_part;
/*
// or Resampling wheel!!!

  // generate random starting index [0, num_particles-1]
  uniform_int_distribution<int> distUni_i(0, num_particles-1);
  int index = distUni_i(gen);

  double beta = 0.0;
  // max weight
  double dWeight_max = *max_element(begin(weights),end(weights));

  // uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> distUni_r(0.0, (2*dWeight_max));

  // resampling wheel
  for (int i = 0; i < num_particles; i++) {
    beta += distUni_r(gen);
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    pNew_part.push_back(particles[index]);
  }

particles = pNew_part;
*/
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
