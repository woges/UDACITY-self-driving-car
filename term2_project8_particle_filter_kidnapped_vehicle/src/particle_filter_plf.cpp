/*
 * particle_filter.cpp
 *
 *      Author: Pierluigi Ferrari
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

	// Set the number of particles
  num_particles = 100;

  // Declare the random generator
  default_random_engine gen;

  // Extract the standard deviations for x, y, and theta
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  // Creates normal distributions for x, y and theta.
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  // Create the vector to contain the `num_particles` particles
  particles = vector<Particle>(num_particles);

  // Create the vector to contain the weight for each particle
  weights = vector<double>(num_particles);

  // Loop over the particle vector and initialize each particle with the initial (x,y,theta) position
  // passed in the arguments with added Gaussian noise and weight 1
  for (int i = 0; i < num_particles; i++) {

    particles[i].id = i; // Set this particle's ID
    particles[i].x = dist_x(gen); // Generate a random value from `dist_x` to set as this particle's x position
    particles[i].y = dist_y(gen); // Generate a random value from `dist_y` to set as this particle's y position
    particles[i].theta = dist_theta(gen); // Generate a random value from `dist_theta` to set as this particle's orientation theta
    particles[i].weight = 1.0; // Set the initial weight for all particles to 1

  }
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  // Declare the random generator
  default_random_engine gen;

  // Extract the standard deviations for x, y, and theta
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  // Creates normal distributions for x, y and theta.
  normal_distribution<double> dist_x(0, std_x);
  normal_distribution<double> dist_y(0, std_y);
  normal_distribution<double> dist_theta(0, std_theta);

  // Since the formulae for the CTRV motion model to move each particle depend on the yaw_rate,
  // check the yaw rate first and then choose the appropriate set of formulae
  if (fabs(yaw_rate) > 0.001) {

    // Loop over all particles and predict the new position and orientation for each particle
    for (int i = 0; i < num_particles; i++) {

      // Cache the following computations because we need them multiple times
      double curvature = velocity / yaw_rate;
      double delta_yaw = yaw_rate * delta_t;

      // Apply the CTRV motion model and add Gaussian noise to each predicted coordinate
      particles[i].x += curvature * (sin(particles[i].theta + delta_yaw) - sin(particles[i].theta)) + dist_x(gen);
      particles[i].y += curvature * (cos(particles[i].theta) - cos(particles[i].theta + delta_yaw)) + dist_y(gen);
      particles[i].theta += delta_yaw + dist_theta(gen);

    }
  } else { // If the `yaw_rate` is zero or close to zero

    // Loop over all particles and predict the new position and orientation for each particle
    for (int i = 0; i < num_particles; i++) {

      double distance = velocity * delta_t; // Cache this because we need it twice

      // Apply the CTRV motion model and add Gaussian noise to each predicted coordinate
      particles[i].x += distance * cos(particles[i].theta) + dist_x(gen);
      particles[i].y += distance * sin(particles[i].theta) + dist_y(gen);
      particles[i].theta += dist_theta(gen);

    }
  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations, Particle &particle) {

  std::vector<int> associated_lms = std::vector<int>(observations.size()); // Store associated landmark IDs here
  std::vector<double> associated_xs = std::vector<double>(observations.size()); // Store x-coordinates of associated landmarks here
  std::vector<double> associated_ys = std::vector<double>(observations.size()); // Store y-coordinates of associated landmarks here

  // For each measured object, find the closest landmark in the map and match it
  for (int i = 0; i < observations.size(); i++) { // Loop over the transformed observations gathered from the LIDAR

    double min_distance = std::numeric_limits<double>::infinity(); // Store the current smallest distance between this observation and any landmark here
    // Store the current best match here
    int match_id;
    double match_x;
    double match_y;

    for (int j = 0; j < predicted.size(); j++) { // Loop over all landmarks in `predicted`

      double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

      if (distance < min_distance) {
        min_distance = distance;
        match_id = predicted[j].id;
        match_x = predicted[j].x;
        match_y = predicted[j].y;
      }
    }

    associated_lms[i] = match_id;
    associated_xs[i] = match_x;
    associated_ys[i] = match_y;
  }

  particle.associations = associated_lms;
  particle.sense_x = associated_xs;
  particle.sense_y = associated_ys;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {

  for (int i = 0; i < particles.size(); i++) { // Loop over all particles

    /*****************************************************************************************
     * 1: Get all map landmarks within the sensor range
     *****************************************************************************************/

    std::vector<LandmarkObs> lms_in_range;

    for (int l = 0; l < map_landmarks.landmark_list.size(); l++) {

      // Compute the distance between this landmark and this particle
      double distance = dist(map_landmarks.landmark_list[l].x_f, map_landmarks.landmark_list[l].y_f, particles[i].x, particles[i].y);
      // If the distance is within this particle's sensor range, include this landmark
      if (distance <= sensor_range) {

        LandmarkObs lm {map_landmarks.landmark_list[l].id_i, map_landmarks.landmark_list[l].x_f, map_landmarks.landmark_list[l].y_f}; // Convert this map landmark into a LandmarkObs object
        lms_in_range.push_back(lm);

      }
    }

    // If all landmarks are out of range, then the weight of this particle will be zero and we can skip to the next loop
    if (lms_in_range.size() == 0) {

      particles[i].weight = 0;
      weights[i] = 0;

    } else {

      /*****************************************************************************************
       * 2: Transform the sensor measurements into global map coordinates from the perspective of this particle
       *****************************************************************************************/

      // Create a vector for the transformed observations for this particle
      std::vector<LandmarkObs> transformed_observations = std::vector<LandmarkObs>(observations.size());

      double cos_theta = cos(particles[i].theta);
      double sin_theta = sin(particles[i].theta);

      for (int j = 0; j < observations.size(); j++) { // Loop over all observations

        // Transform vehicle coordinates of this observation into map coordinates from the perspective of this particle
        transformed_observations[j].id = observations[j].id;
        transformed_observations[j].x = observations[j].x * cos_theta - observations[j].y * sin_theta + particles[i].x;
        transformed_observations[j].y = observations[j].x * sin_theta + observations[j].y * cos_theta + particles[i].y;

      }

      /*****************************************************************************************
       * 3: Associate each observation with the nearest neighbor landmark on the map
       *****************************************************************************************/

      dataAssociation(lms_in_range, transformed_observations, particles[i]);

      /*****************************************************************************************
       * 4: Compute this particle's new weight
       *****************************************************************************************/

      double weight_i = 1;

      for (int j = 0; j < transformed_observations.size(); j++) { // Loop over all observations

        double x = transformed_observations[j].x; // The x-coordinate of this observation
        double y = transformed_observations[j].y; // The y-coordinate of this observation
        double mu_x = particles[i].sense_x[j]; // The x-coordinate of the associated landmark
        double mu_y = particles[i].sense_y[j]; // The y-coordinate of the associated landmark
        double sigma_x = std_landmark[0]; // The measurement uncertainty in the x-coordinate
        double sigma_y = std_landmark[1]; // The measurement uncertainty in the y-coordinate

        weight_i *= gaussian_2d(x, y, mu_x, mu_y, sigma_x, sigma_y);

      }

      particles[i].weight = weight_i;
      weights[i] = weight_i;

    }
  }
}

void ParticleFilter::resample() {

  std::vector<Particle> new_particles(num_particles);

  std::default_random_engine generator;
  // Create a discrete distribution according to the current weights vector
  std::discrete_distribution<int> distribution(weights.begin(), weights.end());

  for (int i = 0; i < num_particles; i++) { // Draw from this distribution `num_particle` times to sample the new particles list

    new_particles[i] = particles[distribution(generator)];

  }

  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y) {
	// particle: The particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: The associations x mapping already converted to world coordinates
	// sense_y: The associations y mapping already converted to world coordinates

	// Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best) {
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // Get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best) {
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // Get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best) {
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // Get rid of the trailing space
    return s;
}
