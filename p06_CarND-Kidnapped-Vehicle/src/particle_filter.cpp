/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles
  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; ++i){
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;
    particles.push_back(p);
  }
  is_initialized = true;
  //std::cout << "initialized" << std::endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
   std::cout << "prediction start" << std::endl;
   std::default_random_engine gen;
   std::normal_distribution<double> dist_x(0, std_pos[0]);
   std::normal_distribution<double> dist_y(0, std_pos[1]);
   std::normal_distribution<double> dist_theta(0, std_pos[2]);

   for (int i = 0; i < num_particles; ++i){
     Particle p;
     p = particles[i];
     p.x += velocity/yaw_rate*(sin(p.theta + yaw_rate*delta_t) - sin(p.theta)) + dist_x(gen);
     p.y += velocity/yaw_rate*(cos(p.theta) - cos(p.theta + yaw_rate*delta_t)) + dist_y(gen);
     p.theta += yaw_rate*delta_t + dist_theta(gen);
     particles[i] = p;
   }
   std::cout << "prediction end" << std::endl;
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */
   std::cout << "data association start" << std::endl;
   for (unsigned int i = 0; i < observations.size(); i++){
     int min_id = predicted[0].id;
     double min_dist = dist(observations[i].x, observations[i].y, predicted[0].x, predicted[0].y);
     for (unsigned int j = 1; j < predicted.size(); j++){
       double cur_dist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
       if(cur_dist < min_dist){
         min_id = predicted[j].id;
         min_dist = cur_dist;
       }
     }
     observations[i].id = min_id;
   }
   std::cout << "data association end" << std::endl;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
   std::cout << "updateWeight start" << std::endl;
   double coef = 1.0/(2.0*M_PI*std_landmark[0]*std_landmark[1]);
   double wei = 1.0;
   for (int i = 0; i < num_particles; ++i){
     double c_yaw = cos(particles[i].theta);
     double s_yaw = sin(particles[i].theta);
     for (unsigned int j = 0; j < observations.size(); ++j){
       //transformations
       double xm = particles[i].x + observations[j].x * c_yaw - observations[j].y * s_yaw;
       double ym = particles[i].y + observations[j].x * s_yaw + observations[j].y * c_yaw;

       //
       int id = observations[j].id;
       int landm = 0;
       for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); ++k){
         if(map_landmarks.landmark_list[k].id_i == id){
           landm = k;
         }
       }
       double wei_x = exp(-0.5*pow((xm - map_landmarks.landmark_list[landm].x_f)/std_landmark[0],2));
       double wei_y = exp(-0.5*pow((ym - map_landmarks.landmark_list[landm].y_f)/std_landmark[1],2));
       wei *= (coef*wei_x*wei_y);
     }
     particles[i].weight = wei;
   }
   std::cout << "data update weight end" << std::endl;
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
   std::cout << "resample start" << std::endl;
   double max_weight = particles[0].weight;
   weights.push_back(particles[0].weight);
   for (int i=1; i<num_particles; ++i){
     weights.push_back(particles[i].weight);
     if(particles[i].weight > max_weight){
       max_weight = particles[i].weight;
     }
   }

   std::vector<Particle> resampled_particles;
   std::default_random_engine gen;
   std::uniform_int_distribution<> dist_uni_int(0, num_particles-1);
   std::uniform_real_distribution<> dist_uni_real(0, 2.0*max_weight);
   int index = dist_uni_int(gen);
   double beta = 0;
   for (int i = 0; i<num_particles; i++){
     beta += dist_uni_real(gen);
     while(weights[index] < beta){
       beta = beta - weights[index];
       index = (index + 1)%num_particles;
     }
     resampled_particles.push_back(particles[index]);
   }
   particles = resampled_particles;
   std::cout << "resample end" << std::endl;
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
