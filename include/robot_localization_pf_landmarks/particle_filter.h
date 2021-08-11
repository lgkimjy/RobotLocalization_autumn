/*
 * particle_filter.h
 *      Author: junyoung kim / lgkimjy
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"

#include <iostream>
#include <ctime>
#include <iomanip>
#include <random>
#include <algorithm>
#include <numeric>
#include <random>

#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <ros/package.h>

using namespace std;
using namespace cv;

struct Particle{
	int id;
	double x;
	double y;
	double theta;
	double weight;
};

class ParticleFilter{

	public:

		// Number of particles to draw
		int num_particles;
		// Flag, if filter is initialized
		bool is_initialized;
		// Vector of weights of all particles
		std::vector<double> weights;
		// Set of current particles
		std::vector<Particle> particles;

		// Constructor
		ParticleFilter() : num_particles(0), is_initialized(false) {}
		// Destructor
		~ParticleFilter() {}

		float getRandom(float low, float high);
		float getRandomGaussian(float mean, float sigma);

		void init_circle(double x, double y, double theta, double std[]);
		void init_square(float x, float y, float theta);

		void prediction(float x, float y, float theta);
		
		void resample();
		
		const bool initialized() const {
			return is_initialized;
		}
};

#endif /* PARTICLE_FILTER_H_ */