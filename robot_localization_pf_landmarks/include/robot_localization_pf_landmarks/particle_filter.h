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

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <alice_msgs/FoundObject.h>
#include <alice_msgs/FoundObjectArray.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void RePositionCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
void bodydeltaCallback(const geometry_msgs::Twist::ConstPtr& msg);
void landmarkCallback(const alice_msgs::FoundObjectArray::ConstPtr& msg);

struct LandmarkObs {
	
	int id;		// Id of matching landmark in the map.
	double x;	// Local (vehicle coordinates) x position of landmark observation [m]
	double y;	// Local (vehicle coordinates) y position of landmark observation [m]
};

struct Particle{
	int id;
	double x;
	double y;
	double theta;
	double weight;
};

class ParticleFilter{

	public:

		int num_particles;
		bool is_initialized;
		std::vector<double> weights;
		std::vector<Particle> particles;

		ParticleFilter():num_particles(0), is_initialized(false){}

		~ParticleFilter(){}

		float getRandom(float low, float high);
		float getRandomGaussian(float mean, float sigma);

		void init_circle(double x, double y, double theta, double std[]);
		void init_square(float x, float y, float theta);

		void prediction(float x, float y, float theta);

		void updateWeights(double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks);
		
		const bool initialized() const {
			return is_initialized;
		}
};

#endif /* PARTICLE_FILTER_H_ */