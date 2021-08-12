/*
 * particle_filter.cpp
 *      Author: junyoung kim / lgkimjy
 */
#include <robot_localization_pf_landmarks/particle_filter.h>

// declare a random engine to be used across multiple and various method calls
static default_random_engine gen;

float ParticleFilter::getRandom(float low, float high)
{
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<float> dis(low, high);
    return dis(gen);
}

float ParticleFilter::getRandomGaussian(float mean, float sigma)
{
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<float> distribution(mean,sigma);
    return distribution(rd);
}

void ParticleFilter::init_square(float x, float y, float theta) {

  num_particles = 1;

  float repos_range_x[2] = {x - 2 / 2, x + 2 / 2};
  float repos_range_y[2] = {y - 2 / 2, y + 2 / 2};
  float repos_range_w[2] = {theta - 3.14/2, theta + 3.14/2};

  // init particles 
  for (int i = 0; i < num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = getRandom(repos_range_x[0], repos_range_x[1]);
    p.y = getRandom(repos_range_y[0], repos_range_y[1]);
    p.theta = getRandom(repos_range_w[0], repos_range_w[1]);
    p.weight = 1.0;

    particles.push_back(p);
  }

  is_initialized = true;
}

void ParticleFilter::init_circle(double x, double y, double theta, double std[]) {

  num_particles = 500;

  // define normal distributions for sensor noise
  normal_distribution<double> N_x_init(0, std[0]);
  normal_distribution<double> N_y_init(0, std[1]);
  normal_distribution<double> N_theta_init(0, std[2]);

  // init particles 
  for (int i = 0; i < num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = x;
    p.y = y;
    p.theta = theta;
    p.weight = 1.0;

    // add noise
    p.x += N_x_init(gen);
    p.y += N_y_init(gen);
    p.theta += N_theta_init(gen);

    particles.push_back(p);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(float move_x, float move_y, float move_w){

  for (int i = 0; i < num_particles; i++) {
  
    float noise_x = move_x + getRandomGaussian(0.0, move_x * 0.1);
    float noise_y = move_y + getRandomGaussian(0.0, move_y * 0.1);
    float noise_w = move_w + getRandomGaussian(0.0, move_w * 0.1);

    particles[i].x += cos(particles[i].theta) * noise_x + sin(particles[i].theta) * noise_y;
    particles[i].y += sin(particles[i].theta) * noise_x + cos(particles[i].theta) * noise_y;
    particles[i].theta += noise_w;
    particles[i].theta = fmodf(particles[i].theta + M_PI * 2.0, M_PI*2.0);

    /* w/o movement noise */
    // particles[i].x += (cos(particles[i].theta) * move_x + sin(particles[i].theta) * move_y);
    // particles[i].y += (sin(particles[i].theta) * move_x + cos(particles[i].theta) * move_y);
    // particles[i].theta += move_w;
    // particles[i].theta = fmodf(particles[i].theta + M_PI * 2.0, M_PI*2.0);
  }
}

void ParticleFilter::resample(){
  
  vector<Particle> new_particles;

  // get all of the current weights
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  // generate random starting index for resampling wheel
  uniform_int_distribution<int> uniintdist(0, num_particles-1);
  auto index = uniintdist(gen);

  // get max weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> unirealdist(0.0, max_weight);

  double beta = 0.0;

  // spin the resample wheel!
  for (int i = 0; i < num_particles; i++) {
    beta += unirealdist(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }

  particles = new_particles;
}
