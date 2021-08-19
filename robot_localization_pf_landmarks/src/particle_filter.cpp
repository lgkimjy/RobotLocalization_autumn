/*
 * particle_filter.cpp
 *      Author: junyoung kim / lgkimjy
 */
#include <robot_localization_pf_landmarks/particle_filter.h>

/* declare a random engine to be used across multiple and various method calls */
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
    normal_distribution<float> distribution(mean, sigma);
    return distribution(rd);
}

double ParticleFilter::multivGaussian(double sig_x, double sig_y, double x_obs, double y_obs, double mu_x, double mu_y)
{
    double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);
    double exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2))) + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));
    double weight = gauss_norm * exp(-exponent);
    return weight;
}

void ParticleFilter::initSquare(float x, float y, float theta)
{
    num_particles = 1000;

    float repos_range_x[2] = {x - 0.5, x + 0.5};
    float repos_range_y[2] = {y - 0.5, y + 0.5};
    float repos_range_w[2] = {theta - 3.14 / 2, theta + 3.14 / 2};

    /* init particles */
    for (int i = 0; i < num_particles; i++)
    {
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

void ParticleFilter::initCircle(double x, double y, double theta, double std[])
{
    num_particles = 1000;

    /* define normal distributions for sensor noise */
    normal_distribution<double> N_x_init(0, std[0]);
    normal_distribution<double> N_y_init(0, std[1]);
    normal_distribution<double> N_theta_init(0, std[2]);

    /* init particles */
    for (int i = 0; i < num_particles; i++)
    {
        Particle p;
        p.id = i;
        p.x = x;
        p.y = y;
        p.theta = theta;
        p.weight = 1.0;

        /* add noise */
        p.x += N_x_init(gen);
        p.y += N_y_init(gen);
        // p.theta += N_theta_init(gen);
        p.theta += getRandom(theta-0.8, theta+0.8);

        particles.push_back(p);
    }

    is_initialized = true;
}

void ParticleFilter::noisyMove(float move_x, float move_y, float move_w)
{
    for (int i = 0; i < num_particles; i++)
    {
        float noise_x = move_x + getRandomGaussian(0.0, move_x * 0.1);
        float noise_y = move_y + getRandomGaussian(0.0, move_y * 0.1);
        float noise_w = move_w + getRandomGaussian(0.0, move_w * 5.0);
        
        /* w/ movement noise */
        particles[i].x += cos(particles[i].theta) * noise_x + sin(particles[i].theta) * noise_y;
        particles[i].y += sin(particles[i].theta) * noise_x + cos(particles[i].theta) * noise_y;
        particles[i].theta += noise_w;
        particles[i].theta = fmodf(particles[i].theta + M_PI * 2.0, M_PI * 2.0);
    }
}

void ParticleFilter::Move(float move_x, float move_y, float move_w)
{
    for (int i = 0; i < num_particles; i++)
    {
        /* w/o movement noise */
        particles[i].x += (cos(particles[i].theta) * move_x + sin(particles[i].theta) * move_y);
        particles[i].y += (sin(particles[i].theta) * move_x + cos(particles[i].theta) * move_y);
        particles[i].theta += move_w;
        particles[i].theta = fmodf(particles[i].theta + M_PI * 2.0, M_PI*2.0);
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> landmarks_ref, vector<LandmarkObs>& observations)
{
    for(int i = 0; i < observations.size(); i++)
    {
        double min_dist = 100.0;
        int map_id = -1;

        for(int j=0; j < landmarks_ref.size(); j++)
        {
            double cur_dist = dist(observations[i].x, observations[i].y, landmarks_ref[j].x, landmarks_ref[j].y);

            if(cur_dist < min_dist){
                min_dist = cur_dist;
                map_id = landmarks_ref[j].id;
            }
        }
        observations[i].id = map_id;
    }
}

void ParticleFilter::updateWeights(double std_landmark[], vector<LandmarkObs>& observations, Map map_landmarks)
{
    for (int i = 0; i < num_particles; i++)
    {
        /* get the particle x, y coordinates */
        double p_x = particles[i].x;
        double p_y = particles[i].y;
        double p_theta = particles[i].theta;

        vector<LandmarkObs> landmarks_ref;
        for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++)
        {
            /* get landmark id(empty), x,y coordinates */
            float lm_x = map_landmarks.landmark_list[j].x_f;
            float lm_y = map_landmarks.landmark_list[j].y_f;
            int lm_id = map_landmarks.landmark_list[j].id_i;

            landmarks_ref.push_back(LandmarkObs{lm_id, lm_x, lm_y});
        }

        /* multiply rotation matrix to each particles and find out new global position of the observed landmarks */
        vector<LandmarkObs> transformed_obs;
        for(unsigned int j = 0; j < observations.size(); j++)
        {
            double t_x = cos(p_theta) * observations[j].x - sin(p_theta) * observations[j].y + p_x;
            double t_y = sin(p_theta) * observations[j].x + cos(p_theta) * observations[j].y + p_y;
            transformed_obs.push_back(LandmarkObs{observations[j].id, t_x, t_y});
        }

		/* nearest neighbor matching algorithm = observed landmark id matching, fill the matched map id */
        dataAssociation(landmarks_ref, transformed_obs);

        /* re-initalize particle weight */
        particles[i].weight = 1.0;

        /* calculate each particles weight value */
        for(unsigned int j=0; j < transformed_obs.size(); j++)
        {
            double obs_x, obs_y, ref_x, ref_y;
            obs_x = transformed_obs[j].x;
            obs_y = transformed_obs[j].y;

            int associated_landmark = transformed_obs[j].id;
            observations[j].id = associated_landmark;

            for(unsigned int k=0; k < landmarks_ref.size();k++){
                if(landmarks_ref[k].id == associated_landmark){
                    ref_x = landmarks_ref[k].x;
                    ref_y = landmarks_ref[k].y;
                }
            }
            double each_multi_gauss_w = multivGaussian(std_landmark[0], std_landmark[1], ref_x, ref_y, obs_x, obs_y);
            particles[i].weight *= each_multi_gauss_w;
        }
    }
}

void ParticleFilter::resampling()
{
    vector<Particle> new_particles;

    // get all of the current weights
    vector<double> weights;
    for(int i = 0; i < num_particles; i++){
        weights.push_back(particles[i].weight);
    }

    // generate random starting index for resampling wheel
    uniform_int_distribution<int> uniintdist(0, num_particles - 1);
    auto index = uniintdist(gen);

    // get max weight
    double max_weight = *max_element(weights.begin(), weights.end());

    // uniform random distribution [0.0, max_weight)
    uniform_real_distribution<double> unirealdist(0.0, max_weight);

    double beta = 0.0;

    // spin the resample wheel!
    for(int i = 0; i < num_particles; i++)
    {
        beta += unirealdist(gen) * 2.0;
        while (beta > weights[index]){
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);
    }

    particles = new_particles;
}