/*
 * robot_localization_pf_landmarks.cpp
 *      Author: junyoung kim / lgkimjy
 */
#include <robot_localization_pf_landmarks/particle_filter.h>

float repos_x = 0, repos_y = 0, repos_w = 0;
float delta_x = 0, delta_y = 0, delta_w = 0;

void RePositionCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
void bodydeltaCallback(const geometry_msgs::Twist::ConstPtr& msg);

string color_image_map_path = ros::package::getPath("robot_localization_data") + "/map/soccer_field.jpg";
Mat color_map_image = imread(color_image_map_path.c_str());

ParticleFilter pf;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_localization_pf_landmark");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Subscriber reposition_sub = nh.subscribe("/alice/reposition", 10, RePositionCallback);
    ros::Subscriber sub_referencbody = nh.subscribe("/alice/ideal_body_delta", 10, bodydeltaCallback);

    ros::Publisher robotpos_pub = nh.advertise<geometry_msgs::Pose2D>("/alice/robot_pos", 10);
    
    namedWindow("localization image", WINDOW_NORMAL);

    double sigma_pos[3] = {0.25, 0.25, 1.57}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]

	// noise generation
	default_random_engine gen;
	normal_distribution<double> N_x_init(-sigma_pos[0], sigma_pos[0]);
	normal_distribution<double> N_y_init(-sigma_pos[1], sigma_pos[1]);
	normal_distribution<double> N_theta_init(0, sigma_pos[2]);
	double n_x, n_y, n_theta, n_range, n_heading;

    while(ros::ok())
    {
		if(!pf.initialized())
        {
            pf.particles.clear();
            cout << "initalize particles starting position" << endl;
            // pf.init_circle(repos_x, repos_y, repos_w, sigma_pos);
            pf.init_square(repos_x, repos_y, repos_w);
		}
        else
        {
			// pf.prediction(delta_t, sigma_pos, position_meas[i-1].velocity, position_meas[i-1].yawrate);
            // pf.prediction(delta_t, sigma_pos, position_meas[i-1].velocity, position_meas[i-1].yawrate);
            pf.prediction(delta_x, delta_y, delta_w);
        }

		// // Update the weights and resample
		// pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
		// pf.resample();
		
		// // Calculate and output the average weighted error of the particle filter over all time steps so far.
		// vector<Particle> particles = pf.particles;
		// int num_particles = particles.size();
		// double highest_weight = 0.0;
		// Particle best_particle;
		// for (int i = 0; i < num_particles; ++i) {
		// 	if (particles[i].weight > highest_weight) {
		// 		highest_weight = particles[i].weight;
		// 		best_particle = particles[i];
		// 	}
		// }

        Mat localization_img = color_map_image.clone();
        for(int i=0; i<pf.particles.size(); i++)
        {
            Point2f point = Point2f(pf.particles[i].x * 100, pf.particles[i].y * 100);
            circle(localization_img, point, 10, cv::Scalar(0,255,255), -1);
            arrowedLine(localization_img, point, Point2f(point.x + 10 * cos(-pf.particles[i].theta), point.y + 10 * sin(-pf.particles[i].theta)), Scalar(0,0,255), 5);
        }
        // Point2f robot_point = Point2f(520.0, 370.0);
        // circle(localization_img, robot_point, 10, Scalar(255,0,0), -1);
        // arrowedLine(localization_img, robot_point, Point2f(robot_point.x + 10 * cos(-robot_position.z), robot_point.y + 10 * sin(-robot_position.z)), Scalar(255,255,0), 5);

        imshow("localization image", localization_img);
        waitKey(1);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void RePositionCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    repos_x = (msg->x);
    repos_y = (msg->y);
    repos_w = (msg->theta);

    pf.is_initialized = false;
}

void bodydeltaCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    delta_x = (msg->linear).x;
    delta_y = (msg->linear).y;
    delta_w = (msg->angular).z;
}