/*
 * robot_localization_pf_landmarks.cpp
 *      Author: junyoung kim / lgkimjy
 */
#include <robot_localization_pf_landmarks/particle_filter.h>

float repos_x = 0, repos_y = 0, repos_w = 0;
float delta_x = 0, delta_y = 0, delta_w = 0;
float sum_theta = 0, sum_x = 0, sum_y = 0;

string color_image_map_path = ros::package::getPath("robot_localization_data") + "/map/soccer_field.jpg";
Mat color_map_image = imread(color_image_map_path.c_str());

ParticleFilter pf;
bool flag_move;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_localization_pf_landmark");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Subscriber reposition_sub = nh.subscribe("/alice/reposition", 10, RePositionCallback);
    ros::Subscriber sub_referencbody = nh.subscribe("/alice/ideal_body_delta", 10, bodydeltaCallback);
    ros::Subscriber sub_landmark = nh.subscribe("/alice/vision/detected_objects", 10, landmarkCallback);

    ros::Publisher robotpos_pub = nh.advertise<geometry_msgs::Pose2D>("/alice/robot_pos", 10);
    
    namedWindow("localization image", WINDOW_NORMAL);

	// noise generation
    double sigma_pos[3] = {0.25, 0.25, 1.57}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]

	// Read map data
	Map map;
	if (!read_map_data(ros::package::getPath("robot_localization_data") + "/data/map_data.txt", map)) {
        ROS_ERROR("Error: Could not open map file");
		return -1;
	}

    while(ros::ok())
    {
        geometry_msgs::Pose2D robot_pos_msg;

		if(!pf.initialized())
        {
            pf.particles.clear();
            cout << "initalize particles starting position" << endl;
            // pf.init_circle(repos_x, repos_y, repos_w, sigma_pos);
            pf.init_square(repos_x, repos_y, repos_w);

            /* reposition for kinematics only movement */
            sum_x = repos_x;
            sum_y = repos_y;
            sum_theta = repos_w;
		}
        if(flag_move == true)
        {
            /* particle movement */
            pf.prediction(delta_x, delta_y, delta_w);

            /* summation of alice ideal body delta */
            sum_x += (cos(sum_theta) * delta_x + sin(sum_theta) * delta_y);
            sum_y += (sin(sum_theta) * delta_x + cos(sum_theta) * delta_y);
            sum_theta += delta_w;
            sum_theta = fmodf(sum_theta + M_PI * 2.0, M_PI*2.0);
            
            flag_move = false;
        }

        /* number of landmarks */
        // cout << map.landmark_list.size() << endl;

		/* Update the weights and resample */
		// pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
		// pf.resample();
		
		/* Calculate and output the average weighted error of the particle filter over all time steps so far. */
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

        robot_pos_msg.x = sum_x - 5.2;
        robot_pos_msg.y = sum_y - 3.7;
        robot_pos_msg.theta = sum_theta;

        // ROS_INFO("Robot Kinematics : %3.2f, %3.2f, %2.3f", sum_x, sum_y, sum_theta);                                   // real world
        // ROS_INFO("Robot Kinematics : %3.2f, %3.2f, %2.3f", robot_pos_msg.x, robot_pos_msg.y, robot_pos_msg.theta);     // global world

        robotpos_pub.publish(robot_pos_msg);

        Mat localization_img = color_map_image.clone();
        for(int i=0; i<pf.particles.size(); i++)
        {
            // ROS_INFO("Particle Pos : %3.2f, %3.2f, %2.3f", pf.particles[i].x, pf.particles[i].y, pf.particles[i].theta);              // real world 
            // ROS_INFO("Particle Pos : %3.2f, %3.2f, %2.3f", pf.particles[i].x - 5.2, pf.particles[i].y - 3.7, pf.particles[i].theta);  // gloabl world
            Point2f point = Point2f(pf.particles[i].x * 100, localization_img.size().height - pf.particles[i].y * 100);
            circle(localization_img, point, 10, cv::Scalar(0,255,255), -1);
            arrowedLine(localization_img, point, Point2f(point.x + 10 * cos(pf.particles[i].theta), point.y + 10 * -sin(pf.particles[i].theta)), Scalar(0,0,255), 5);
        }

        Point2f robot_point = Point2f(robot_pos_msg.x*100 + localization_img.size().width/2, localization_img.size().height - robot_pos_msg.y*100 - localization_img.size().height/2);
        circle(localization_img, robot_point, 10, Scalar(255,0,0), -1);
        arrowedLine(localization_img, robot_point, Point2f(robot_point.x + 10 * cos(robot_pos_msg.theta), robot_point.y + 10 * -sin(robot_pos_msg.theta)), Scalar(255,255,0), 5);

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

    flag_move = true;
}

void landmarkCallback(const alice_msgs::FoundObjectArray::ConstPtr& msg)
{
    vector<LandmarkObs> observations;
    LandmarkObs obs;
    for(int i=0;i<msg->length;i++){
        if(msg->data[i].name == "cpoint" || msg->data[i].name == "lpoint" || 
           msg->data[i].name == "tpoint" || msg->data[i].name == "goalpost"){
            obs.x = msg->data[i].pos.x;
            obs.y = msg->data[i].pos.y;
            observations.push_back(obs);
        }
        else
            continue;
    }
    for(auto a : observations){
        cout << a.x << ", " << a.y << "    ";
    }
    cout << endl;
}