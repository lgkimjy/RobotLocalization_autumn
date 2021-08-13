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
vector<LandmarkObs> observations;
bool flag_move;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_localization_pf_landmark");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Subscriber reposition_sub = nh.subscribe("/alice/reposition", 10, RePositionCallback);
    ros::Subscriber sub_referencbody = nh.subscribe("/alice/ideal_body_delta", 10, bodydeltaCallback);
    ros::Subscriber sub_landmark = nh.subscribe("/alice/vision/detected_objects", 10, landmarkCallback);

    ros::Publisher robotpos_pub = nh.advertise<geometry_msgs::Pose2D>("/alice/robot_pos", 10);

    namedWindow("localization image", WINDOW_NORMAL);

    /* noise generation */
    double sigma_pos[3] = {0.25, 0.25, 1.57}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
    double sigma_landmark[2] = {0.3, 0.3};    // Landmark measurement uncertainty [x [m], y [m]]

    /* Read map data */
    Map map;
    if(!read_map_data(ros::package::getPath("robot_localization_data") + "/data/map_data.txt", map)){
        ROS_ERROR("[robot_localization_pf_landmark] Error: Could not open map file");
        return -1;
    }else ROS_INFO("[robot_localization_pf_landmark] Success: Open and Read the map file");
    /* number of landmarks */
    ROS_INFO("[robot_localization_pf_landmark] Total reference landmarks (global coordinates) : %zu", map.landmark_list.size());

    while(ros::ok())
    {
        geometry_msgs::Pose2D robot_pos_msg;

        if(!pf.initialized())
        {
            pf.particles.clear();
            ROS_INFO("[robot_localization_pf_landmark] Initalize robot start position and particles random start position");
            // pf.initCircle(repos_x, repos_y, repos_w, sigma_pos);
            pf.initSquare(repos_x, repos_y, repos_w);

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
            sum_theta = fmodf(sum_theta + M_PI * 2.0, M_PI * 2.0);

            flag_move = false;
        }

        /* observed landmarks global coordinates based on kinematics_odom*/
        // for(auto a : observations)
        // {
        //     double t_x = cos(sum_theta) * a.x - sin(sum_theta) * a.y + sum_x;
        //     double t_y = sin(sum_theta) * a.x + cos(sum_theta) * a.y + sum_y;
        //     cout << t_x << " " << t_y << "    ";
        // }
        // cout << endl;


        /* Update the weights and resample */
        pf.updateWeights(sigma_landmark, observations, map);
        // pf.resampling();


        robot_pos_msg.x = sum_x - 5.2;
        robot_pos_msg.y = sum_y - 3.7;
        robot_pos_msg.theta = sum_theta;

        // ROS_INFO("Robot Kinematics : %3.2f, %3.2f, %2.3f", sum_x, sum_y, sum_theta);                                   // real world
        // ROS_INFO("Robot Kinematics : %3.2f, %3.2f, %2.3f", robot_pos_msg.x, robot_pos_msg.y, robot_pos_msg.theta);     // global world

        robotpos_pub.publish(robot_pos_msg);

        /* visualize */
        /* visualize landmark based on kinematic calculated pos, kinematics_odom */
        Mat localization_img = color_map_image.clone();
        for(int i = 0; i < observations.size(); i++){
            double t_x = cos(sum_theta) * observations[i].x - sin(sum_theta) * observations[i].y + sum_x;
            double t_y = sin(sum_theta) * observations[i].x + cos(sum_theta) * observations[i].y + sum_y;
            Point2f ld_point = Point2f(t_x * 100, localization_img.size().height - t_y * 100);
            circle(localization_img, ld_point, 10, Scalar(0, 0, 255), -1);
        }
        /* visualize particle pos */
        for(int i = 0; i < pf.particles.size(); i++){
            // ROS_INFO("Particle Pos : %3.2f, %3.2f, %2.3f", pf.particles[i].x, pf.particles[i].y, pf.particles[i].theta);              // real world
            // ROS_INFO("Particle Pos : %3.2f, %3.2f, %2.3f", pf.particles[i].x - 5.2, pf.particles[i].y - 3.7, pf.particles[i].theta);  // gloabl world
            Point2f point = Point2f(pf.particles[i].x * 100, localization_img.size().height - pf.particles[i].y * 100);
            circle(localization_img, point, 10, Scalar(0, 255, 255), -1);
            arrowedLine(localization_img, point, Point2f(point.x + 10 * cos(pf.particles[i].theta), point.y + 10 * -sin(pf.particles[i].theta)), Scalar(0, 0, 255), 5);
        }
        /* visualize robot pos */
        Point2f robot_point = Point2f(robot_pos_msg.x * 100 + localization_img.size().width / 2, localization_img.size().height - robot_pos_msg.y * 100 - localization_img.size().height / 2);
        circle(localization_img, robot_point, 10, Scalar(255, 0, 0), -1);
        arrowedLine(localization_img, robot_point, Point2f(robot_point.x + 10 * cos(robot_pos_msg.theta), robot_point.y + 10 * -sin(robot_pos_msg.theta)), Scalar(255, 255, 0), 5);

        imshow("localization image", localization_img);
        waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void RePositionCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    repos_x = (msg->x);
    repos_y = (msg->y);
    repos_w = (msg->theta);

    pf.is_initialized = false;
}

void bodydeltaCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    delta_x = (msg->linear).x;
    delta_y = (msg->linear).y;
    delta_w = (msg->angular).z;

    flag_move = true;
}

void landmarkCallback(const alice_msgs::FoundObjectArray::ConstPtr &msg)
{
    observations.clear();
    LandmarkObs obs;
    for(int i = 0; i < msg->length; i++){
        if(msg->data[i].name == "cpoint" || msg->data[i].name == "lpoint" || msg->data[i].name == "tpoint" || msg->data[i].name == "goalpost"){
            obs.x = msg->data[i].pos.x;
            obs.y = msg->data[i].pos.y;
            observations.push_back(obs);
        }
        else
            continue;
    }
    /* checker */
    // cout << "size : " << observations.size() << "   ";
    // for(const auto& a : observations){
    //     cout << a.x << ", " << a.y << "    ";
    // }
    // cout << endl;
}