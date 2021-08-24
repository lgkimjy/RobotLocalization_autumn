/*
 * robot_localization_pf_landmarks.cpp
 *      Author: junyoung kim / lgkimjy
 */
#include <robot_localization_pf_landmarks/particle_filter.h>


std::mutex m1;

float delta_x = 0, delta_y = 0, delta_w = 0;
float repos_x = 0, repos_y = 0, repos_w = 0;
float sum_x_prev = 0, sum_y_prev = 0, sum_theta_prev = 0;
float x_local = 0, y_local = 0, theta_local = 0;
float sum_theta = 0, sum_x = 0, sum_y = 0;
float sum_theta_ideal = 0, sum_x_ideal = 0, sum_y_ideal = 0;

/* map data */ 
string color_image_map_path = ros::package::getPath("robot_localization_data") + "/map/soccer_field.jpg";
Mat color_map_image = imread(color_image_map_path.c_str());
Mat log_image = color_map_image.clone();

/* particle and landmark vectors */
ParticleFilter pf;
Particle best_particle;
Particle best_particle_prev;
vector<LandmarkObs> observations;
LandmarkObs ball;
int obs_count = 0;

/* robot joint status */
float angle_torso_yaw = 0.0; // units : rad  
float angle_head_yaw = 0.0;  // units : rad
bool flag_move;

/* ros param */
string mode;
string gui_mode;

/* time data for logging localize result */
time_t curTime = time(NULL);
struct tm *pLocal = localtime(&curTime);
string path;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_localization_pf_landmark");
    ros::NodeHandle nh;
    ros::Rate loop_rate(5);

    nh.getParam("/mode", mode);
    nh.getParam("/gui_mode", gui_mode);

    ros::Subscriber reposition_sub = nh.subscribe("/alice/reposition", 1000, RePositionCallback);
    ros::Subscriber sub_referencbody = nh.subscribe("/alice/ideal_body_delta", 10, bodydeltaCallback);
    ros::Subscriber sub_landmark = nh.subscribe("/alice/vision/detected_objects", 10, landmarkCallback);
    ros::Subscriber sub_foot_center = nh.subscribe("/heroehs/alice_center_foot_pose", 1000, centerfootCallback);
    ros::Subscriber joint_sub = nh.subscribe("/robotis/present_joint_states",10,jointCallback);

    ros::Publisher robotpos_pub = nh.advertise<geometry_msgs::Pose2D>("/alice/robot_pos", 10);
    ros::Publisher localization_image_pub = nh.advertise<sensor_msgs::Image>("/robot_localization/localization_image", 10);

    #if (CV_MAJOR_VERSION <= 3) 
    {
        // for opencv 3.4.0 user
        namedWindow("localization image", CV_WINDOW_NORMAL);
        if (gui_mode == "on_alice"){
            cvSetWindowProperty("localization image", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
        }
    }
    #else
    {
        // for opencv 4.1.1 user
        namedWindow("localization image", CV_RAND_NORMAL);
    }
    #endif

    /* noise generation */
    double sigma_pos[3] = {0.1, 0.1, 1.57}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
    double sigma_landmark[2] = {0.3, 0.3};    // Landmark measurement uncertainty [x [m], y [m]]

    /* Read map data */
    Map map;
    if (!read_map_data(ros::package::getPath("robot_localization_data") + "/data/map_data.txt", map))
    {
        ROS_ERROR("[robot_localization_pf_landmark] Error: Could not open map file");
        return -1;
    }
    else
        ROS_INFO("[robot_localization_pf_landmark] Success: Open and Read the map file");
    /* number of landmarks */
    ROS_INFO("[robot_localization_pf_landmark] Total reference landmarks (global coordinates) : %zu", map.landmark_list.size());

    while (ros::ok())
    {
        geometry_msgs::Pose2D robot_pos_msg;

        if (!pf.initialized())
        {
            if (mode == "PF")
            {
                pf.particles.clear();
                ROS_INFO("[robot_localization_pf_landmark] Initalize robot start position and particles random start position");
                // pf.initCircle(repos_x, repos_y, repos_w, sigma_pos);
                pf.initSquare(repos_x, repos_y, repos_w);
                best_particle = Particle{0, repos_x, repos_y, repos_w, 1.0};
                best_particle_prev = Particle{0, repos_x, repos_y, repos_w, 1.0};
            }
            /* reposition for kinematics only movement */
            sum_x_ideal = repos_x;
            sum_y_ideal = repos_y;
            sum_theta_ideal = repos_w;

            sum_x = repos_x;
            sum_y = repos_y;
            sum_theta = repos_w;

            sum_x_prev = repos_x;
            sum_y_prev = repos_y;
            sum_theta_prev = repos_w;

            pf.is_initialized = true;
        }

        if (flag_move == true)
        {
            if (mode == "PF")
            {
                /* if keypoints are not detected more than 2, only use kinematics */
                if (observations.size() <= 2)
                {
                    /* ideal body delta */
                    // best_particle.x += (cos(best_particle.theta) * delta_x - sin(best_particle.theta) * delta_y);
                    // best_particle.y += (sin(best_particle.theta) * delta_x + cos(best_particle.theta) * delta_y);
                    // best_particle.theta += delta_w;
                    
                    /* foot center pose */
                    best_particle.x = best_particle_prev.x + (cos(best_particle_prev.theta) * x_local - sin(best_particle_prev.theta) * y_local);
                    best_particle.y = best_particle_prev.y + (sin(best_particle_prev.theta) * x_local + cos(best_particle_prev.theta) * y_local);
                    best_particle.theta = best_particle_prev.theta + theta_local;
                    pf.particles.clear();

                    obs_count++;
                }
                else
                {
                    /* observation detected after non-detection situation, it will re-initalize particle filter */
                    if (obs_count > 0)
                    {
                        pf.initCircle(best_particle.x, best_particle.y, best_particle.theta, sigma_pos);
                        // pf.initSquare(best_particle.x, best_particle.y, best_particle.theta);
                        ROS_INFO("[robot_localization_pf_landmark] Re-initalize particle filter baed on best particle pose");
                        obs_count = 0;
                    }
                    /* particle movement */
                    pf.noisyMove(delta_x, delta_y, delta_w); // ideal_body_delta

                    /* Update the weights and resample */
                    pf.updateWeights(sigma_landmark, observations, map);
                    pf.resampling();

                    /* Calculate and output the best particle weight */
                    vector<Particle> particles = pf.particles;
                    int num_particles = particles.size();
                    double highest_weight = 0.0;
                    for (int i = 0; i < num_particles; ++i){
                        if (particles[i].weight > highest_weight){
                            highest_weight = particles[i].weight;
                            best_particle = particles[i];
                        }
                    }
                    
                    /* head status apply to particle theta */
                    for (int i = 0; i < num_particles; ++i)
                    {
                        particles[i].theta -= angle_head_yaw;
                    }
                }
            }

            /* summation of alice ideal body delta */
            sum_x_ideal += (cos(sum_theta_ideal) * delta_x + sin(sum_theta_ideal) * delta_y);
            sum_y_ideal += (sin(sum_theta_ideal) * delta_x + cos(sum_theta_ideal) * delta_y);
            sum_theta_ideal += delta_w;

            /* summation of alice center foot */
            sum_x = sum_x_prev + (cos(sum_theta_prev) * x_local - sin(sum_theta_prev) * y_local);
            sum_y = sum_y_prev + (sin(sum_theta_prev) * x_local + cos(sum_theta_prev) * y_local);
            sum_theta = sum_theta_prev + theta_local;

            flag_move = false;
        }

        if (mode == "kinematics")
        {
            /* Kinematics position */
            robot_pos_msg.x = sum_x - 5.2;
            robot_pos_msg.y = sum_y - 3.7;
            robot_pos_msg.theta = fmodf(sum_theta + M_PI * 2.0, M_PI * 2.0);
            // ROS_INFO("[robot_localization_pf_landmark] Robot Kinematics : %3.2f, %3.2f, %2.3f", sum_x, sum_y, robot_pos_msg.theta);                     // real world
            // ROS_INFO("[robot_localization_pf_landmark] Robot Kinematics : %3.2f, %3.2f, %2.3f", robot_pos_msg.x, robot_pos_msg.y, robot_pos_msg.theta); // global world
        }
        else if (mode == "PF")
        {
            /* PF predicted position */
            robot_pos_msg.x = best_particle.x - 5.2;
            robot_pos_msg.y = best_particle.y - 3.7;
            robot_pos_msg.theta = fmodf(best_particle.theta + M_PI * 2.0, M_PI * 2.0);
            // ROS_INFO("[robot_localization_pf_landmark] Best Particle : %3.2f, %3.2f, %2.3f", best_particle.x, best_particle.y, robot_pos_msg.theta); // real world
            // ROS_INFO("[robot_localization_pf_landmark] Best Particle : %3.2f, %3.2f, %2.3f", robot_pos_msg.x, robot_pos_msg.y, robot_pos_msg.theta); // global world
        }

        /* ROS Msg Publisher */
        robotpos_pub.publish(robot_pos_msg);

        /* visualize */
        if (gui_mode == "on_alice" || gui_mode == "on" || gui_mode == "nuc")
        {
            Mat localization_img = color_map_image.clone();

            if (mode == "PF")
            {
                /* visualize landmark based on kinematic calculated pos, kinematics_odom */
                for (int i = 0; i < pf.particles.size(); i++)
                {
                    for (int j = 0; j < observations.size(); j++)
                    {
                        double t_x = cos(pf.particles[i].theta) * observations[j].x - sin(pf.particles[i].theta) * observations[j].y + pf.particles[i].x;
                        double t_y = sin(pf.particles[i].theta) * observations[j].x + cos(pf.particles[i].theta) * observations[j].y + pf.particles[i].y;
                        Point2f ld_point = Point2f(t_x * 100, localization_img.size().height - t_y * 100);
                        circle(localization_img, ld_point, 7, Scalar(0, 255, 255), -1);
                        putText(localization_img, to_string(observations[j].id), (ld_point), 1, 1.2, (255, 0, 0), 2, true);
                    }
                }
                double t_x = cos(best_particle.theta) * ball.x - sin(best_particle.theta) * ball.y + best_particle.x;
                double t_y = sin(best_particle.theta) * ball.x + cos(best_particle.theta) * ball.y + best_particle.y;
                Point2f ld_point = Point2f(t_x * 100, localization_img.size().height - t_y * 100);
                circle(localization_img, ld_point, 10, Scalar(100, 100, 255), -1);
            }
            else if (mode == "kinematics")
            {
                /* visualize landmark based on kinematic calculated pos, kinematics_odom */
                for (int i = 0; i < observations.size(); i++)
                {
                    double t_x = cos(sum_theta) * observations[i].x - sin(sum_theta) * observations[i].y + sum_x;
                    double t_y = sin(sum_theta) * observations[i].x + cos(sum_theta) * observations[i].y + sum_y;
                    Point2f ld_point = Point2f(t_x * 100, localization_img.size().height - t_y * 100);
                    circle(localization_img, ld_point, 7, Scalar(0, 0, 255), -1);
                }
                double t_x = cos(sum_theta) * ball.x - sin(sum_theta) * ball.y + sum_x;
                double t_y = sin(sum_theta) * ball.x + cos(sum_theta) * ball.y + sum_y;
                Point2f ld_point = Point2f(t_x * 100, localization_img.size().height - t_y * 100);
                circle(localization_img, ld_point, 10, Scalar(100, 100, 255), -1);
            }

            if (mode == "PF")
            {
                /* visualize particle pos */
                for (int i = 0; i < pf.particles.size(); i++)
                {
                    Point2f point = Point2f(pf.particles[i].x * 100, localization_img.size().height - pf.particles[i].y * 100);
                    circle(localization_img, point, 10, Scalar(0, 255, 255), -1);
                    arrowedLine(localization_img, point, Point2f(point.x + 10 * cos(pf.particles[i].theta), point.y + 10 * -sin(pf.particles[i].theta)), Scalar(0, 0, 255), 3);
                }
            }
            /* visualize robot pos */
            Point2f robot_point = Point2f(robot_pos_msg.x * 100 + localization_img.size().width / 2, localization_img.size().height - robot_pos_msg.y * 100 - localization_img.size().height / 2);
            circle(localization_img, robot_point, 12, Scalar(255, 0, 0), -1);
            arrowedLine(localization_img, robot_point, Point2f(robot_point.x + 12 * cos(robot_pos_msg.theta), robot_point.y + 12 * -sin(robot_pos_msg.theta)), Scalar(255, 255, 0), 3);

            /* log images */
            circle(log_image, robot_point, 6, Scalar(255, 0, 0), -1);
            arrowedLine(log_image, robot_point, Point2f(robot_point.x + 6 * cos(robot_pos_msg.theta), robot_point.y + 6 * -sin(robot_pos_msg.theta)), Scalar(255, 255, 0), 3);

            /* clear and re-initialize */
            observations.clear();
            ball = LandmarkObs{};

            /* publish fully drawn images to web socket */
            sensor_msgs::Image ros_localization_img = image2message(localization_img);
            localization_image_pub.publish(ros_localization_img);

            if (gui_mode != "nuc")
            {
                imshow("localization image", localization_img);
                waitKey(1);
            }
        }
        else if (gui_mode == "off"){continue;}

        ros::spinOnce();
        loop_rate.sleep();
    }

    /* Logging the final pose data */
    path = ros::package::getPath("robot_localization_data") + "/logs/results-" + to_string(pLocal->tm_mday) + "-" + to_string(pLocal->tm_hour) + "-" + to_string(pLocal->tm_min);
    boost::filesystem::create_directories(path);
    imwrite(path + "/PF.JPG", log_image);
    // imwrite(path + "/ideal_body_delta.JPG", log_image);
    // imwrite(path + "/center_foot.JPG", log_image);
    ROS_INFO("[robot_localization_pf_landmark] Success save log data");

    return 0;
}


/* ----------------------- ROS CALLBACK FUNC ----------------------- */
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
    for (int i = 0; i < msg->length; i++)
    {
        if (msg->data[i].name == "cpoint" || msg->data[i].name == "lpoint" || msg->data[i].name == "tpoint" || msg->data[i].name == "goalpost")
        {
            obs.x = msg->data[i].pos.x;
            obs.y = msg->data[i].pos.y;
            observations.push_back(obs);
        }
        else if (msg->data[i].name == "ball")
        {
            ball.x = msg->data[i].pos.x;
            ball.y = msg->data[i].pos.y;
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

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int torso_pitch_index, torso_yaw_index, head_yaw_index, head_pitch_index;
    for(int i=0; i<(msg->name).size(); i++)
    {
        if(msg->name[i] == "waist_yaw"  ) torso_yaw_index = i;
        if(msg->name[i] == "head_yaw"   ) head_yaw_index = i;
    }
    angle_torso_yaw = msg->position[torso_yaw_index];
    angle_head_yaw = msg->position[head_yaw_index];
    // ROS_INFO("[robot_localization_pf_landmark] : head_yaw : %3.3f, waist_yaw : %3.3f", angle_head_yaw * 180 / M_PI, angle_torso_yaw * 180 / M_PI);
}

void centerfootCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    m1.lock();
    if (msg->linear.z == 1)
    {
        sum_x_prev = sum_x;
        sum_y_prev = sum_y;
        sum_theta_prev = sum_theta;

        best_particle_prev.x = best_particle.x;
        best_particle_prev.y = best_particle.y;
        best_particle_prev.theta = best_particle.theta;

        x_local = 0;
        y_local = 0;
        theta_local = 0;
    }

    else
    {
        x_local = (msg->linear).x;
        y_local = (msg->linear).y;
        theta_local = (msg->angular).z;
    }

    // sum_x = sum_x_prev + (cos(sum_theta_prev) * x_local - sin(sum_theta_prev) * y_local);
    // sum_y = sum_y_prev + (sin(sum_theta_prev) * x_local + cos(sum_theta_prev) * y_local);
    // sum_theta = sum_theta_prev + theta_local;

    // best_particle.x = best_particle_prev.x + (cos(best_particle_prev.theta) * x_local - sin(best_particle_prev.theta) * y_local);
    // best_particle.y = best_particle_prev.y + (cos(best_particle_prev.theta) * x_local + sin(best_particle_prev.theta) * y_local);
    // best_particle.theta = best_particle_prev.theta + theta_local;

    // ROS_INFO("----------------------%f-------------------------",msg->linear.z);
    // ROS_INFO("local : %.5f | %.5f | %.5f",x_local,y_local,theta_local);
    // ROS_INFO("sum_p : %.5f | %.5f | %.5f",sum_x_prev,sum_y_prev,sum_theta_prev);
    // ROS_INFO("sum_t : %.5f | %.5f | %.5f",sum_x,sum_y,sum_theta);
    // ROS_INFO("------------------------------------------------");
    m1.unlock();
}

sensor_msgs::Image image2message(Mat image)
{
    //insert images to ros message
    cv_bridge::CvImage cv_image;
    cv_image.image = image;
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    return ros_image;
}
