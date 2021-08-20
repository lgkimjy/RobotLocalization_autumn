/*
* special thanks to milyangparkjaehun
*/

#include <iostream>
#include <string>
#include <cmath>
#include <sstream>
#include <random>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <alice_msgs/FoundObject.h>
#include <alice_msgs/FoundObjectArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <diagnostic_msgs/KeyValue.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

float obstacle_m;
string obstacle = "Obstacles : 0.00 (m)";
string move_key, move_value;
string head_key, head_value;
string arm_key, arm_value;

int drop_ball_value;
bool move_flag, head_flag, arm_flag, drop_ball_flag;

sensor_msgs::Image image2message(Mat image);

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 2)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

void obstacleCallback(const std_msgs::Float32::ConstPtr& msg){
    obstacle_m = msg->data;
    obstacle = "obstacles : " + to_string_with_precision(obstacle_m) + " (m)";
}

void moveCmdCallback(const diagnostic_msgs::KeyValue::ConstPtr& msg){
    move_key = msg->key;
    move_value = msg->value;
    move_flag = true;
}

void headCmdCallback(const diagnostic_msgs::KeyValue::ConstPtr& msg){
    head_key = msg->key;
    head_value = msg->value;
    head_flag = true;
}

void armCmdCallback(const diagnostic_msgs::KeyValue::ConstPtr& msg){
    arm_key = msg->key;
    arm_value = msg->value;
    arm_flag = true;
}

void dropBallCallback(const std_msgs::UInt8::ConstPtr& msg){
    drop_ball_value = msg->data;
    drop_ball_flag = true;
}

void resetFlag() {
    move_flag = false;
    head_flag = false;
    arm_flag = false;
    drop_ball_flag = false;
}

int randomInt() {
    // return rand() % 155 + 100;
    return rand() % 255;
}

Scalar randomColor() {
    return Scalar(randomInt(), randomInt(), randomInt());
}

int main(int argc, char **argv){

    ros::init(argc, argv, "robot_localization_pf_landmark");
    ros::NodeHandle nh; 
    ros::Rate loop_rate(10);

    // Mat console_img = Mat::zeros(cv::Size(500, 500), CV_64FC1);
    cv::Mat console_img(250, 800, CV_8UC3, cv::Scalar(255,255,255));

    // putText(console_img, "Move", Point(20, 80), 1, 1.5, Scalar(0, 0, 0), 2);
    // putText(console_img, "Head", Point(20, 130), 1, 1.5, Scalar(0, 0, 0), 2);
    // putText(console_img, "Arm", Point(20, 180), 1, 1.5, Scalar(0, 0, 0), 2);
    // putText(console_img, "Drop ball", Point(20, 230), 1, 1.5, Scalar(0, 0, 0), 2);

    // putText(console_img, ":", Point(150, 80), 1, 1.5, Scalar(0, 0, 0), 2);
    // putText(console_img, ":", Point(150, 130), 1, 1.5, Scalar(0, 0, 0), 2);
    // putText(console_img, ":", Point(150, 180), 1, 1.5, Scalar(0, 0, 0), 2);
    // putText(console_img, ":", Point(150, 230), 1, 1.5, Scalar(0, 0, 0), 2);


    ros::Subscriber obstacle_sub = nh.subscribe("/alice/vision/obstacles", 10, obstacleCallback);
    ros::Subscriber move_cmconsole_img__sub = nh.subscribe("/alice/move_command", 10, moveCmdCallback);
    ros::Subscriber head_cmd_sub = nh.subscribe("/alice/head_command", 10, headCmdCallback);
    ros::Subscriber arm_cmd_sub = nh.subscribe("/alice/arm_command", 10, armCmdCallback);
    ros::Subscriber drop_ball_sub = nh.subscribe("/alice/drop_ball", 10, dropBallCallback);
    // ros::Subscriber sub_landmark = nh.subscribe("/alice/vision/detected_objects", 10, landmarkCallback);

    ros::Publisher cmd_image_pub = nh.advertise<sensor_msgs::Image>("/cmd_img", 10);

    Scalar color;
    while(ros::ok()){

        if(move_flag) color = randomColor();
        else color = Scalar(0, 0, 0);
        putText(console_img, "Move", Point(20, 80), 1, 1.5, color, 2);
        if(head_flag) color = randomColor();
        else color = Scalar(0, 0, 0);
        putText(console_img, "Head", Point(20, 130), 1, 1.5, color, 2);
        if(arm_flag) color = randomColor();
        else color = Scalar(0, 0, 0);
        putText(console_img, "Arm", Point(20, 180), 1, 1.5, color, 2);
        if(drop_ball_flag) color = randomColor();
        else color = Scalar(0, 0, 0);
        putText(console_img, "Drop ball", Point(20, 230), 1, 1.5, color, 2);

        putText(console_img, ":", Point(150, 80), 1, 1.5, Scalar(0, 0, 0), 2);
        putText(console_img, ":", Point(150, 130), 1, 1.5, Scalar(0, 0, 0), 2);
        putText(console_img, ":", Point(150, 180), 1, 1.5, Scalar(0, 0, 0), 2);
        putText(console_img, ":", Point(150, 230), 1, 1.5, Scalar(0, 0, 0), 2);

        Mat console_img_ = console_img.clone();

        // putText(console_img_, obstacle, Point(20, 30), 1, 1.5, Scalar(0, 0, 0), 2);
        // putText(console_img_, move_key, Point(165, 80), 1, 1, Scalar(0, 0, 0), 1);
        // putText(console_img_, head_key, Point(165, 130), 1, 1, Scalar(0, 0, 0), 1);
        // putText(console_img_, arm_key, Point(165, 180), 1, 1, Scalar(0, 0, 0), 1);
        // putText(console_img_, to_string(drop_ball_value), Point(165, 230), 1, 1.5, Scalar(0, 0, 0), 2);

        // putText(console_img_, move_value, Point(500, 80), 1, 1, Scalar(0, 0, 0), 1);
        // putText(console_img_, head_value, Point(500, 130), 1, 1, Scalar(0, 0, 0), 1);
        // putText(console_img_, arm_value, Point(500, 180), 1, 1, Scalar(0, 0, 0), 1);

        putText(console_img_, obstacle, Point(20, 30), 1, 1.5, randomColor(), 2);
        putText(console_img_, move_key, Point(165, 80), 1, 1.5, randomColor(), 2);
        putText(console_img_, head_key, Point(165, 130), 1, 1.5, randomColor(), 2);
        putText(console_img_, arm_key, Point(165, 180), 1, 1.5, randomColor(), 2);
        putText(console_img_, to_string(drop_ball_value), Point(165, 230), 1, 1.5, randomColor(), 2);

        putText(console_img_, move_value, Point(500, 80), 1, 1.5, randomColor(), 2);
        putText(console_img_, head_value, Point(500, 130), 1, 1.5, randomColor(), 2);
        putText(console_img_, arm_value, Point(500, 180), 1, 1.5, randomColor(), 2);
        
        line(console_img, Point(0, 40), Point(800, 40), Scalar(0, 0, 0), 2, 8, 0);
        line(console_img, Point(0, 90), Point(800, 90), Scalar(0, 0, 0), 2, 8, 0);
        line(console_img, Point(0, 140), Point(800, 140), Scalar(0, 0, 0), 2, 8, 0);
        line(console_img, Point(0, 190), Point(800, 190), Scalar(0, 0, 0), 2, 8, 0);
        line(console_img, Point(0, 240), Point(800, 240), Scalar(0, 0, 0), 2, 8, 0);
        // imshow("test", console_img_);
        waitKey(1);

        sensor_msgs::Image cmd_img = image2message(console_img_);
        cmd_image_pub.publish(cmd_img);

        resetFlag();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
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
