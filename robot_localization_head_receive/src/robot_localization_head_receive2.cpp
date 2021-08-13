#include <stdio.h>
#include <iostream>
#include <math.h>
#include <string>
#include <Eigen/Dense>
#include "ros/ros.h"

#include <yaml-cpp/yaml.h>
#include <ros/package.h>

#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <robot_localization_msgs/HeadTransform.h>

using namespace ros;
using namespace std;
using namespace Eigen;

float angle_head_yaw=0.0;
float angle_head_pitch=0.0;
    
void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);

bool RobotYamlRead(string path, float* length_pelvis2headyaw, float* length_headyaw2headpitch, float* length_headpitch2camera);
bool ParticleYamlRead(string path, float& unit);

Matrix4f makeYawMatrix(float& theta);
Matrix4f makePitchMatrix(float& theta);
Matrix4f makeTransMatrix(Vector3f& position);
Matrix4f makeTransMatrix(Vector3f& position, float& unit);

void insertMatrix2Message(Matrix4f& matrix, std_msgs::Float32MultiArray& msg);
void insertMatrix2Message(Matrix4f& matrix, robot_localization_msgs::HeadTransform& msg, int flag); // flag=0 -> original, flag=1 -> unit

int main(int argc, char** argv)
{
    //read setting
    string yaml_robot_path = package::getPath("robot_localization_data") + "/data/kin_dyn_2.yaml";
    string yaml_particle_path = package::getPath("robot_localization_data") + "/data/particle_setting.yaml";
   
    float length_pelvis2headyaw[3];
    float length_headyaw2headpitch[3];
    float length_headpitch2camera[3];

    float unit;
   
    if( !RobotYamlRead(yaml_robot_path, length_pelvis2headyaw, length_headyaw2headpitch, length_headpitch2camera) ) exit(0);
    if( !ParticleYamlRead(yaml_particle_path, unit) ) exit(0);
    
    Vector3f V_length_pelvis2headyaw(length_pelvis2headyaw[0],length_pelvis2headyaw[1],length_pelvis2headyaw[2]);
    Vector3f V_length_headyaw2headpitch(length_headyaw2headpitch[0],length_headyaw2headpitch[1],length_headyaw2headpitch[2]);
    Vector3f V_length_headpitch2camera(length_headpitch2camera[0], length_headpitch2camera[1], length_headpitch2camera[2]);
    

    //start ros
    ros::init(argc, argv, "robot_localization_head_receive2");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    ros::Publisher headtransform_pub = n.advertise<std_msgs::Float32MultiArray>("/alice/camera_transform",10);
    ros::Publisher custom_headtransform_pub = n.advertise<robot_localization_msgs::HeadTransform>("/robot_localization/head_transform",10);
    ros::Subscriber joint_sub = n.subscribe("/robotis/present_joint_states",10,jointCallback);

    while(ros::ok())
    {
        std_msgs::Float32MultiArray headtransform_msg;
        robot_localization_msgs::HeadTransform custom_headtransform_msg;

        //real world
        Matrix4f t_pelvis2headyaw = makeTransMatrix(V_length_pelvis2headyaw);
        Matrix4f r_pelvis2headyaw = Matrix4f::Identity();
        
        Matrix4f t_headyaw2headpitch = makeTransMatrix(V_length_headyaw2headpitch);
        Matrix4f r_headyaw2headpitch = makeYawMatrix(angle_head_yaw);

        Matrix4f t_headpitch2camera = makeTransMatrix(V_length_headpitch2camera);
        Matrix4f r_headpitch2camera = makePitchMatrix(angle_head_pitch);

        Matrix4f TR01 = r_pelvis2headyaw * t_pelvis2headyaw;
        Matrix4f TR12 = r_headyaw2headpitch * t_headyaw2headpitch;
        Matrix4f TR23 = r_headpitch2camera * t_headpitch2camera;

        Matrix4f Transform_pelvis2head = TR01 * TR12 * TR23;

        //virtual world
        Matrix4f t_pelvis2headyaw_V = makeTransMatrix(V_length_pelvis2headyaw, unit);
        Matrix4f t_headyaw2headpitch_V = makeTransMatrix(V_length_headyaw2headpitch, unit);
        Matrix4f t_headpitch2camera_V = makeTransMatrix(V_length_headpitch2camera, unit);

        Matrix4f TV01 = r_pelvis2headyaw * t_pelvis2headyaw_V;
        Matrix4f TV12 = r_headyaw2headpitch * t_headyaw2headpitch_V;
        Matrix4f TV23 = r_headpitch2camera * t_headpitch2camera_V;

        Matrix4f unit_Transform_pelvis2head = TV01 * TV12 * TV23;

        insertMatrix2Message(Transform_pelvis2head, headtransform_msg); 

        insertMatrix2Message(Transform_pelvis2head, custom_headtransform_msg, 0);
        insertMatrix2Message(unit_Transform_pelvis2head, custom_headtransform_msg, 1);

        headtransform_pub.publish(headtransform_msg);
        custom_headtransform_pub.publish(custom_headtransform_msg);
      
        cout << "-----------------------------------------------" << endl;
        cout << "head_yaw : " << angle_head_yaw * 180 / M_PI << endl;
        cout << "head_pitch : " << angle_head_pitch * 180 / M_PI << endl;
        cout << "-----------------------------------------------" << endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int head_yaw_index, head_pitch_index;
    for(int i=0; i<(msg->name).size(); i++)
    {
        if(msg->name[i] == "head_yaw"   ) head_yaw_index = i;
        if(msg->name[i] == "head_pitch" ) head_pitch_index = i;
    }

    angle_head_yaw = msg->position[head_yaw_index];
    angle_head_pitch = msg->position[head_pitch_index];
}

bool RobotYamlRead(string path, float* length_pelvis2headyaw, float* length_headyaw2headpitch, float* length_headpitch2camera)
{
    YAML::Node yaml_node;
    try
    {
        yaml_node = YAML::LoadFile(path.c_str());
        length_pelvis2headyaw[0]    = yaml_node["head_y"]["relative_position"][0].as<float>();
        length_pelvis2headyaw[1]    = yaml_node["head_y"]["relative_position"][1].as<float>();
        length_pelvis2headyaw[2]    = yaml_node["head_y"]["relative_position"][2].as<float>();

        length_headyaw2headpitch[0] = yaml_node["head_p"]["relative_position"][0].as<float>();
        length_headyaw2headpitch[1] = yaml_node["head_p"]["relative_position"][1].as<float>();
        length_headyaw2headpitch[2] = yaml_node["head_p"]["relative_position"][2].as<float>();

        length_headpitch2camera[0]  = yaml_node["cam"]["relative_position"][0].as<float>();
        length_headpitch2camera[1]  = yaml_node["cam"]["relative_position"][1].as<float>();
        length_headpitch2camera[2]  = yaml_node["cam"]["relative_position"][2].as<float>();
    }
    catch(const exception& e)
    {
        ROS_ERROR("fail to read robot yaml file");
        return false;
    }
    return true;
}

bool ParticleYamlRead(string path, float& unit)
{
    YAML::Node yaml_node;
    try
    {
        yaml_node = YAML::LoadFile(path.c_str());
        unit = yaml_node["ETCSetting"]["unit"].as<float>();
    }
    catch(const exception& e)
    {
        ROS_ERROR("fail to read particle yaml file");
        return false;
    }
    return true;
}

void insertMatrix2Message(Matrix4f& matrix, std_msgs::Float32MultiArray& msg)
{
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            msg.data.push_back(matrix(i,j));
}

void insertMatrix2Message(Matrix4f& matrix, robot_localization_msgs::HeadTransform& msg, int flag)
{
    switch(flag)
    {
        case 0:
            for(int i=0; i<4; i++)
                for(int j=0; j<4; j++)
                    msg.Tdata.push_back(matrix(i,j));
            break;
        case 1:
            for(int i=0; i<4; i++)
                for(int j=0; j<4; j++)
                    msg.Udata.push_back(matrix(i,j));
            break;
        default:
            break;
    }
}

Matrix4f makeYawMatrix(float& theta)
{
    Matrix4f matrix;
    matrix(0,0)=cos(theta); matrix(0,1)=-sin(theta); matrix(0,2)=0; matrix(0,3)=0;
    matrix(1,0)=sin(theta); matrix(1,1)= cos(theta); matrix(1,2)=0; matrix(1,3)=0;
    matrix(2,0)=0;          matrix(2,1)=0;           matrix(2,2)=1; matrix(2,3)=0;
    matrix(3,0)=0;          matrix(3,1)=0;           matrix(3,2)=0; matrix(3,3)=1;

    return matrix;
}

Matrix4f makePitchMatrix(float& theta)
{
    Matrix4f matrix;
    matrix(0,0)= cos(theta); matrix(0,1)=0; matrix(0,2)=sin(theta); matrix(0,3)=0;
    matrix(1,0)=0;           matrix(1,1)=1; matrix(1,2)=0;          matrix(1,3)=0;
    matrix(2,0)=-sin(theta); matrix(2,1)=0; matrix(2,2)=cos(theta); matrix(2,3)=0;
    matrix(3,0)=0;           matrix(3,1)=0; matrix(3,2)=0;          matrix(3,3)=1;

    return matrix;
}

Matrix4f makeTransMatrix(Vector3f& position)
{
    Matrix4f matrix;
    matrix(0,0)=1; matrix(0,1)=0; matrix(0,2)=0; matrix(0,3)=position(0);
    matrix(1,0)=0; matrix(1,1)=1; matrix(1,2)=0; matrix(1,3)=position(1);
    matrix(2,0)=0; matrix(2,1)=0; matrix(2,2)=1; matrix(2,3)=position(2);
    matrix(3,0)=0; matrix(3,1)=0; matrix(3,2)=0; matrix(3,3)=1;

    return matrix;
}

Matrix4f makeTransMatrix(Vector3f& position, float& unit)
{
    Matrix4f matrix;
    matrix(0,0)=1; matrix(0,1)=0; matrix(0,2)=0; matrix(0,3)=position(0)*unit;
    matrix(1,0)=0; matrix(1,1)=1; matrix(1,2)=0; matrix(1,3)=position(1)*unit;
    matrix(2,0)=0; matrix(2,1)=0; matrix(2,2)=1; matrix(2,3)=position(2)*unit;
    matrix(3,0)=0; matrix(3,1)=0; matrix(3,2)=0; matrix(3,3)=1;

    return matrix;
}
