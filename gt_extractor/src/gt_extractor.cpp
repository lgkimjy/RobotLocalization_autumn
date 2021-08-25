/*
 * gt_extracotr.cpp
 *      Author: junyoung kim / lgkimjy
 */
#include "gt_extractor/helper_functions.h"

Mat blank_sum(740, 1040, CV_8UC4, cv::Scalar(0,0,0,0));
Mat blank_gt_img(740, 1040, CV_8UC4, cv::Scalar(0,0,0,0));
Mat blank_pf_img(740, 1040, CV_8UC4, cv::Scalar(0,0,0,0));
Mat blank_center_img(740, 1040, CV_8UC4, cv::Scalar(0,0,0,0));
Mat blank_ideal_img(740, 1040, CV_8UC4, cv::Scalar(0,0,0,0));

int main(int argc, char **argv){

    ros::init(argc, argv, "gt_extractor");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

	vector<ground_truth> gt;
	if (!read_gt_data(ros::package::getPath("robot_localization_data") + "/scenarios/scenario4/gt.txt", gt)) {
		cout << "Error: Could not open ground truth data file" << endl;
		return -1;
	}else{cout << "Success!" << endl;}

	// vector<pose> ideal;
	// if (!read_pose_data(ros::package::getPath("robot_localization_data") + "/scenarios/scenario1/ideal.txt", ideal)) {
	// 	cout << "Error: Could not open ground truth data file" << endl;
	// 	return -1;
	// }else{cout << "Success!" << endl;}

	vector<pose> foot_center;
	if (!read_pose_data(ros::package::getPath("robot_localization_data") + "/scenarios/scenario1/foot_center.txt", foot_center)) {
		cout << "Error: Could not open ground truth data file" << endl;
		return -1;
	}else{cout << "Success!" << endl;}

	vector<pose> pf;
	if (!read_pose_data(ros::package::getPath("robot_localization_data") + "/scenarios/scenario1/pf.txt", pf)) {
		cout << "Error: Could not open ground truth data file" << endl;
		return -1;
	}else{cout << "Success!" << endl;}

    while (ros::ok())
    {
        /* draw circle dt position */
        // for(int i=0; i<gt.size(); i++){
        //     Point2f robot_point; 
        //     robot_point = Point2f(gt[i].x * 100 + blank_pf_img.size().width / 2, blank_pf_img.size().height - gt[i].y * 100 - blank_pf_img.size().height / 2);
        //     circle(blank_pf_img, robot_point, 3, Scalar(0, 0, 0, 255), -1);
        // }
        
        Point2f robot_point;
        Point2f pre_robot_point;

        /* draw line gt position */
        for(int i=0; i<gt.size(); i++){
            if(i==0){
                pre_robot_point = Point2f(gt[i].x * 100 + blank_gt_img.size().width / 2, blank_gt_img.size().height - gt[i].y * 100 - blank_gt_img.size().height / 2);
                circle(blank_gt_img, robot_point, 3, Scalar(0, 0, 0, 255), -1);
            }
            else{
                robot_point = Point2f(gt[i].x * 100 + blank_gt_img.size().width / 2, blank_gt_img.size().height - gt[i].y * 100 - blank_gt_img.size().height / 2);
                line(blank_gt_img, pre_robot_point, robot_point, Scalar(0, 0, 0, 255), 4);
                line(blank_sum, pre_robot_point, robot_point, Scalar(0, 0, 0, 255), 4);
                pre_robot_point = robot_point;
            }
        }
        // /* ideal */
        // for(int i=0; i<ideal.size(); i++){
        //     if(i==0){
        //         pre_robot_point = Point2f(ideal[i].x * 100 + blank_ideal_img.size().width / 2, blank_ideal_img.size().height - ideal[i].y * 100 - blank_ideal_img.size().height / 2);
        //         circle(blank_ideal_img, robot_point, 3, Scalar(0, 128, 255, 255), -1);
        //     }
        //     else{
        //         robot_point = Point2f(ideal[i].x * 100 + blank_ideal_img.size().width / 2, blank_ideal_img.size().height - ideal[i].y * 100 - blank_ideal_img.size().height / 2);
        //         line(blank_ideal_img, pre_robot_point, robot_point, Scalar(0, 128, 255, 255), 4);
        //         line(blank_sum, pre_robot_point, robot_point, Scalar(0, 128, 255, 255), 4);
        //         pre_robot_point = robot_point;
        //     }
        // }
        /* center foot */
        for(int i=0; i<foot_center.size(); i++){
            if(i==0){
                pre_robot_point = Point2f(foot_center[i].x * 100 + blank_center_img.size().width / 2, blank_center_img.size().height - foot_center[i].y * 100 - blank_center_img.size().height / 2);
                circle(blank_center_img, robot_point, 3, Scalar(0, 0, 255, 255), -1);
            }
            else{
                robot_point = Point2f(foot_center[i].x * 100 + blank_center_img.size().width / 2, blank_center_img.size().height - foot_center[i].y * 100 - blank_center_img.size().height / 2);
                line(blank_center_img, pre_robot_point, robot_point, Scalar(0, 0, 255, 255), 4);
                line(blank_sum, pre_robot_point, robot_point, Scalar(0, 0, 255, 255), 4);
                pre_robot_point = robot_point;
            }
        }
        /* pf */
        for(int i=0; i<pf.size(); i++){
            if(i==0){
                pre_robot_point = Point2f(pf[i].x * 100 + blank_pf_img.size().width / 2, blank_pf_img.size().height - pf[i].y * 100 - blank_pf_img.size().height / 2);
                circle(blank_pf_img, robot_point, 3, Scalar(255, 0, 0, 255), -1);
            }
            else{
                robot_point = Point2f(pf[i].x * 100 + blank_pf_img.size().width / 2, blank_pf_img.size().height - pf[i].y * 100 - blank_pf_img.size().height / 2);
                line(blank_pf_img, pre_robot_point, robot_point, Scalar(255, 0, 0, 255), 4);
                line(blank_sum, pre_robot_point, robot_point, Scalar(255, 0, 0, 255), 4);
                pre_robot_point = robot_point;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    imwrite(ros::package::getPath("robot_localization_data") + "/logs/scenario4/gt.png", blank_gt_img);
    // imwrite(ros::package::getPath("robot_localization_data") + "/logs/scenario3/ideal.png", blank_ideal_img);
    imwrite(ros::package::getPath("robot_localization_data") + "/logs/scenario3/center.png", blank_center_img);
    imwrite(ros::package::getPath("robot_localization_data") + "/logs/scenario3/pf.png", blank_pf_img);
    imwrite(ros::package::getPath("robot_localization_data") + "/logs/scenario3/sum.png", blank_sum);

    return 0;
}