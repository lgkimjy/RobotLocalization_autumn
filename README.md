# RobotLocalization for autumn heroehscup 
use known keypoints monte carlo localzation algorithm

### Requirements
```
git clone https://github.com/RobotWebTools/mjpeg_server
```

### How to use
```
## Detection 
roslaunch zed_wrapper zed.launch
roslaunch darknet_ros darknet_ros.launch
rosrun alice_vision alice_vision_node

## Robot Localization
rosrun robot_localization_pf_landmarks robot_localization_pf_landmarks

## Initialize start/restart postion
rostopic pub /alice/reposition geometry_msgs/Pose2D \
"x: 5.2
y: 3.7
theta: 0.0" 

## Web streaming
rosrun mjpeg_server mjpeg_server _port:=8080
http://[network ip]:8080/stream?topic=//robot_localization/localization_image

## or just Launch parameterized launch file 
roslaunch robot_localization_pf_landmarks robot_localization.launch mode:=kinematics/PF gui_mode:=on/off
```

### Data Record
```
rosbag record -o filename.bag /alice/ideal_body_delta /alice/vision/detected_objects /heroehs/alice_center_foot_pose /robotis/present_joint_states
rosbag record -o result.bag /alice/robot_pos
```

### DATA to record
1. images to save
* foot_center_pose
* ideal_body_delta
* PF_robot_pos
* every poses in single image
2. positions to save -> rosbag (global position)
* foot_center_pose
* ideal_body_delta
* PF_robot_pos