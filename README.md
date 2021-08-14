# RobotLocalization for autumn heroehscup 
use known keypoints monte carlo localzation algorithm

### How to use
```
## Detection 
roslaunch zed_wrapper zed.launch
roslaunch darknet_ros darknet_ros.launch

## Robot Localization
rosrun robot_localization_pf_landmarks robot_localization_pf_landmarks

## Initialize start/restart postion
rostopic pub /alice/reposition geometry_msgs/Pose2D \
"x: 5.2
y: 3.7
theta: 0.0" 
```