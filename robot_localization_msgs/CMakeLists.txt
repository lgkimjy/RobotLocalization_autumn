cmake_minimum_required(VERSION 2.8.3)
project(robot_localization_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

add_message_files(
  FILES
  ZedData.msg
  Setting.msg
  HeadTransform.msg
  ImageProcessing.msg
  ExtractKeyPointData.msg
  Obstacle.msg
  Obstacles.msg
  Poses.msg
  
  FootStepCommand.msg
  RobotMove.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}		
)
