cmake_minimum_required(VERSION 2.8.3)
project(robot_localization_setposition)

find_package(catkin REQUIRED COMPONENTS
  rospy geometry_msgs std_msgs alice_ft_sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES robot_localization_setposition
  CATKIN_DEPENDS rospy geometry_msgs std_msgs alice_ft_sensor_msgs
  DEPENDS system_lib
)

catkin_install_python(PROGRAMS src/robot_localization_setposition.py src/robot_localization_setposition_player.py src/robot_localization_setposition_keeper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
