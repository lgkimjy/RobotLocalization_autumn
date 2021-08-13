cmake_minimum_required(VERSION 2.8.3)
project(robot_localization_head_receive)
add_definitions(-std=c++11)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
# find_package(PkgConfig)

find_package(PkgConfig REQUIRED) 
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")


find_package(catkin REQUIRED COMPONENTS
roscpp
roslib
std_msgs
sensor_msgs
robot_localization_msgs
Eigen3
)

catkin_package(
#INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp roslib std_msgs  sensor_msgs robot_localization_msgs
DEPENDS system_lib EIGEN3
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_CPP_INCLIDE_DIRS}
)

add_executable(robot_localization_head_receive2 src/robot_localization_head_receive2.cpp)
target_link_libraries(robot_localization_head_receive2 ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(robot_localization_head_receive2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(robot_localization_head_receive3 src/robot_localization_head_receive3.cpp)
target_link_libraries(robot_localization_head_receive3 ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(robot_localization_head_receive3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(robot_localization_head_receive3_test src/robot_localization_head_receive3_test.cpp)
target_link_libraries(robot_localization_head_receive3_test ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(robot_localization_head_receive3_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
