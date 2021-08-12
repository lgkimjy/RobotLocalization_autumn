cmake_minimum_required(VERSION 3.0.2)
project(robot_localization_pf_landmarks)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
find_package(PkgConfig REQUIRED) 
pkg_search_module(OPENCV opencv opencv4)
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

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  sensor_msgs
  std_msgs
  roslib
  cv_bridge
  alice_msgs
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp roslib cv_bridge sensor_msgs  geometry_msgs alice_msgs
DEPENDS system_lib OPENCV
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLIDE_DIRS}
    ${OPENCV_INCLUDE_DIRS}
)

add_executable(robot_localization_pf_landmarks src/robot_localization_pf_landmarks.cpp src/particle_filter.cpp)
target_link_libraries(robot_localization_pf_landmarks ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${OPENCV_LIBRARIES})
add_dependencies(robot_localization_pf_landmarks ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})