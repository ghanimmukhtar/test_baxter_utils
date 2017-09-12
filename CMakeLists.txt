cmake_minimum_required(VERSION 2.8.3)
project(test_baxter_utils)

find_package(catkin REQUIRED COMPONENTS
  baxter_mover_utils
  roscpp
  rospy
  std_msgs
  tf  
  eigen_conversions
  eigen_stl_containers
  geometric_shapes
  geometry_msgs
  kdl_parser
  moveit_msgs
  octomap_msgs
  random_numbers
  roslib
  rostime
  sensor_msgs
  shape_msgs
  srdfdom
  std_msgs
  trajectory_msgs
  visualization_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

#set(BAXTER_MOVER_UTILS_DIR /usr/local/)
#find_path(BAXTER_MOVER_UTILS_INCLUDE_DIR baxter_mover_utils/baxter_mover.hpp baxter_mover_utils/helpers_methods.hpp)
#find_library(BAXTER_MOVER_UTILS_LIBRARY NAMES baxter_mover_utils)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES test_baxter_utils
  CATKIN_DEPENDS baxter_mover_utils roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_baxter_utils src/test_baxter_utils.cpp)
target_link_libraries(test_baxter_utils ${catkin_LIBRARIES}  )

add_executable(service_user src/service_user.cpp)
target_link_libraries(service_user ${catkin_LIBRARIES}  )

add_executable(test_octomap src/test_octomap.cpp)
target_link_libraries(test_octomap ${catkin_LIBRARIES}  )