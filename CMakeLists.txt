cmake_minimum_required(VERSION 3.0.2)
project(pc_merge)

add_compile_options(-std=c++17 -O2 -g -Wall)

find_package(catkin REQUIRED
    roscpp
    rosbag
)
find_package(PCL)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_executable(pc_merge_offline
    src/pc_merge_offline.cpp
)
target_link_libraries(pc_merge_offline
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
