cmake_minimum_required(VERSION 2.8.3)
project(hippocampus_startup_node)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        mav_msgs
        nav_msgs
        mavros_msgs
        roscpp
        roscpp
        rospy
        sensor_msgs
        std_msgs
        tf tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES startup_node
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hippocampus_startup src/hippocampus_startup_node.cpp)
target_link_libraries(hippocampus_startup ${catkin_LIBRARIES})
