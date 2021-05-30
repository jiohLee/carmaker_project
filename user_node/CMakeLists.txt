cmake_minimum_required(VERSION 3.0.2)
project(user_node)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  pcl_conversions
  message_generation
  geometry_msgs
  visualization_msgs
  hellocm_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   #sensor_msgs#
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES publish_node
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(user_pkg src/user_control_code.cpp)
target_link_libraries(user_pkg ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(user_pkg ${catkin_EXPORTED_TARGETS} ${hellocm_msgs})
