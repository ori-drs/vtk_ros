cmake_minimum_required(VERSION 3.0.2)
project(vtk_ros)

set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  image_utils_drs
  depth_image_utils_drs
  image_transport
  grid_map_ros
  pcl_plugin
  tf2_geometry_msgs
  tf2_ros
  tf2)

if($ENV{ROS_DISTRO} MATCHES "noetic")
  message(STATUS "ROS Noetic")
  find_package(VTK 7 REQUIRED)
elseif($ENV{ROS_DISTRO} MATCHES "melodic")
  message(STATUS "ROS Melodic")
  find_package(VTK 6.3 REQUIRED)
else()
  message(FATAL_ERROR "Environment variable ROS_DISTRO not defined - cannot deduce correct VTK version. Did you source a ROS workspace?")
endif()
include(${VTK_USE_FILE})

find_package(PCL REQUIRED)

set(library_name vtkRos)
catkin_package(
INCLUDE_DIRS src/
LIBRARIES ${library_name}
)

###########
## Build ##
###########


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

