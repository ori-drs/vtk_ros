cmake_minimum_required(VERSION 2.8.3)
project(vtk_ros)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  multisense_image_utils
  grid_map_ros)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL REQUIRED)

catkin_package(
)

###########
## Build ##
###########


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

