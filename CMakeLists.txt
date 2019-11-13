cmake_minimum_required(VERSION 2.8.3)
project(vtk_ros)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  image_utils
  depth_image_utils
  image_transport
  grid_map_ros
  pcl_plugin)

find_package(VTK REQUIRED)
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

