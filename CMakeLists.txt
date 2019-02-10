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


list(APPEND sources
    src/vtk_ros/vtkRosGridMapSubscriber.cpp 
    src/vtk_ros/vtkRosPointCloudSubscriber.cpp
    src/vtk_ros/vtkRosDepthImageSubscriber.cpp
    src/vtk_ros/vtkRosInit.cpp
    src/vtk_ros/depthImageUtils.cpp
    src/vtk_ros/transformPolyDataUtils.cpp)


set(library_name vtk_ros)
add_library(${library_name} ${sources})
target_link_libraries(${library_name} ${catkin_LIBRARIES})


if(pkg_deps)
  use_pkg(${library_name} ${pkg_deps})
endif()

#install(TARGETS ${library_name}
#    EXPORT ${DD_TARGETS_NAME}
#    RUNTIME DESTINATION ${DD_INSTALL_BIN_DIR}
#    LIBRARY DESTINATION ${DD_INSTALL_LIB_DIR}
#    ARCHIVE DESTINATION ${DD_INSTALL_LIB_DIR})

#if(NOT VTK_WRAP_PYTHON)
#  message(FATAL_ERROR "VTK was compiled without Python support. "
#    "VTK_WRAP_PYTHON is OFF. You must use a version of VTK with Python. "
#    "VTK_DIR is: ${VTK_DIR}")
#endif()

#set(VTK_LIBRARIES)

#include_directories(${PYTHON_INCLUDE_DIR})
#include(${CMAKE_SOURCE_DIR}/cmake/wrap-python.cmake)
#wrap_python(${library_name} "${sources}")
#set_target_properties(${library_name}Python
#  PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/
#${DD_INSTALL_PYTHON_DIR}/director")
#target_link_libraries(${library_name}Python ${PYTHON_LIBRARIES})
