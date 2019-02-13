
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

list(APPEND sources
    vtkRosGridMapSubscriber.cpp 
    vtkRosPointCloudSubscriber.cpp
    vtkRosDepthImageSubscriber.cpp
    vtkRosInit.cpp
    depthImageUtils.cpp
    transformPolyDataUtils.cpp)


set(library_name vtk_ros)
add_library(${library_name} ${sources})
target_link_libraries(${library_name} ${catkin_LIBRARIES})

include_directories(${PYTHON_INCLUDE_DIR})

include(${CMAKE_SOURCE_DIR}/cmake/wrap-python.cmake)
wrap_python(${library_name} "${sources}")

