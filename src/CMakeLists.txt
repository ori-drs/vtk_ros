
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

list(APPEND sources
    vtkRosGridMapSubscriber.cpp 
    vtkRosPointCloudSubscriber.cpp
    vtkRosDepthImageSubscriber.cpp
    vtkRosInit.cpp
    depthImageUtils.cpp
    transformPolyDataUtils.cpp)

set(VTK_LIBRARIES
  vtkRenderingFreeTypeFontConfig
  vtkInteractionWidgets
  vtkInteractionStyle
  vtkRenderingFreeType
  vtkRenderingCore
  vtkRendering${VTK_RENDERING_BACKEND}
  vtkFiltersExtraction
  vtkFiltersCore
  vtkCommonCore
  )


set(library_name vtkRos)
add_library(${library_name} ${sources})
target_link_libraries(${library_name} ${catkin_LIBRARIES}
  ${VTK_LIBRARIES})

include_directories(${PYTHON_INCLUDE_DIR})

include(${CMAKE_SOURCE_DIR}/cmake/wrap-python.cmake)
wrap_python(${library_name} "${sources}")
