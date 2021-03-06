
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

list(APPEND sources
    rosSubscriberAlgorithm.cpp
    transformPolyDataUtils.cpp
    vtkRosGridMapSubscriber.cpp
    vtkRosMarker.cpp
    vtkRosMarkerArraySubscriber.cpp
    vtkRosMarkerSubscriber.cpp
    vtkRosPathSubscriber.cpp 
    vtkRosPointCloudSubscriber.cpp
    vtkRosDepthImageSubscriber.cpp
    vtkRosInit.cpp
    vtkRosImageSubscriber.cpp
)

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
  vtkIOMPIImage
  vtkIOMPIParallel
  )

if(VTK_VERSION VERSION_GREATER 6.9)
  list(APPEND VTK_LIBRARIES
    vtkFiltersParallelDIY2
    vtkCommonExecutionModel
    vtkWrappingPythonCore
    vtkCommonExecutionModelPythonD
    )
endif()


add_library(${library_name} ${sources})
target_link_libraries(${library_name} ${catkin_LIBRARIES}
  ${VTK_LIBRARIES})

include_directories(${PYTHON_INCLUDE_DIR})

include(${CMAKE_SOURCE_DIR}/cmake/wrap-python.cmake)
wrap_python(${library_name} "${sources}")

