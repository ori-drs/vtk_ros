#ifndef VTKROSMARKERARRAYSUBSCRIBER_H_
#define VTKROSMARKERARRAYSUBSCRIBER_H_


#include <mutex>
#include <deque>

#include <boost/shared_ptr.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <vtkDRCFiltersModule.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkPolyData.h>
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include <vtkSmartPointer.h>

#include <vtkRosMarker.h>

class vtkImageData;
class vtkTransform;

class VTKDRCFILTERS_EXPORT vtkRosMarkerArraySubscriber : public vtkRosMarker
{
public:
  vtkTypeMacro(vtkRosMarkerArraySubscriber, vtkRosMarker);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRosMarkerArraySubscriber *New();

  void Start(const std::string& topic_name);

  void Stop();

  /**
   * @brief Due to a limitation of vtk python wrapper, it's not possible to return an array of vtkPolyData
   * so the way to get all the received markers is to call this function with index going from 0 to
   * GetNumberOfMesh() - 1
   * @param polyData
   */
  void GetMesh(vtkPolyData* poly_data, int index);

  /**
   * @brief GetMesh returns all the received markers in a single polyData.
   * @param poly_data
   */
  void GetMesh(vtkPolyData* poly_data);

  /**
   * @brief GetNumberOfMesh returns the number of markers received
   * @return
   */
  int GetNumberOfMesh();


protected:

  vtkRosMarkerArraySubscriber();
  virtual ~vtkRosMarkerArraySubscriber();

private:
  vtkRosMarkerArraySubscriber(const vtkRosMarkerArraySubscriber&);  // Not implemented.
  void operator=(const vtkRosMarkerArraySubscriber&);  // Not implemented.

  void Callback(const visualization_msgs::MarkerArrayPtr& message);

  std::vector<vtkSmartPointer<vtkPolyData> > dataset_;
  vtkSmartPointer<vtkPolyData> merged_dataset_;
};

#endif // VTKROSMARKERARRAYSUBSCRIBER_H_
