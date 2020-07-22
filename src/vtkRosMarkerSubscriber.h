#ifndef VTKROSMARKERSUBSCRIBER_H_
#define VTKROSMARKERSUBSCRIBER_H_


#include <mutex>
#include <deque>

#include <boost/shared_ptr.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <visualization_msgs/Marker.h>
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

class VTKDRCFILTERS_EXPORT vtkRosMarkerSubscriber : public vtkRosMarker
{
public:
  vtkTypeMacro(vtkRosMarkerSubscriber, vtkRosMarker);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRosMarkerSubscriber *New();

  void Start(const std::string& topic_name);

  void Stop();

  void ResetTime();

  /**
   * @brief GetMesh
   * @param polyData
   */
  void GetMesh(vtkPolyData* polyData);

  void SetFixedFrame(const std::string& fixed_frame_in)
  {
    fixed_frame_ = fixed_frame_in;
  }

  void SetTFPrefix(std::string prefix) {
    tf_prefix_ = prefix;
  }

protected:

  vtkRosMarkerSubscriber();
  virtual ~vtkRosMarkerSubscriber();

private:
  vtkRosMarkerSubscriber(const vtkRosMarkerSubscriber&);  // Not implemented.
  void operator=(const vtkRosMarkerSubscriber&);  // Not implemented.

  void Callback(const visualization_msgs::MarkerPtr& message);

  vtkSmartPointer<vtkPolyData> dataset_;
};

#endif // VTKROSMARKERSUBSCRIBER_H_
