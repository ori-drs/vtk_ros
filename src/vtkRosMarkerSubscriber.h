#ifndef VTKROSMARKER_H_
#define VTKROSMARKER_H_


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

#include <rosSubscriberAlgorithm.h>

class vtkImageData;
class vtkTransform;

class VTKDRCFILTERS_EXPORT vtkRosMarkerSubscriber : public RosSubscriberAlgorithm
{
public:
  vtkTypeMacro(vtkRosMarkerSubscriber, RosSubscriberAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRosMarkerSubscriber *New();

  void Start(const std::string& topic_name);

  void Stop();

  /**
   * @brief GetMesh
   * @param polyData
   */
  void GetMesh(vtkPolyData* polyData);

  void SetFixedFrame(const std::string& fixed_frame_in)
  {
    fixed_frame_ = fixed_frame_in;
  }

  /**
   * @brief ResetTime reset the transform listener
   */
  //void ResetTime();

protected:

  vtkRosMarkerSubscriber();
  virtual ~vtkRosMarkerSubscriber();

private:
  vtkRosMarkerSubscriber(const vtkRosMarkerSubscriber&);  // Not implemented.
  void operator=(const vtkRosMarkerSubscriber&);  // Not implemented.

  void Callback(const visualization_msgs::MarkerPtr& message);

  vtkSmartPointer<vtkPolyData> ConvertMarker(const visualization_msgs::MarkerPtr& message);

  void ApplyColor(vtkSmartPointer<vtkPolyData> &polyData, const visualization_msgs::MarkerPtr& message);

  vtkSmartPointer<vtkPolyData> ConvertTriangleList(const visualization_msgs::MarkerPtr& message);

  vtkSmartPointer<vtkPolyData> ConvertSphere(const visualization_msgs::MarkerPtr& message);

  vtkSmartPointer<vtkPolyData> ConvertCylinder(const visualization_msgs::MarkerPtr& message);

  vtkSmartPointer<vtkPolyData> ConvertLineList(const visualization_msgs::MarkerPtr& message);

  vtkSmartPointer<vtkPolyData> dataset_;
  std::string fixed_frame_; // the elevation map is transformed into this frame
  std::string topic_name_;

  boost::shared_ptr<ros::Subscriber> subscriber_;
  std::mutex mutex_;
};

#endif // VTKROSMARKER_H_
