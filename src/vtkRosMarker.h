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

class VTKDRCFILTERS_EXPORT vtkRosMarker : public RosSubscriberAlgorithm
{
public:

  vtkTypeMacro(vtkRosMarker, RosSubscriberAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRosMarker *New();


  void SetFixedFrame(const std::string& fixed_frame_in)
  {
    fixed_frame_ = fixed_frame_in;
  }

protected:

  vtkRosMarker();
  virtual ~vtkRosMarker();

  vtkSmartPointer<vtkPolyData> ConvertMarker(const visualization_msgs::Marker &message);

  void ApplyColor(vtkSmartPointer<vtkPolyData> &polyData, const visualization_msgs::Marker& message);

  vtkSmartPointer<vtkPolyData> ConvertTriangleList(const visualization_msgs::Marker& message);

  vtkSmartPointer<vtkPolyData> ConvertSphere(const visualization_msgs::Marker& message);

  vtkSmartPointer<vtkPolyData> ConvertCylinder(const visualization_msgs::Marker& message);

  vtkSmartPointer<vtkPolyData> ConvertLineList(const visualization_msgs::Marker& message);

  std::string fixed_frame_;
  std::string topic_name_;

  boost::shared_ptr<ros::Subscriber> subscriber_;
  std::mutex mutex_;

};

#endif // VTKROSMARKER_H_
