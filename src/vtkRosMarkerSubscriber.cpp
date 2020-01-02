#include "vtkRosMarkerSubscriber.h"
#include <transformPolyDataUtils.h>
#include "vtkRosPointCloudConversions.h"

#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include <vtkImageData.h>
#include "vtkNew.h"

#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolygon.h>
#include <vtkLineSource.h>
#include <vtkTriangle.h>
#include <vtkAppendPolyData.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkRosMarkerSubscriber);

vtkRosMarkerSubscriber::vtkRosMarkerSubscriber()
{
  if (!ros::isInitialized()) {
    std::cout << "WARNING: vtkRosMarkerSubscriber: ROS not Initialized\n";
  }
  fixed_frame_ = "map"; // or "odom"
  dataset_ = 0;
}

vtkRosMarkerSubscriber::~vtkRosMarkerSubscriber()
{
  ros::shutdown();
}

void vtkRosMarkerSubscriber::Start(const std::string& topic_name)
{    
  if (topic_name_ != topic_name)
  {
    dataset_ = 0;
  }

  topic_name_ = topic_name;
  ros::NodeHandle n;
  subscriber_ = boost::make_shared<ros::Subscriber>(
        n.subscribe(topic_name, 1, &vtkRosMarkerSubscriber::Callback, this));

}

void vtkRosMarkerSubscriber::Stop()
{
  subscriber_->shutdown();
}

void vtkRosMarkerSubscriber::Callback(const visualization_msgs::MarkerPtr& message)
{
  if (message->header.frame_id.empty())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    dataset_ = vtkSmartPointer<vtkPolyData>::New();
    return;
  }

  try
  {
    TransformBetweenFrames(fixed_frame_, message->header.frame_id);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  dataset_ = ConvertMarker(*message);
  transformPolyDataUtils::transformPolyData(dataset_, dataset_, sensor_to_local_transform_);
}

void vtkRosMarkerSubscriber::GetMesh(vtkPolyData* polyData)
{
  if (!polyData || !dataset_)
  {
    return;
  }

  //we can't copy dataset_ if it's being modified in Callback
  std::lock_guard<std::mutex> lock(mutex_);
  polyData->DeepCopy(dataset_);
}


void vtkRosMarkerSubscriber::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}
