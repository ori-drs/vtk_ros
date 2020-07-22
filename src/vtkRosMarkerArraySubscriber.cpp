#include "vtkRosMarkerArraySubscriber.h"
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

vtkStandardNewMacro(vtkRosMarkerArraySubscriber);

vtkRosMarkerArraySubscriber::vtkRosMarkerArraySubscriber()
  :vtkRosMarker()
{
}

vtkRosMarkerArraySubscriber::~vtkRosMarkerArraySubscriber()
{
  ros::shutdown();
}

void vtkRosMarkerArraySubscriber::Start(const std::string& topic_name)
{
  if(topic_name_ != topic_name)
  {
    dataset_.clear();
  }

  topic_name_ = topic_name;
  ros::NodeHandle n;
  subscriber_ = boost::make_shared<ros::Subscriber>(
        n.subscribe(topic_name, 1, &vtkRosMarkerArraySubscriber::Callback, this));

}

void vtkRosMarkerArraySubscriber::Stop()
{
  subscriber_->shutdown();
}

void vtkRosMarkerArraySubscriber::ResetTime()
{
  RosSubscriberAlgorithm::ResetTime();

  // reset subscriber to empty queue of incoming messages
  std::lock_guard<std::mutex> lock(mutex_);
  if (subscriber_)
  {
    Stop();
    Start(topic_name_);
  }
}

void vtkRosMarkerArraySubscriber::Callback(const visualization_msgs::MarkerArrayPtr& message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  dataset_.clear();
  for(int i = 0; i < message->markers.size(); ++i)
  {
    if (message->markers[i].header.frame_id.empty())
    {
      dataset_.push_back(vtkSmartPointer<vtkPolyData>::New());
      continue;
    }

    try
    {
      TransformBetweenFrames(fixed_frame_, message->markers[i].header.frame_id);
    }
    catch (tf::TransformException& ex){
      ROS_ERROR("%s",ex.what());
      return;
    }
    vtkSmartPointer<vtkPolyData> data = ConvertMarker(message->markers[i]);
    if(data->GetNumberOfPoints() > 0)
    {
      transformPolyDataUtils::transformPolyData(data, data, sensor_to_local_transform_);
      dataset_.push_back(data);
    }
  }
  if(dataset_.size() > 0)
  {
    vtkSmartPointer<vtkAppendPolyData> merge = vtkSmartPointer<vtkAppendPolyData>::New();
    merged_dataset_ = vtkSmartPointer<vtkPolyData>::New();
    for(int i = 0; i < dataset_.size(); ++i)
    {
      merge->AddInputData(dataset_[i]);
    }
    merge->Update();
    merged_dataset_->DeepCopy(merge->GetOutput());
  }
}

void vtkRosMarkerArraySubscriber::GetMesh(vtkPolyData* polyData, int index)
{
  if (dataset_.size()-1 < index || index < 0 || !polyData)
  {
    return;
  }

  //we can't copy dataset_ if it's being modified elsewhere
  std::lock_guard<std::mutex> lock(mutex_);
  polyData->DeepCopy(dataset_[index]);
}

void vtkRosMarkerArraySubscriber::GetMesh(vtkPolyData* poly_data)
{
  if (!poly_data || !merged_dataset_)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  poly_data->DeepCopy(merged_dataset_);
}

int vtkRosMarkerArraySubscriber::GetNumberOfMesh()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return dataset_.size();
}


void vtkRosMarkerArraySubscriber::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}
