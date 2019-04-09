#include "vtkRosPointCloudSubscriber.h"

#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkImageData.h>
#include <vtkNew.h>

#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolygon.h>
#include <vtkTriangle.h>
#include <vtkRosPointCloudConversions.h>
#include <vtkObjectFactory.h>

#include "transformPolyDataUtils.h"

vtkStandardNewMacro(vtkRosPointCloudSubscriber);

vtkRosPointCloudSubscriber::vtkRosPointCloudSubscriber()
{
  if (!ros::isInitialized()) {
    std::cout << "WARNING: vtkRosGridMapSubscriber: ROS not Initialized\n";
  }
  frame_id_ = "no_frame";
  fixed_frame_ = "map"; // or "odom"
  sec_ = 0;
  nsec_ = 0;
  number_of_point_clouds_ = 1;
}

vtkRosPointCloudSubscriber::~vtkRosPointCloudSubscriber() {
  ros::shutdown();
}

void vtkRosPointCloudSubscriber::Start(std::string topic_name) {

  append_poly_data_ = vtkSmartPointer<vtkAppendPolyData>::New();
  dataset_.clear();

  ros::NodeHandle n;
  subscriber_ = boost::make_shared<ros::Subscriber>(
        n.subscribe(topic_name, 1000, &vtkRosPointCloudSubscriber::PointCloudCallback, this));

}

void vtkRosPointCloudSubscriber::Stop() {
  subscriber_->shutdown();
}

void vtkRosPointCloudSubscriber::ResetTime()
{
  RosSubscriberAlgorithm::ResetTime();
}

void vtkRosPointCloudSubscriber::PointCloudCallback(const sensor_msgs::PointCloud2Ptr& message) {
  input_ = message;
  frame_id_ = message->header.frame_id;
  sec_ = message->header.stamp.sec;
  nsec_ = message->header.stamp.nsec;

  //
  ros::Time time = message->header.stamp;
  try{
    TransformBetweenFrames(fixed_frame_, frame_id_, time);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s",ex.what());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  new_data_ = true;
  vtkSmartPointer<vtkPolyData> poly_data = ConvertPointCloud2ToVtk(input_);
  vtkSmartPointer<vtkPolyData> transformed_poly_data = vtkSmartPointer<vtkPolyData>::New();
  transformPolyDataUtils::transformPolyData(poly_data, transformed_poly_data, sensor_to_local_transform_);
  addPointCloud(transformed_poly_data);
}

void vtkRosPointCloudSubscriber::GetPointCloud(vtkPolyData* polyData, bool only_new_data)
{
  if (!polyData || !dataset_.size() || (only_new_data && !new_data_))
  {
    return;
  }

  //we can't copy dataset_ if it's being modified in PointCloudCallback
  std::lock_guard<std::mutex> lock(mutex_);
  new_data_ = false;
  polyData->DeepCopy(append_poly_data_->GetOutput());
}

void vtkRosPointCloudSubscriber::addPointCloud(const vtkSmartPointer<vtkPolyData>& poly_data)
{
  dataset_.push_back(poly_data);
  append_poly_data_->AddInputData(poly_data);

  while(dataset_.size() > number_of_point_clouds_)
  {
    append_poly_data_->RemoveInputData(dataset_.front());
    dataset_.pop_front();
  }
  append_poly_data_->Update();
}

void vtkRosPointCloudSubscriber::SetNumberOfPointClouds(int number_of_point_clouds)
{
  number_of_point_clouds_ = number_of_point_clouds;
}

void vtkRosPointCloudSubscriber::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}
