#include "vtkRosPathSubscriber.h"

#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkAxes.h>
#include <vtkTubeFilter.h>
#include <vtkLineSource.h>

#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolygon.h>
#include <vtkTriangle.h>
#include <vtkObjectFactory.h>

#include <pcl_conversions/pcl_conversions.h>

#include "transformPolyDataUtils.h"

vtkStandardNewMacro(vtkRosPathSubscriber);

vtkRosPathSubscriber::vtkRosPathSubscriber()
{
  if (!ros::isInitialized()) {
    std::cout << "WARNING: vtkRosGridMapSubscriber: ROS not Initialized\n";
  }
  frame_id_ = "no_frame";
  fixed_frame_ = "map"; // or "odom"
  scale_ = 0.35;
  use_tube_ = false;
  tube_width_ = 0.03;
}

vtkRosPathSubscriber::~vtkRosPathSubscriber()
{
  ros::shutdown();
}

void vtkRosPathSubscriber::Start(std::string topic_name)
{

  frames_dataset_ = 0;
  lines_dataset_ = 0;
  topic_name_ = topic_name;

  ros::NodeHandle n;
  subscriber_ = boost::make_shared<ros::Subscriber>(
        n.subscribe(topic_name, 10, &vtkRosPathSubscriber::ProcessData, this));

}

void vtkRosPathSubscriber::Stop() {
  if (subscriber_)
    subscriber_->shutdown();
}

void vtkRosPathSubscriber::ResetTime()
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

void vtkRosPathSubscriber::ProcessData(const nav_msgs::PathPtr& message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  frame_id_ = message->header.frame_id;
  last_data_ = boost::make_shared<nav_msgs::Path>(*message);
  try{
    TransformBetweenFrames(fixed_frame_, frame_id_);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s",ex.what());
    return;
  }

  new_data_ = true;

  vtkSmartPointer<vtkAppendPolyData> append_frames = vtkSmartPointer<vtkAppendPolyData>::New();
  vtkSmartPointer<vtkAppendPolyData> append_lines = vtkSmartPointer<vtkAppendPolyData>::New();
  for (int i=0; i < message->poses.size(); ++i)
  {
    vtkSmartPointer<vtkPolyData> frame = GetFrame(message->poses[i].pose);
    append_frames->AddInputData(frame);
    if(i > 0)
    {
      vtkSmartPointer<vtkPolyData> line = GetLine(message->poses[i-1].pose, message->poses[i].pose);
      append_lines->AddInputData(line);
    }
  }
  append_frames->Update();
  append_lines->Update();
  if(!frames_dataset_)
    frames_dataset_ = vtkSmartPointer<vtkPolyData>::New();
  frames_dataset_->DeepCopy(append_frames->GetOutput());
  if(!lines_dataset_)
    lines_dataset_ = vtkSmartPointer<vtkPolyData>::New();
  lines_dataset_->DeepCopy(append_lines->GetOutput());
}

void vtkRosPathSubscriber::GetMeshes(vtkPolyData* frames, vtkPolyData* lines, bool only_new_data)
{
  if (!frames || !lines ||
      !lines_dataset_ || !frames_dataset_ || (only_new_data && !new_data_))
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  new_data_ = false;
  frames->DeepCopy(frames_dataset_);
  lines->DeepCopy(lines_dataset_);
}

vtkSmartPointer<vtkPolyData> vtkRosPathSubscriber::GetFrame(const geometry_msgs::Pose& pose)
{
  vtkSmartPointer<vtkPolyData> output = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New();
  axes->SetComputeNormals(0);
  axes->SetScaleFactor(scale_);
  axes->Update();

  if(use_tube_)
  {
    vtkSmartPointer<vtkTubeFilter> tube = vtkSmartPointer<vtkTubeFilter>::New();
    tube->SetInputConnection(axes->GetOutputPort());
    tube->SetRadius(tube_width_);
    tube->SetNumberOfSides(12);
    tube->Update();
    output = tube->GetOutput();
  } else {
    output = axes->GetOutput();
  }

  vtkSmartPointer<vtkPolyData> transformed_poly_data = vtkSmartPointer<vtkPolyData>::New();
  tf::Transform transform(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                          tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  vtkSmartPointer<vtkTransform> vtk_transform = transformPolyDataUtils::transformFromPose(transform);
  transformPolyDataUtils::transformPolyData(output, transformed_poly_data, vtk_transform);
  transformed_poly_data->GetPointData()->GetArray("Axes")->SetName("Color");
  return transformed_poly_data;
}

vtkSmartPointer<vtkPolyData> vtkRosPathSubscriber::GetLine(const geometry_msgs::Pose& pose1,
                                                           const geometry_msgs::Pose& pose2)
{
  vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
  line->SetPoint1(pose1.position.x, pose1.position.y, pose1.position.z);
  line->SetPoint2(pose2.position.x, pose2.position.y, pose2.position.z);
  line->Update();
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
  poly_data->DeepCopy(line->GetOutput());

  int num_points = poly_data->GetNumberOfPoints();
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Color");
  // set line color
  unsigned char * vtk_color_cells = (unsigned char *)calloc(num_points*3, sizeof(unsigned char));
  colors->SetArray(vtk_color_cells, num_points*3, 0);
  unsigned char* ptr_color = vtk_color_cells;
  for(int i = 0; i < num_points; ++i)
  {
    *ptr_color = 255;
    *(ptr_color + 1) = 255;
    *(ptr_color + 2) = 0;
    ptr_color += 3;
  }
  poly_data->GetPointData()->AddArray(colors);
  return poly_data;
}

void vtkRosPathSubscriber::SetScale(double scale)
{
  scale_ = scale;
  if(last_data_)
    ProcessData(last_data_);
}

void vtkRosPathSubscriber::SetTubeWidth(double tube_width)
{
  tube_width_ = tube_width;
  if(last_data_)
    ProcessData(last_data_);
}

void vtkRosPathSubscriber::UseTube(bool use_tube)
{
  use_tube_ = use_tube;
  if(last_data_)
    ProcessData(last_data_);
}



void vtkRosPathSubscriber::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}
