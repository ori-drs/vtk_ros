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
#include <vtkTriangle.h>
#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkRosMarkerSubscriber);

vtkRosMarkerSubscriber::vtkRosMarkerSubscriber()
{
  if (!ros::isInitialized()) {
    std::cout << "WARNING: vtkRosMarkerSubscriber: ROS not Initialized\n";
  }
  fixed_frame_ = "map"; // or "odom"
}

vtkRosMarkerSubscriber::~vtkRosMarkerSubscriber()
{
  ros::shutdown();
}

void vtkRosMarkerSubscriber::Start(const std::string& topic_name)
{

  dataset_ = 0;
  topic_name_ = topic_name;

  ros::NodeHandle n;
  subscriber_ = boost::make_shared<ros::Subscriber>(
        n.subscribe(topic_name, 1000, &vtkRosMarkerSubscriber::Callback, this));

}

void vtkRosMarkerSubscriber::Stop()
{
  subscriber_->shutdown();
}

void vtkRosMarkerSubscriber::Callback(const visualization_msgs::MarkerPtr& message)
{
  try{
    TransformBetweenFrames(fixed_frame_, message->header.frame_id, message->header.stamp);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  dataset_ = ConvertMarker(message);
  transformPolyDataUtils::transformPolyData(dataset_, dataset_, sensor_to_local_transform_);
}

vtkSmartPointer<vtkPolyData> vtkRosMarkerSubscriber::ConvertMarker(const visualization_msgs::MarkerPtr& message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  int type = message->type;
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

  if(type == visualization_msgs::Marker::TRIANGLE_LIST)
  {
    int numCells = message->points.size() / 3; //number of triangle in the mesh
    int numPoints = 3*numCells;
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->SetDataTypeToDouble();
    points->SetNumberOfPoints(numPoints);

    bool hasColor = message->colors.size();
    vtkSmartPointer<vtkUnsignedCharArray> color = vtkSmartPointer<vtkUnsignedCharArray>::New();
    unsigned char* ptrColor = 0;
    if(hasColor)
    {
      color->SetNumberOfComponents(3);
      // initialize colors
      unsigned char * vtkColorCells = (unsigned char *)calloc(numPoints*3, sizeof(unsigned char));
      color->SetArray(vtkColorCells, numPoints*3, 0);
      color->SetName("color");
      ptrColor = vtkColorCells;
    }

    vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkIdTypeArray> cellsPtr = vtkSmartPointer<vtkIdTypeArray>::New();
    vtkIdType* vtkCells = (vtkIdType*)malloc(4*sizeof(vtkIdType)*numCells);
    vtkIdType* ptr = vtkCells;
    for(int i = 0; i < numCells; ++i)
    {
      *ptr = 3; // number of points followed by points ids
      *(ptr + 1) = i*3;
      *(ptr + 2) = i*3 + 1;
      *(ptr + 3) = i*3 + 2;
      ptr += 4;
    }
    cellsPtr->SetArray(vtkCells, 4*numCells, 0);
    cellArray->SetCells(numCells, cellsPtr);

    for(int i = 0; i < numPoints; ++i)
    {
      points->SetPoint(i, message->points[i].x, message->points[i].y, message->points[i].z);
      if(hasColor && ptrColor)
      {
        *ptrColor = 255*message->colors[i].r;
        *(ptrColor + 1) = 255*message->colors[i].g;
        *(ptrColor + 2) = 255*message->colors[i].b;
        ptrColor += 3;
      }
    }

    polyData->SetPoints(points);
    polyData->SetPolys(cellArray);
    if(hasColor)
      polyData->GetPointData()->AddArray(color);

  }
  return polyData;
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
