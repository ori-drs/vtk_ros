#include "vtkRosMarker.h"
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

vtkStandardNewMacro(vtkRosMarker);

vtkRosMarker::vtkRosMarker()
{
  if (!ros::isInitialized()) {
    std::cout << "WARNING: vtkRosMarker: ROS not Initialized\n";
  }
  fixed_frame_ = "map"; // or "odom"
}

vtkRosMarker::~vtkRosMarker()
{
  ros::shutdown();
}

vtkSmartPointer<vtkPolyData> vtkRosMarker::ConvertMarker(const visualization_msgs::Marker& message)
{
  int type = message.type;
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();

  if(type == visualization_msgs::Marker::TRIANGLE_LIST)
  {
    return ConvertTriangleList(message);
  } else if(type == visualization_msgs::Marker::CYLINDER)
  {
    return ConvertCylinder(message);
  } else if(type == visualization_msgs::Marker::SPHERE)
  {
    return ConvertSphere(message);
  } else if(type == visualization_msgs::Marker::LINE_LIST)
  {
    return ConvertLineList(message);
  } else {
    ROS_ERROR("Unknow type in vtkRosMarker");
  }
  return poly_data;
}

vtkSmartPointer<vtkPolyData> vtkRosMarker::ConvertTriangleList(const visualization_msgs::Marker &message)
{
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
  int num_cells = message.points.size() / 3; //number of triangle in the mesh
  int num_points = 3*num_cells;
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToDouble();
  points->SetNumberOfPoints(num_points);

  vtkSmartPointer<vtkCellArray> cell_array = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkIdTypeArray> cells_ptr = vtkSmartPointer<vtkIdTypeArray>::New();
  vtkIdType* vtk_cells = (vtkIdType*)malloc(4*sizeof(vtkIdType)*num_cells);
  vtkIdType* ptr = vtk_cells;
  for(int i = 0; i < num_cells; ++i)
  {
    *ptr = 3; // number of points followed by points ids
    *(ptr + 1) = i*3;
    *(ptr + 2) = i*3 + 1;
    *(ptr + 3) = i*3 + 2;
    ptr += 4;
  }
  cells_ptr->SetArray(vtk_cells, 4*num_cells, 0);
  cell_array->SetCells(num_cells, cells_ptr);

  for(int i = 0; i < num_points; ++i)
  {
    points->SetPoint(i, message.points[i].x, message.points[i].y, message.points[i].z);
  }

  poly_data->SetPoints(points);
  poly_data->SetPolys(cell_array);
  ApplyColor(poly_data, message);
  return poly_data;
}

vtkSmartPointer<vtkPolyData> vtkRosMarker::ConvertSphere(const visualization_msgs::Marker& message)
{
  vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
  sphere->SetPhiResolution(11);
  sphere->SetThetaResolution(11);
  sphere->SetCenter(message.pose.position.x, message.pose.position.y, message.pose.position.z);
  sphere->SetRadius(0.5*message.scale.x);
  sphere->Update();

  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
  poly_data->DeepCopy(sphere->GetOutput());
  ApplyColor(poly_data, message);
  return poly_data;
}

vtkSmartPointer<vtkPolyData> vtkRosMarker::ConvertCylinder(const visualization_msgs::Marker &message)
{
  vtkSmartPointer<vtkCylinderSource> cylinder = vtkSmartPointer<vtkCylinderSource>::New();
  cylinder->SetResolution(11);
  //cylinder->SetCenter(message.pose.position.x, message.pose.position.y, message.pose.position.z);
  cylinder->SetRadius(0.5*message.scale.x);
  cylinder->Update();
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
  poly_data->DeepCopy(cylinder->GetOutput());
  ApplyColor(poly_data, message);

  //transform cylinder to make it aligned along the z zxis
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->Translate(message.pose.position.x, message.pose.position.y, message.pose.position.z);
  transform->RotateX(90.0);
  transformPolyDataUtils::transformPolyData(poly_data, poly_data, transform);
  return poly_data;
}

void vtkRosMarker::ApplyColor(vtkSmartPointer<vtkPolyData>& poly_data,
                              const visualization_msgs::Marker& message)
{
  int num_points = poly_data->GetNumberOfPoints();
  bool has_color = (num_points == message.colors.size());
  vtkSmartPointer<vtkUnsignedCharArray> color = vtkSmartPointer<vtkUnsignedCharArray>::New();
  color->SetNumberOfComponents(3);
  // initialize colors
  unsigned char * vtk_color_cells = (unsigned char *)calloc(num_points*3, sizeof(unsigned char));
  color->SetArray(vtk_color_cells, num_points*3, 0);
  color->SetName("color");
  unsigned char* ptr_color = vtk_color_cells;

  for(int i = 0; i < num_points; ++i)
  {
    if(has_color)
    {
      *ptr_color = 255 * message.colors[i].r;
      *(ptr_color + 1) = 255 * message.colors[i].g;
      *(ptr_color + 2) = 255 * message.colors[i].b;
    } else {
      *ptr_color = 255 * message.color.r;
      *(ptr_color + 1) = 255 * message.color.g;
      *(ptr_color + 2) = 255 * message.color.b;
    }
    ptr_color += 3;
  }

  poly_data->GetPointData()->AddArray(color);
}

vtkSmartPointer<vtkPolyData> vtkRosMarker::ConvertLineList(const visualization_msgs::Marker& message)
{
  vtkSmartPointer<vtkAppendPolyData> lines_list = vtkSmartPointer<vtkAppendPolyData>::New();

  for(int i = 0; i+1 < message.points.size(); i+=2)
  {
    vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
    line->SetPoint1(message.points[i].x, message.points[i].y, message.points[i].z);
    line->SetPoint2(message.points[i+1].x, message.points[i+1].y, message.points[i+1].z);
    line->Update();
    lines_list->AddInputData(line->GetOutput());
  }
  lines_list->Update();
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
  poly_data->DeepCopy(lines_list->GetOutput());
  ApplyColor(poly_data, message);
  return poly_data;
}


void vtkRosMarker::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}
