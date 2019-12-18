#include "vtkRosGridMapSubscriber.h"
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

vtkStandardNewMacro(vtkRosGridMapSubscriber);

vtkRosGridMapSubscriber::vtkRosGridMapSubscriber()
{
  if (!ros::isInitialized()) {
    std::cout << "WARNING: vtkRosGridMapSubscriber: ROS not Initialized\n";
  }
  fixed_frame_ = "map"; // or "odom"
}

vtkRosGridMapSubscriber::~vtkRosGridMapSubscriber() {
  ros::shutdown();
}


void vtkRosGridMapSubscriber::Start(const std::string &topic_name) {
  topic_name_ = topic_name;
  dataset_ = 0;
  ros::NodeHandle n;
  subscriber_ = boost::make_shared<ros::Subscriber>(
        n.subscribe(topic_name_, 1000, &vtkRosGridMapSubscriber::GridMapCallback, this));
}

void vtkRosGridMapSubscriber::Stop() {
  if (subscriber_)
    subscriber_->shutdown();
}

void vtkRosGridMapSubscriber::ResetTime()
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

void vtkRosGridMapSubscriber::GridMapCallback(const grid_map_msgs::GridMap& message) {
  // Convert message to map.
  grid_map::GridMapRosConverter::fromMessage(message, inputMap_);

  std::string frame_id = message.info.header.frame_id;
  try{
    TransformBetweenFrames(fixed_frame_, frame_id);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  //we can't modify dataset_ if it's being copied in GetMesh
  std::lock_guard<std::mutex> lock(mutex_);
  new_data_ = true;
  CreatePolyData();
}

void vtkRosGridMapSubscriber::CreatePolyData()
{
  dataset_ = ConvertMesh();
  transformPolyDataUtils::transformPolyData(dataset_, dataset_, sensor_to_local_transform_);
}

vtkSmartPointer<vtkPolyData> vtkRosGridMapSubscriber::ConvertMesh()
{
  inputMap_.convertToDefaultStartIndex();
  const size_t rows = inputMap_.getSize()(0);
  const size_t cols = inputMap_.getSize()(1);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToDouble();

  const std::vector< std::string > &  layers = inputMap_.getLayers();
  std::vector<vtkSmartPointer<vtkUnsignedCharArray> > colors(layers.size());
  float minIntensity, maxIntensity;


  std::string elevationLayer = "elevation";
  // check if the gridmap contains the layer called elevationLayer
  if ( std::find(layers.begin(), layers.end(), elevationLayer) == layers.end()) {
    std::cout << "No layer called elevation" << std::endl;
    return polyData;
  }
  // check if the gridmap contains the layer called colorLayer
  bool hasColorLayer = true;
  if ( std::find(layers.begin(), layers.end(), colorLayer_) == layers.end()) {
     hasColorLayer = false;
  }

  std::vector<std::vector<grid_map::Position3> > vertices((rows-1)*(cols-1));
  std::vector<std::vector<grid_map::Index> > indexes((rows-1)*(cols-1));
  int numPoints = 0;
  int numCells = 0;
  // first loop to compute the number of vertices and the number of cells
  for (size_t i = 0; i < rows - 1; ++i) {
    for (size_t j = 0; j < cols - 1; ++j) {

      for (size_t k = 0; k < 2; k++) {
        for (size_t l = 0; l < 2; l++) {
          grid_map::Position3 position;
          grid_map::Index index(i + k, j + l);
          if (!inputMap_.isValid(index)) {
            continue;
          }

          inputMap_.getPosition3(elevationLayer, index, position);
          vertices[i*(cols-1) + j].push_back(position);
          indexes[i*(cols-1) + j].push_back(index);
        }
      }
      if (vertices[i*(cols-1) + j].size() == 3) {
        numPoints += 3;
        numCells += 1;
      } else if (vertices[i*(cols-1) + j].size() == 4) {
        numPoints += 6;
        numCells += 2;
      }

    }
  }

  points->SetNumberOfPoints(numPoints);
  // initialize colors, one for each layers
  unsigned char* ptrColor = 0;
  for(int i = 0; i < layers.size(); ++i) {
    if(layers[i] == colorLayer_) {
      minIntensity = inputMap_[layers[i]].minCoeffOfFinites();
      maxIntensity = inputMap_[layers[i]].maxCoeffOfFinites();
    }

    colors[i] = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors[i]->SetNumberOfComponents(3);
    colors[i]->SetName(layers[i].c_str());

    // initialize colors
    unsigned char * vtkCells = (unsigned char *)calloc(numPoints*3, sizeof(unsigned char));
    colors[i]->SetArray(vtkCells, numPoints*3, 0);
    if(layers[i] == colorLayer_) {
      ptrColor = vtkCells;
    }
  }

  // populating the cells, the following code is equivalent to
  /*vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
  triangle->GetPointIds()->SetId(0, count_point);
  triangle->GetPointIds()->SetId(1, count_point + 1);
  triangle->GetPointIds()->SetId(2, count_point + 2);
  cellArray->InsertNextCell(triangle);*/
  vtkSmartPointer<vtkIdTypeArray> cellsPtr = vtkSmartPointer<vtkIdTypeArray>::New();
  vtkIdType* vtkCells = (vtkIdType*)malloc(4*sizeof(vtkIdType)*numCells);
  vtkIdType* ptr = vtkCells;
  for(int i = 0; i < numCells; ++i) {
    *ptr = 3; // number of points followed by points ids
    *(ptr + 1) = i*3;
    *(ptr + 2) = i*3 + 1;
    *(ptr + 3) = i*3 + 2;
    ptr += 4;
  }
  cellsPtr->SetArray(vtkCells, 4*numCells, 0);
  //cellsPtr->SetVoidArray(vtkCells, 4*numCells, 0);
  cellArray->SetCells(numCells, cellsPtr);

  // inserting the points and their colors
  int count_point = 0;
  unsigned char color[3];
  for (size_t j = 0; j < vertices.size(); ++j) {
    if (vertices[j].size() > 2) {
      for (size_t m = 1; m < vertices[j].size() - 1; m++) {
        points->SetPoint(count_point, vertices[j][m-1](0), vertices[j][m-1](1), vertices[j][m-1](2));
        points->SetPoint(count_point + 1, vertices[j][m](0), vertices[j][m](1), vertices[j][m](2));
        points->SetPoint(count_point + 2, vertices[j][m+1](0), vertices[j][m+1](1), vertices[j][m+1](2));

        //colors
        if( hasColorLayer && ptrColor != 0)
        {
          getInterpolatedColor(inputMap_, colorLayer_, indexes[j][m-1],
              minIntensity, maxIntensity, color);
          //colors[i]->InsertNextTupleValue(color);
          *ptrColor = color[0];
          *(ptrColor + 1) = color[1];
          *(ptrColor + 2) = color[2];
          getInterpolatedColor(inputMap_, colorLayer_, indexes[j][m],
                               minIntensity, maxIntensity, color);
          *(ptrColor + 3) = color[0];
          *(ptrColor + 4) = color[1];
          *(ptrColor + 5) = color[2];
          getInterpolatedColor(inputMap_, colorLayer_, indexes[j][m+1],
              minIntensity, maxIntensity, color);
          *(ptrColor + 6) = color[0];
          *(ptrColor + 7) = color[1];
          *(ptrColor + 8) = color[2];
          ptrColor += 9;
        }
        count_point += 3;

      }
    }
  }

  polyData->SetPoints(points);
  for(int i = 0; i< layers.size(); ++i) {
    polyData->GetPointData()->AddArray(colors[i]);
  }
  polyData->SetPolys(cellArray);

  return polyData;
}

void vtkRosGridMapSubscriber::SetColorLayer(const std::string& colorLayer) {
  colorLayer_ = colorLayer;
  // update dataset_
  std::lock_guard<std::mutex> lock(mutex_);
  CreatePolyData();
}

void vtkRosGridMapSubscriber::GetMesh(vtkPolyData* polyData, bool only_new_data)
{
  if (!polyData || !dataset_ || (only_new_data && !new_data_))
  {
    return;
  }

  //we can't copy dataset_ if it's being modified in GridMapCallback
  std::lock_guard<std::mutex> lock(mutex_);
  new_data_ = false;
  polyData->DeepCopy(dataset_);
}


// ----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkRosGridMapSubscriber::ConvertMeshToPointCloud() {

  const size_t numberOfPoints = dataset_->GetNumberOfPoints();

  vtkSmartPointer<vtkPolyData> polyData = vtkPolyData::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToFloat();
  points->Allocate(numberOfPoints);
  polyData->SetPoints(points);
  polyData->SetVerts(NewVertexCells(numberOfPoints));

  // TODO: add the other fields into the point cloud

  for (size_t i = 0; i < numberOfPoints; ++i){
    double p[3];
    dataset_->GetPoint(i,p);
    points->InsertNextPoint(p[0], p[1], p[2]);
  }

  //std::cout << "numberOfPoints: "<< numberOfPoints <<"\n";
  return polyData;
}



void vtkRosGridMapSubscriber::GetPointCloud(vtkPolyData* polyData)
{
  if (!polyData || !dataset_)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  vtkSmartPointer<vtkPolyData> pointcloud = ConvertMeshToPointCloud();
  polyData->DeepCopy(pointcloud);
}

void vtkRosGridMapSubscriber::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}

void vtkRosGridMapSubscriber::normalizeIntensity(float& intensity, float minIntensity, float maxIntensity) const
{
  if (std::abs(maxIntensity - minIntensity) > 0.01) {
    intensity = std::min(intensity, maxIntensity);
    intensity = std::max(intensity, minIntensity);
    intensity = (intensity - minIntensity) / (maxIntensity - minIntensity);
  } else {
    intensity = 1.f;
  }
}

void vtkRosGridMapSubscriber::getInterpolatedColor(grid_map::GridMap& inputMap, const std::string& colorLayer,
                                                   const grid_map::Index& index, float minIntensity, float maxIntensity,
                                                   unsigned char (&color)[3]) const {
  float intensity = inputMap[colorLayer](index(0), index(1));
  if (colorLayer != "color") {
    // transform the intensity to get a value between [0-1]
    normalizeIntensity(intensity, minIntensity, maxIntensity);
    // transform the intensity into jet color
    color[0] = 255 * clamp(std::min(4*intensity - 1.5, -4*intensity + 4.5) , 0.0, 1.0);
    color[1] = 255 * clamp(std::min(4*intensity - 0.5, -4*intensity + 3.5) , 0.0, 1.0);
    color[2] = 255 * clamp(std::min(4*intensity + 0.5, -4*intensity + 2.5) , 0.0, 1.0);
  } else {
    // extracting the color from the layer
    Eigen::Vector3f colorVectorRGB;
    grid_map::colorValueToVector(intensity, colorVectorRGB);
    color[0] = 255 * colorVectorRGB(0);
    color[1] = 255 * colorVectorRGB(1);
    color[2] = 255 * colorVectorRGB(2);
  }
}

float vtkRosGridMapSubscriber::clamp(float x, float lower, float upper)
{
  return std::min(upper, std::max(x, lower));
}



