#ifndef VTKROSGRIDMAPSUBSCRIBER_H_
#define VTKROSGRIDMAPSUBSCRIBER_H_

#include <mutex>
#include <deque>

#include <boost/shared_ptr.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
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

class VTKDRCFILTERS_EXPORT vtkRosGridMapSubscriber : public RosSubscriberAlgorithm
{
public:
  vtkTypeMacro(vtkRosGridMapSubscriber, RosSubscriberAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRosGridMapSubscriber *New();

  void Start();

  void Stop();

  /**
   * @brief GetMesh
   * @param polyData
   * @param only_new_data, if true return a point cloud if the current data is different from
   * the data returned by the previous call of this method
   */
  void GetMesh(vtkPolyData* polyData, bool only_new_data = false);

  void GetPointCloud(vtkPolyData* polyData);

  void SetColorLayer(const std::string& colorLayer);

  void SetFixedFrame(const std::string& fixed_frame_in){
    fixed_frame_ = fixed_frame_in;
  }

  /**
   * @brief ResetTime reset the transform listener
   */
  void ResetTime();

protected:

  vtkRosGridMapSubscriber();
  virtual ~vtkRosGridMapSubscriber();

private:
  vtkRosGridMapSubscriber(const vtkRosGridMapSubscriber&);  // Not implemented.
  void operator=(const vtkRosGridMapSubscriber&);  // Not implemented.

  void GridMapCallback(const grid_map_msgs::GridMap& message);

  /**
   * @brief Convert the input grid_map_msgs::GridMap into a vtkPolyData
   * @return the converted vtkPolyData
   */
  vtkSmartPointer<vtkPolyData> ConvertMesh();

  /**
   * @brief Convert and transform the input grid_map_msgs::GridMap
   */
  void CreatePolyData();

  /**
  * @brief normalizeIntensity computes color value in the interval [0,1].
  */
  void normalizeIntensity(float& intensity, float minIntensity, float maxIntensity) const;

  /**
   * @brief getInterpolatedColor computes the color contained in the colorLayer
   */
  void getInterpolatedColor(grid_map::GridMap& inputMap, const std::string& colorLayer,
                            const grid_map::Index& index, float minIntensity, float maxIntensity,
                            unsigned char (&color)[3]) const;

  vtkSmartPointer<vtkPolyData> ConvertMeshToPointCloud();

  static float clamp(float x, float lower, float upper);

  vtkSmartPointer<vtkPolyData> dataset_;
  grid_map::GridMap inputMap_;
  std::string fixed_frame_; // the elevation map is transformed into this frame
  std::string colorLayer_;

  boost::shared_ptr<ros::Subscriber> subscriber_;
  std::mutex mutex_;
};

#endif // VTKROSGRIDMAPSUBSCRIBER_H_
