#ifndef VTKROSPOINTCLOUDSUBSCRIBER_H_
#define VTKROSPOINTCLOUDSUBSCRIBER_H_

#include <mutex>
#include <deque>

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <vtkDRCFiltersModule.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkPolyData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>
#include <vtkAppendPolyData.h>

#include <rosSubscriberAlgorithm.h>

class vtkImageData;
class vtkTransform;

class VTKDRCFILTERS_EXPORT vtkRosPointCloudSubscriber : public RosSubscriberAlgorithm
{
public:
  vtkTypeMacro(vtkRosPointCloudSubscriber, RosSubscriberAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRosPointCloudSubscriber *New();

  void Start(std::string topic_name);

  void Stop();

  /**
   * @brief GetPointCloud
   * @param polyData
   * @param only_new_data, if true return a point cloud if the current data is different from
   * the data returned by the previous call of this method
   */
  void GetPointCloud(vtkPolyData* polyData, bool only_new_data = false);

  std::string GetFrameId(){
    return frame_id_;
  }

  long GetSec(){
    return sec_;
  }
  long GetNsec(){
    return nsec_;
  }


  void SetFixedFrame(const std::string& fixed_frame_in){
    fixed_frame_ = fixed_frame_in;
  }

  void SetNumberOfPointClouds(int number_of_point_clouds);

  //this method is defined in RosSubscriberAlgorithm but it must be redefined here because of a limitation of vtk_wrap_python3
  /**
   * @brief ResetTime reset the transform listener
   */
  void ResetTime();


protected:

  vtkRosPointCloudSubscriber();
  virtual ~vtkRosPointCloudSubscriber();

private:
  vtkRosPointCloudSubscriber(const vtkRosPointCloudSubscriber&);  // Not implemented.
  void operator=(const vtkRosPointCloudSubscriber&);  // Not implemented.

  void PointCloudCallback(const sensor_msgs::PointCloud2Ptr& message);

  void addPointCloud(const vtkSmartPointer<vtkPolyData>& poly_data);

  /**
   * @brief dataset_ is a deque of the most recent point clouds received
   */
  std::deque<vtkSmartPointer<vtkPolyData> > dataset_;
  vtkSmartPointer<vtkAppendPolyData> append_poly_data_;
  std::string frame_id_;
  std::string fixed_frame_;
  long sec_;
  long nsec_;
  int number_of_point_clouds_;
  sensor_msgs::PointCloud2Ptr input_;

  boost::shared_ptr<ros::Subscriber> subscriber_;
  std::mutex mutex_;
  std::string topic_name_;
};

#endif // VTKROSPOINTCLOUDSUBSCRIBER_H_
