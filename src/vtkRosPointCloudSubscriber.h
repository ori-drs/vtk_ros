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

#include <rosSubscriberAlgorithm.h>

class vtkImageData;
class vtkTransform;

class VTKDRCFILTERS_EXPORT vtkRosPointCloudSubscriber : public vtkPolyDataAlgorithm, public RosSubscriberAlgorithm
{
public:
  vtkTypeMacro(vtkRosPointCloudSubscriber, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRosPointCloudSubscriber *New();

  void Start(std::string topic_name);

  void Stop();

  void GetPointCloud(vtkPolyData* polyData);

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


protected:

  vtkRosPointCloudSubscriber();
  virtual ~vtkRosPointCloudSubscriber();

private:
  vtkRosPointCloudSubscriber(const vtkRosPointCloudSubscriber&);  // Not implemented.
  void operator=(const vtkRosPointCloudSubscriber&);  // Not implemented.

  void PointCloudCallback(const sensor_msgs::PointCloud2Ptr& message);

  void addPointCloud(const vtkSmartPointer<vtkPolyData>& poly_data);

  std::deque<vtkSmartPointer<vtkPolyData> > dataset_;
  std::string frame_id_;
  std::string fixed_frame_;
  long sec_;
  long nsec_;
  int number_of_point_clouds_;
  sensor_msgs::PointCloud2Ptr input_;

  boost::shared_ptr<ros::Subscriber> subscriber_;
  std::mutex mutex_;
};

#endif // VTKROSPOINTCLOUDSUBSCRIBER_H_
