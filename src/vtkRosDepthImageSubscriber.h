#ifndef VTKROSDEPTHIMAGESUBSCRIBER_H_
#define VTKROSDEPTHIMAGESUBSCRIBER_H_

#include <mutex>
#include <deque>

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <depth_image_utils_drs/depth_image_utils.hpp>

#include <vtkDRCFiltersModule.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkPolyData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>


#include <rosSubscriberAlgorithm.h>

class vtkImageData;
class vtkTransform;

class VTKDRCFILTERS_EXPORT vtkRosDepthImageSubscriber : public RosSubscriberAlgorithm
{
public:
  vtkTypeMacro(vtkRosDepthImageSubscriber, RosSubscriberAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRosDepthImageSubscriber *New();

  void Start(const std::string& image_topic_a, const std::string& image_a_transport,
             const std::string& info_topic_a, const std::string& image_topic_b,
             const std::string& image_b_transport, const std::string& info_topic_b);

  void Stop();

  /**
   * @brief GetPointCloud
   * @param polyData
   * @param only_new_data, if true return a point cloud if the current data is different from
   * the data returned by the previous call of this method
   */
  void GetPointCloud(vtkPolyData* poly_data, bool only_new_data = false);

  void SetDecimate(int decimate);

  void SetRemoveSize(int size_threshold);

  void SetRangeThreshold(float range_threshold);

  long GetSec(){
    return sec_;
  }
  long GetNsec(){
    return nsec_;
  }

  void SetFixedFrame(const std::string& fixed_frame_in){
    fixed_frame_ = fixed_frame_in;
  }

  /**
   * @brief ResetTime reset the transform listener
   */
  void ResetTime();

  void SetTFPrefix(std::string prefix) {
    tf_prefix_ = prefix;
  }

protected:

  vtkRosDepthImageSubscriber();
  virtual ~vtkRosDepthImageSubscriber();

private:
  vtkRosDepthImageSubscriber(const vtkRosDepthImageSubscriber&);  // Not implemented.
  void operator=(const vtkRosDepthImageSubscriber&);  // Not implemented.


  void DepthImageCallback(const sensor_msgs::ImageConstPtr& image_a,
                          const sensor_msgs::CameraInfoConstPtr& info_a,
                          const sensor_msgs::ImageConstPtr& image_b,
                          const sensor_msgs::CameraInfoConstPtr& info_b);



  vtkSmartPointer<vtkPolyData> dataset_;
  DepthImageUtils utils_;
  std::string fixed_frame_;
  long sec_;
  long nsec_;

  boost::shared_ptr<image_transport::SubscriberFilter> image_a_sub_, image_b_sub_;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > info_a_sub_, info_b_sub_;
  boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image,
      sensor_msgs::CameraInfo> > sync_;

  std::string image_topic_a_, image_a_transport_, info_topic_a_;
  std::string image_topic_b_, image_b_transport_, info_topic_b_;

  boost::shared_ptr<image_transport::ImageTransport> it_;
  std::mutex mutex_;
};

#endif // VTKROSDEPTHIMAGESUBSCRIBER_H_
