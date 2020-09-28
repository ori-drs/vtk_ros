#include "rosSubscriberAlgorithm.h"

#include "transformPolyDataUtils.h"

#include <vtkObjectFactory.h>

#include <tf2/LinearMath/Transform.h>

vtkStandardNewMacro(RosSubscriberAlgorithm);

boost::shared_ptr<tf2_ros::Buffer> RosSubscriberAlgorithm::tf_buffer_ = nullptr;
boost::shared_ptr<tf2_ros::TransformListener> RosSubscriberAlgorithm::tf_listener_ = nullptr;

RosSubscriberAlgorithm::RosSubscriberAlgorithm()
  :new_data_(false)
{
  if(!tf_listener_)
  {
    tf_buffer_ = boost::make_shared<tf2_ros::Buffer>();
    tf_listener_ = boost::make_shared<tf2_ros::TransformListener>(*(tf_buffer_.get()));
  }    
  sensor_to_local_transform_ = vtkSmartPointer<vtkTransform>::New();
}

RosSubscriberAlgorithm::~RosSubscriberAlgorithm()
{
}

void RosSubscriberAlgorithm::TransformBetweenFrames(const std::string& target_frame, const std::string& source_frame)
{
  std::lock_guard<std::mutex> lock(tf_mutex_);
  geometry_msgs::TransformStamped transform;
  // return the latest transform between target_frame and source_frame
  try {
    transform = tf_buffer_->lookupTransform(target_frame, source_frame, ros::Time(0));
  }  catch (tf2::TransformException &ex) {
    ROS_WARN("TF lookupTransform exception : %s",ex.what());
    return;
  }

  tf::Transform tf_transform;
  tf::transformMsgToTF (transform.transform, tf_transform);
  sensor_to_local_transform_ = transformPolyDataUtils::transformFromPose(tf_transform);
}

void RosSubscriberAlgorithm::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}

void RosSubscriberAlgorithm::ResetTime()
{
  // TODO : with the old tf_listener there was a "clear" method that doesn't exist anymore.
  // The awkward following steps are to reset it, is there a better way to do that ?
  std::lock_guard<std::mutex> lock(tf_mutex_);
  tf_listener_.reset();
  tf_buffer_.reset();
  tf_buffer_ = boost::make_shared<tf2_ros::Buffer>();
  tf_listener_ = boost::make_shared<tf2_ros::TransformListener>(*(tf_buffer_.get()));

}

void RosSubscriberAlgorithm::SetTFPrefix(std::string prefix){
  tf_prefix_ = prefix;
}
