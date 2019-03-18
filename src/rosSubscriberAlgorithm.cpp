#include "rosSubscriberAlgorithm.h"

#include "transformPolyDataUtils.h"

RosSubscriberAlgorithm::RosSubscriberAlgorithm()
{
  tf_listener_ = boost::make_shared<tf::TransformListener>();
  sensor_to_local_transform_ = vtkSmartPointer<vtkTransform>::New();
}

void RosSubscriberAlgorithm::ResetTime()
{
  tf_listener_->clear();
}

void RosSubscriberAlgorithm::TransformBetweenFrames(const std::string& target_frame, const std::string& source_frame,
                                                    const ros::Time& time)
{
  tf::StampedTransform transform;
  tf_listener_->waitForTransform(target_frame, source_frame, time, ros::Duration(2.0));

  tf_listener_->lookupTransform(target_frame, source_frame, time, transform);
  sensor_to_local_transform_ = transformPolyDataUtils::transformFromPose(transform);


}
