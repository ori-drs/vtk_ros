#include "rosSubscriberAlgorithm.h"

#include "transformPolyDataUtils.h"

#include <vtkObjectFactory.h>

vtkStandardNewMacro(RosSubscriberAlgorithm);

RosSubscriberAlgorithm::RosSubscriberAlgorithm()
  :new_data_(false)
{
  tf_listener_ = boost::make_shared<tf::TransformListener>();
  sensor_to_local_transform_ = vtkSmartPointer<vtkTransform>::New();
}

RosSubscriberAlgorithm::~RosSubscriberAlgorithm()
{
}

void RosSubscriberAlgorithm::TransformBetweenFrames(const std::string& target_frame, const std::string& source_frame,
                                                    const ros::Time& time)
{
  tf::StampedTransform transform;
  tf_listener_->waitForTransform(target_frame, source_frame, time, ros::Duration(0.3));

  tf_listener_->lookupTransform(target_frame, source_frame, time, transform);
  sensor_to_local_transform_ = transformPolyDataUtils::transformFromPose(transform);


}

void RosSubscriberAlgorithm::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}

void RosSubscriberAlgorithm::ResetTime()
{
  tf_listener_->clear();
}
