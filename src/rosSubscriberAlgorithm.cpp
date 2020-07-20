#include "rosSubscriberAlgorithm.h"

#include "transformPolyDataUtils.h"

#include <vtkObjectFactory.h>

vtkStandardNewMacro(RosSubscriberAlgorithm);

boost::shared_ptr<tf::TransformListener> RosSubscriberAlgorithm::tf_listener_ = nullptr;

RosSubscriberAlgorithm::RosSubscriberAlgorithm()
  :new_data_(false)
{
  if(!tf_listener_)
  {
    tf_listener_ = boost::make_shared<tf::TransformListener>();
  }    
  sensor_to_local_transform_ = vtkSmartPointer<vtkTransform>::New();
}

RosSubscriberAlgorithm::~RosSubscriberAlgorithm()
{
}

void RosSubscriberAlgorithm::TransformBetweenFrames(const std::string& target_frame, const std::string& source_frame)
{
  tf::StampedTransform transform;

  // return the latest transform between target_frame and source_frame
  tf_listener_->lookupTransform(target_frame, source_frame, ros::Time(0), transform);

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

void RosSubscriberAlgorithm::SetTFPrefix(std::string prefix){
  tf_prefix_ = prefix;
}
