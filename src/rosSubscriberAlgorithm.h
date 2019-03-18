#ifndef ROSSUBSCRIBERALGORITHM_H
#define ROSSUBSCRIBERALGORITHM_H

#include <boost/shared_ptr.hpp>

#include <tf/transform_listener.h>

#include <vtkSmartPointer.h>
#include <vtkTransform.h>


class RosSubscriberAlgorithm
{
public:
  RosSubscriberAlgorithm();

  void ResetTime();

  void TransformBetweenFrames(const std::string& target_frame, const std::string& source_frame,
                              const ros::Time& time);

protected:
  boost::shared_ptr<tf::TransformListener> tf_listener_;
  vtkSmartPointer<vtkTransform> sensor_to_local_transform_;
};

#endif // ROSSUBSCRIBERALGORITHM_H
