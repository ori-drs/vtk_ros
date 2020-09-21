#ifndef ROSSUBSCRIBERALGORITHM_H
#define ROSSUBSCRIBERALGORITHM_H

#include <mutex>

#include <boost/shared_ptr.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vtkDRCFiltersModule.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkPolyDataAlgorithm.h>


class VTKDRCFILTERS_EXPORT RosSubscriberAlgorithm : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(RosSubscriberAlgorithm, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static RosSubscriberAlgorithm *New();

  /**
   * @brief TransformBetweenFrames put the latest received transform between the two frames into sensor_to_local_transform_
   */

  void TransformBetweenFrames(const std::string& target_frame, const std::string& source_frame);

  /**
   * @brief ResetTime reset the transform listener
   */
  void ResetTime();

  void SetTFPrefix(std::string prefix);

protected:

  RosSubscriberAlgorithm();
  virtual ~RosSubscriberAlgorithm();

  static boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  static boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex tf_mutex_;
  vtkSmartPointer<vtkTransform> sensor_to_local_transform_;
  bool new_data_;
  std::string tf_prefix_;

private:
  RosSubscriberAlgorithm(const RosSubscriberAlgorithm&);  // Not implemented.
  void operator=(const RosSubscriberAlgorithm&);  // Not implemented.

};

#endif // ROSSUBSCRIBERALGORITHM_H
