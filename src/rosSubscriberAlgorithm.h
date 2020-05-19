#ifndef ROSSUBSCRIBERALGORITHM_H
#define ROSSUBSCRIBERALGORITHM_H

#include <boost/shared_ptr.hpp>

#include <tf/transform_listener.h>

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
   * @brief TransformBetweenFrames returns the latest received transform between the two frames
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

  boost::shared_ptr<tf::TransformListener> tf_listener_;
  vtkSmartPointer<vtkTransform> sensor_to_local_transform_;
  bool new_data_;
  std::string tf_prefix_;

private:
  RosSubscriberAlgorithm(const RosSubscriberAlgorithm&);  // Not implemented.
  void operator=(const RosSubscriberAlgorithm&);  // Not implemented.

};

#endif // ROSSUBSCRIBERALGORITHM_H
