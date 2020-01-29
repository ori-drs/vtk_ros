#ifndef VTKROSPATHSUBSCRIBER_H_
#define VTKROSPATHSUBSCRIBER_H_

#include <mutex>
#include <deque>

#include <boost/shared_ptr.hpp>

#include <nav_msgs/Path.h>
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

class VTKDRCFILTERS_EXPORT vtkRosPathSubscriber : public RosSubscriberAlgorithm
{
public:
  vtkTypeMacro(vtkRosPathSubscriber, RosSubscriberAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRosPathSubscriber *New();

  void Start(std::string topic_name);

  void Stop();

  /**
   * @brief GetMesh
   * @param only_new_data, if true return meshes if the current data is different from
   * the data returned by the previous call of this method
   */
  void GetMeshes(vtkPolyData* frames, vtkPolyData* lines, bool only_new_data = false);


  void SetFixedFrame(const std::string& fixed_frame_in){
    fixed_frame_ = fixed_frame_in;
  }

  /**
   * @brief Set the scale of the frames
   */
  void SetScale(double scale);

  /**
   * @brief Use a tube or a thin line to represent the axis of frames
   */
  void UseTube(bool use_tube);

  void SetTubeWidth(double tube_width);

  //this method is defined in RosSubscriberAlgorithm but it must be redefined here because of a limitation of vtk_wrap_python3
  /**
   * @brief ResetTime reset the transform listener
   */
  void ResetTime();


protected:

  vtkRosPathSubscriber();
  virtual ~vtkRosPathSubscriber();

private:
  vtkRosPathSubscriber(const vtkRosPathSubscriber&);  // Not implemented.
  void operator=(const vtkRosPathSubscriber&);  // Not implemented.

  void ProcessData(const nav_msgs::PathPtr& message);

  vtkSmartPointer<vtkPolyData> GetFrame(const geometry_msgs::Pose &pose);

  vtkSmartPointer<vtkPolyData> GetLine(const geometry_msgs::Pose& pose1,
                                       const geometry_msgs::Pose& pose2);

  vtkSmartPointer<vtkPolyData> frames_dataset_; // contains the frames received
  vtkSmartPointer<vtkPolyData> lines_dataset_;  //contains the lines received
  nav_msgs::PathPtr last_data_;
  std::string frame_id_;
  std::string fixed_frame_;

  bool use_tube_;
  double scale_;
  double tube_width_;

  boost::shared_ptr<ros::Subscriber> subscriber_;
  std::mutex mutex_;
  std::string topic_name_;
};

#endif // VTKROSPATHSUBSCRIBER_H_
