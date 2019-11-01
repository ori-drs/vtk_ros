#include "vtkRosDepthImageSubscriber.h"
#include <transformPolyDataUtils.h>

#include <vtkObjectFactory.h>

vtkStandardNewMacro(vtkRosDepthImageSubscriber);

vtkRosDepthImageSubscriber::vtkRosDepthImageSubscriber()
{
  if (!ros::isInitialized()) {
    std::cout << "WARNING: vtkRosGridMapSubscriber: ROS not Initialized\n";
  }
  dataset_ = vtkSmartPointer<vtkPolyData>::New();
  fixed_frame_ = "map"; // or "odom"
  sec_ = 0;
  nsec_ = 0;
}

vtkRosDepthImageSubscriber::~vtkRosDepthImageSubscriber()
{
  ros::shutdown();
}

void vtkRosDepthImageSubscriber::Start(const std::string& image_topic_a, const std::string& image_a_transport,
                                       const std::string& info_topic_a, const std::string& image_topic_b,
                                       const std::string& image_b_transport, const std::string& info_topic_b)
{
  dataset_ = vtkSmartPointer<vtkPolyData>::New();
  ros::NodeHandle node;

  image_topic_a_ = image_topic_a;
  image_a_transport_ = image_a_transport;
  info_topic_a_ = info_topic_a;
  image_topic_b_ = image_topic_b;
  image_b_transport_ = image_b_transport;
  info_topic_b_ = info_topic_b;

  image_a_sub_ = boost::make_shared<image_transport::SubscriberFilter>();
  image_b_sub_ = boost::make_shared<image_transport::SubscriberFilter>();
  it_ = boost::make_shared<image_transport::ImageTransport>(node);
  image_a_sub_->subscribe(*it_, ros::names::resolve(image_topic_a), 100, image_transport::TransportHints( image_a_transport ));
  image_b_sub_->subscribe(*it_, ros::names::resolve(image_topic_b), 100, image_transport::TransportHints( image_b_transport ));

  info_a_sub_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo> >();
  info_b_sub_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo> >();
  info_a_sub_->subscribe(node, ros::names::resolve(info_topic_a), 100);
  info_b_sub_->subscribe(node, ros::names::resolve(info_topic_b), 100);

  sync_ = boost::make_shared<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image,
      sensor_msgs::CameraInfo> >(10);
  sync_->connectInput(*image_a_sub_, *info_a_sub_, *image_b_sub_, *info_b_sub_);
  sync_->registerCallback(boost::bind(&vtkRosDepthImageSubscriber::DepthImageCallback, this, _1, _2, _3, _4));

}

void vtkRosDepthImageSubscriber::Stop()
{
  image_a_sub_->unsubscribe();
  image_b_sub_->unsubscribe();

  info_a_sub_->unsubscribe();
  info_b_sub_->unsubscribe();

  sync_.reset();
}

void vtkRosDepthImageSubscriber::ResetTime()
{
  RosSubscriberAlgorithm::ResetTime();

  // reset subscriber to empty queue of incoming messages
  std::lock_guard<std::mutex> lock(mutex_);
  if (image_a_sub_)
  {
    Stop();
    Start(image_topic_a_, image_a_transport_, info_topic_a_,
          image_topic_b_, image_b_transport_, info_topic_b_);
  }
}

void vtkRosDepthImageSubscriber::DepthImageCallback(const sensor_msgs::ImageConstPtr& image_a,
                                                    const sensor_msgs::CameraInfoConstPtr& info_a,
                                                    const sensor_msgs::ImageConstPtr& image_b,
                                                    const sensor_msgs::CameraInfoConstPtr& info_b)
{
  sec_ = image_a->header.stamp.sec;
  nsec_ = image_a->header.stamp.nsec;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  utils_.unpackImage(image_a, info_a, image_b, info_b, cloud);

  //
  std::string frame_id = image_b->header.frame_id;
  try {
    TransformBetweenFrames(fixed_frame_, frame_id);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  new_data_ = true;
  vtkSmartPointer<vtkPolyData> polyData = transformPolyDataUtils::PolyDataFromPointCloud(cloud);
  transformPolyDataUtils::transformPolyData(polyData, dataset_, sensor_to_local_transform_);
}

void vtkRosDepthImageSubscriber::GetPointCloud(vtkPolyData* poly_data, bool only_new_data)
{
  if (!poly_data || !dataset_  || (only_new_data && !new_data_))
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  new_data_ = false;
  poly_data->DeepCopy(dataset_);
}

void vtkRosDepthImageSubscriber::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}

void vtkRosDepthImageSubscriber::SetDecimate(int decimate)
{
  utils_.SetDecimate(decimate);
}

void vtkRosDepthImageSubscriber::SetRemoveSize(int size_threshold)
{
  utils_.SetRemoveSize(size_threshold);
}

void vtkRosDepthImageSubscriber::SetRangeThreshold(float range_threshold)
{
  utils_.SetRangeThreshold(range_threshold);
}
