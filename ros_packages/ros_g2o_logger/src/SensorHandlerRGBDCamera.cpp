#include "SensorHandlerRGBDCamera.h"
#include <boost/thread/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <g2o_frontend/sensor_data/rgbd_data.h>
#include <g2o_frontend/sensor_data/sensor_data.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>

SensorHandlerRGBDCamera::SensorHandlerRGBDCamera(tf::TransformListener* tfListener_) : _intensity_sub(NULL),
                                                     _depth_sub(NULL),
                                                     _sync(NULL),
                                                     _nh(NULL)
{
  _tfListener = tfListener_;
  _isCalibrated = false;
}

Sensor* SensorHandlerRGBDCamera::getSensor() {
	return _sensor;
}

bool SensorHandlerRGBDCamera::setQueue(PriorityDataQueue* queue_) {
	PrioritySynchronousDataQueue* prioritySyncQueue = dynamic_cast<PrioritySynchronousDataQueue*>(queue_);
	if (prioritySyncQueue == 0) return false;
	_queue = queue_;
	return true;
}


bool SensorHandlerRGBDCamera::setSensor(Sensor* sensor_) {
	SensorRGBDCamera* rgbdCamera = dynamic_cast<SensorRGBDCamera*>(sensor_);
	if (rgbdCamera == 0) return false;
	_sensor = sensor_;
	return true;
}

void SensorHandlerRGBDCamera::setNodeHandler(ros::NodeHandle* nh_) {
	_nh = nh_;
}


void SensorHandlerRGBDCamera::registerCallback() {
  SensorRGBDCamera* s = dynamic_cast<SensorRGBDCamera*>(_sensor);
  _depth_sub = new message_filters::Subscriber<sensor_msgs::Image>(*_nh, s->getDepthTopic(), 5);
  _intensity_sub = new message_filters::Subscriber<sensor_msgs::Image>(*_nh, s->getIntensityTopic(), 5);
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  _sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *_intensity_sub, *_depth_sub);
  _sync->registerCallback(boost::bind(&SensorHandlerRGBDCamera::callback, this, _1, _2));
  std::cout << "Subscribed to topics: " << std::endl;
  std::cout << "\t" << s->getDepthTopic() << std::endl;
  std::cout << "\t" << s->getIntensityTopic() << std::endl;
}


void SensorHandlerRGBDCamera::callback(const sensor_msgs::ImageConstPtr& raw_intensity_image, const sensor_msgs::ImageConstPtr& raw_depth_image){
  if (!_isCalibrated){
    _calibrationCallback(raw_intensity_image,raw_depth_image);
  } else {
    _sensorCallback(raw_intensity_image,raw_depth_image);
  }
}

void SensorHandlerRGBDCamera::_calibrationCallback(const sensor_msgs::ImageConstPtr& raw_intensity_image, const sensor_msgs::ImageConstPtr& raw_depth_image) {
  SensorRGBDCamera* s=static_cast<SensorRGBDCamera*>(getSensor());
  if (!s) {
    ROS_ERROR("No sensor is set");
    return;
  }
  
//  cerr << "I" << endl;

  string frame_id = raw_intensity_image->header.frame_id;
  tf::StampedTransform transform;
  geometry_msgs::TransformStamped humanReadableAndNiceTransform;
  try {
    ros::Time timeStamp;
    // Get transformation
    _tfListener->lookupTransform("/base_link", frame_id, raw_intensity_image->header.stamp, transform);
    _isCalibrated = true;
    tf::transformStampedTFToMsg(transform, humanReadableAndNiceTransform);
  }
  catch (tf::TransformException & ex) {
    ROS_ERROR("%s", ex.what());
  } 
  
  if (_isCalibrated){
    g2o::Vector7d vectorQT;
    vectorQT[0] = humanReadableAndNiceTransform.transform.translation.x;
    vectorQT[1] = humanReadableAndNiceTransform.transform.translation.y;
    vectorQT[2] = humanReadableAndNiceTransform.transform.translation.z;
    vectorQT[3] = humanReadableAndNiceTransform.transform.rotation.x;
    vectorQT[4] = humanReadableAndNiceTransform.transform.rotation.y;
    vectorQT[5] = humanReadableAndNiceTransform.transform.rotation.z;
    vectorQT[6] = humanReadableAndNiceTransform.transform.rotation.w;
    g2o::ParameterCamera* camParam = dynamic_cast<g2o::ParameterCamera*>(s->parameter());
    assert(camParam && " parameter  not set" );
    camParam->setOffset(g2o::internal::fromVectorQT(vectorQT));
  }
  
}

void SensorHandlerRGBDCamera::_sensorCallback(const sensor_msgs::ImageConstPtr& raw_intensity_image, const sensor_msgs::ImageConstPtr& raw_depth_image) {
  ROS_WARN_ONCE_NAMED("eval", "First RGBD-Data Received");
  // It's where we put the image that we will show
  cv_bridge::CvImagePtr depthImage;
  cv_bridge::CvImagePtr intensityImage;
  // Convert the gray and depth image in input to a OpenCV image type
  try
  {
    intensityImage = cv_bridge::toCvCopy(raw_intensity_image, raw_intensity_image->encoding);
    depthImage = cv_bridge::toCvCopy(raw_depth_image, raw_depth_image->encoding);
  }
  catch (cv_bridge::Exception& ex)
  {
  	ROS_ERROR("cv_bridge exception: %s", ex.what());
    return;
  }
  cv::Mat* intensityImagePtr = new cv::Mat(intensityImage->image);
  cv::Mat* depthImagePtr = new cv::Mat(depthImage->image);
  RGBDData* data = new RGBDData(_sensor,intensityImagePtr, depthImagePtr);
  SensorRGBDCamera* camera= dynamic_cast<SensorRGBDCamera*>(_sensor);
  data->setBaseFilename(camera->getCurrentFilename());
  camera->setNum(camera->getNum()+1);
  data->setTimeStamp(raw_depth_image->header.stamp.toSec());
  _queue->insert(data);
}
