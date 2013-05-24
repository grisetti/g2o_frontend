#include "SensorHandlerOmnicam.h"
#include <cv_bridge/cv_bridge.h>
#include <g2o_frontend/sensor_data/omnicam_data.h>
#include <g2o_frontend/sensor_data/sensor_data.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>

SensorHandlerOmnicam::SensorHandlerOmnicam(tf::TransformListener* tfListener_) : _omnicam_sub(NULL), _nptr(NULL)
{
  _tfListener = tfListener_;
  _isCalibrated = false;
}

SensorHandlerOmnicam::~SensorHandlerOmnicam() {
	delete _omnicam_sub;
}

Sensor* SensorHandlerOmnicam::getSensor(){
  return _sensor;
}

bool SensorHandlerOmnicam::setQueue(PriorityDataQueue* queue_) {
	PrioritySynchronousDataQueue* prioritySyncQueue = dynamic_cast<PrioritySynchronousDataQueue*>(queue_);
	if (prioritySyncQueue == 0) return false;
	_queue = queue_;
	return true;
}


bool SensorHandlerOmnicam::setSensor(Sensor* sensor_) {
	_sensor = dynamic_cast<SensorOmnicam*>(sensor_);
	if (_sensor == 0) return false;
	return true;
}

void SensorHandlerOmnicam::setNodeHandler(ros::NodeHandle* nptr_) {
	_nptr = nptr_;
}

void SensorHandlerOmnicam::callback(const sensor_msgs::ImageConstPtr& msg){
  if(!_isCalibrated){
    _calibrationCallback(msg);
  }
  else{
    _sensorCallback(msg);
  }
}

void SensorHandlerOmnicam::_sensorCallback(const sensor_msgs::ImageConstPtr& msg){
	ROS_INFO("Omnicam sensor callback\n");
	cv_bridge::CvImagePtr omniImage;

	try
	{
		omniImage = cv_bridge::toCvCopy(msg, msg->encoding);
	}
	catch (cv_bridge::Exception& ex)
	{
		ROS_ERROR("cv_bridge exception: %s", ex.what());
		ROS_INFO("quitted Omnicam callback for exception\n");
		return;
	}

	cv::Mat* imagePtr = new cv::Mat(omniImage->image);
	
	OmnicamData * data = new OmnicamData(imagePtr);
	data->setSensor((SensorOmnicam*)_sensor);
	data->setTimeStamp(msg->header.stamp.toSec());
	_queue->insert(data);
	ROS_INFO("finished Omnicam callback\n");

}

void SensorHandlerOmnicam::_calibrationCallback(const sensor_msgs::ImageConstPtr& raw_image){
  SensorOmnicam* s = static_cast<SensorOmnicam*>(getSensor());
  if(!s){
    ROS_ERROR("No sensor set");
    return;
  }
  string frame_id = raw_image->header.frame_id;
  tf::StampedTransform transform;
  geometry_msgs::TransformStamped humanReadableAndNiceTransform;
  try{
    ros::Time timestamp;
    // Get transformation
    _tfListener->lookupTransform("/base_link", frame_id, raw_image->header.stamp,transform);
    _isCalibrated = true;
    tf::transformStampedTFToMsg(transform, humanReadableAndNiceTransform);
  }
  catch(tf::TransformException & ex){
    ROS_ERROR("%s", ex.what());
  }
  
  if(_isCalibrated){
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

void SensorHandlerOmnicam::registerCallback() {
  SensorOmnicam* s = dynamic_cast<SensorOmnicam*>(_sensor);
  if(!s){
    std::cout << "Cannot subscribe to topic, _sensor is not a SensorOmnicam pointer" << std::endl;
    return;
  }
  *_omnicam_sub = _nptr->subscribe(s->getTopic()->c_str(), 10, &SensorHandlerOmnicam::callback, this);
  std::cout << "Subscribed to topic: " << s->getTopic() << std::endl;
}

