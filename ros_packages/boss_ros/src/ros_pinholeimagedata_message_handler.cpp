#include "ros_pinholeimagedata_message_handler.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

using namespace std;

RosPinholeImageDataMessageHandler::RosPinholeImageDataMessageHandler(RosMessageContext* context, std::string topicName_, std::string imageSubtopic, std::string infoSubtopic): RosMessageHandler(context){
    _topicName = topicName_;
    _imageTopicName = _topicName + "/" + imageSubtopic;
    _cameraInfoTopicName = _topicName + "/" + infoSubtopic;
    _imageSub = 0;
    _infoSub = 0;
    _sync = 0;
    _sensor = 0;
  }

RosPinholeImageDataMessageHandler::~RosPinholeImageDataMessageHandler() {
  if (_sync){
    delete _sync;
  }
  if (_infoSub) {
    delete _infoSub;
    }
  if (_imageSub){
    delete _imageSub;
  }
}

void RosPinholeImageDataMessageHandler::subscribe(){
  _imageSub = new message_filters::Subscriber<sensor_msgs::Image>(*_context->nodeHandle(), _imageTopicName, 5);
  _infoSub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(*_context->nodeHandle(), _cameraInfoTopicName, 5);
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  _sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *_imageSub, *_infoSub);
  _sync->registerCallback(boost::bind(&RosPinholeImageDataMessageHandler::callback, this, _1, _2));
  cerr << "subscribed topics: [" <<  _imageTopicName << "] and [" << _cameraInfoTopicName << "]" << endl;
}


void RosPinholeImageDataMessageHandler::callback(sensor_msgs::Image::ConstPtr img, sensor_msgs::CameraInfo::ConstPtr info){
  if  (! _sensor) {
    std::map<std::string, boss::Frame*>::iterator it = _context->frameMap().find(info->header.frame_id);
    if (it == _context->frameMap().end()) {
      cerr << "missing transform for frame [" << info->header.frame_id << "], skipping" << endl;
      return;
    }
    _sensor = new boss::PinholeImageSensor;
    _sensor->setTopic(_imageTopicName);
    _sensor->setFrame(it->second);
    Eigen::Matrix3d cmat;
    int i=0;
    for (int r=0; r<3; r++)
      for (int c=0; c<3; c++, i++)
	cmat(r,c) = info->K[i];
    _sensor->setCameraMatrix(cmat);
    Eigen::VectorXd distParm(info->D.size());
    for (size_t i=0; i<info->D.size(); i++)
      distParm[i]=info->D[i];
    _sensor->setDistortionParameters(distParm);
    _sensor->setDistortionModel(info->distortion_model);
    cerr << "created sensor for topic[" << _sensor->topic()  << "]" << endl;
    _context->sensorMap().insert(make_pair(_sensor->topic(), _sensor));
  }
  Eigen::Isometry3d robotTransform;
  if (! _context->getOdomPose(robotTransform, img->header.stamp.toSec()) ){
    return;
  } 
  boss::Frame* newFrame = new boss::Frame("robotPose", robotTransform);
  // _context->messageQueue().push_back(newFrame);
  // we get the image from ROS

  cv_bridge::CvImagePtr ptr=cv_bridge::toCvCopy(img, img->encoding);
  boss::ImageBLOB* imageBlob = new boss::ImageBLOB();
  imageBlob->cvImage() = ptr->image.clone();
  imageBlob->adjustFormat();
  boss::PinholeImageData* imageData = new boss::PinholeImageData(_sensor);
  imageData->setTimestamp(img->header.stamp.toSec());
  imageData->setTopic(_sensor->topic());
  imageData->imageBlob().set(imageBlob);
  imageData->setRobotFrame(newFrame);
  _context->messageQueue().push_back(imageData);
  //delete imageData;
  //cerr << ".";
}
