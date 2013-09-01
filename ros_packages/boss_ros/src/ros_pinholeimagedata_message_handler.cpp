#include "ros_pinholeimagedata_message_handler.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind.hpp>

using namespace std;

RosPinholeImageDataMessageHandler::RosPinholeImageDataMessageHandler(RosMessageContext* context, std::string topicName_): RosMessageHandler(context){
    _topicName = topicName_;
    _imageTransport = new image_transport::ImageTransport(*context->nodeHandle());
    _cameraSubscriber = 0;
    _sensor = 0;
  }

RosPinholeImageDataMessageHandler::~RosPinholeImageDataMessageHandler() {
  delete _imageTransport;
  if (_cameraSubscriber)
    delete _cameraSubscriber;
}

void RosPinholeImageDataMessageHandler::subscribe(){
  _cameraSubscriber = new image_transport::CameraSubscriber(_imageTransport->subscribeCamera(_topicName,1,boost::bind(&RosPinholeImageDataMessageHandler::callback,this, _1, _2)));
						      ;
  cerr << "subscribed image: [" <<  _topicName << "]" << endl;
}


bool RosPinholeImageDataMessageHandler::configReady() const{
  return _sensor;
}
  
void RosPinholeImageDataMessageHandler::callback(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::CameraInfo::ConstPtr& info){
  if  (! _sensor) {
    std::map<std::string, boss::Frame*>::iterator it = _context->frameMap().find(info->header.frame_id);
    if (it == _context->frameMap().end()) {
      cerr << "missing transform for frame [" << info->header.frame_id << "], skipping" << endl;
      return;
    }
    _sensor = new boss::PinholeImageSensor;
    _sensor->setTopic(_topicName);
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
