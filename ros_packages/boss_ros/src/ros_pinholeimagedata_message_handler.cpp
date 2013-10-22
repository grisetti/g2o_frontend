#include "ros_pinholeimagedata_message_handler.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind.hpp>

using namespace std;

RosPinholeImageDataMessageHandler::RosPinholeImageDataMessageHandler(RosMessageContext* context, std::string topicName_) : RosMessageHandler(context) {
  _pubQueueSize = 100;
  _topicName = topicName_;
  size_t index = _topicName.rfind("/");
  _infoTopicName = _topicName.substr(0, index) + "/camera_info";
  _imageTransport = new image_transport::ImageTransport(*context->nodeHandle());
  _cameraSubscriber = 0;
  _sensor = 0;
}

RosPinholeImageDataMessageHandler::~RosPinholeImageDataMessageHandler() {
  delete _imageTransport;
  if (_cameraSubscriber)
    delete _cameraSubscriber;
}

void RosPinholeImageDataMessageHandler::subscribe() {
  _cameraSubscriber = new image_transport::CameraSubscriber(_imageTransport->subscribeCamera(_topicName, 1, boost::bind(&RosPinholeImageDataMessageHandler::callback,this, _1, _2)));
						      ;
  cout << "subscribing to topic: [" <<  _topicName << "]" << endl;
}

void RosPinholeImageDataMessageHandler::setSensor(boss_logger::BaseSensor* sensor_) {
  boss_logger::PinholeImageSensor* cameraSensor = dynamic_cast<boss_logger::PinholeImageSensor*>(sensor_);
  if(!cameraSensor) {
    cerr << "WARNING: tried to set a non camera sensor to a camera message handler, skipping" << endl;
    return;
  }
  _sensor = cameraSensor;
}

void RosPinholeImageDataMessageHandler::publish(boss_logger::BaseSensorData* sdata) {
  if(!_sensor) {
    cerr << "WARNING: missing camera sensor, skipping" << endl;
    return;
  }

  boss_logger::PinholeImageData *pinholeImageData = dynamic_cast<boss_logger::PinholeImageData*>(sdata);
  if(!pinholeImageData) {
    cerr << "WARNING: trying to publish non camera data from a camera message handler, skipping" << endl;
    return;
  }

  // Create camera info message
  sensor_msgs::CameraInfo cameraInfo;
  std_msgs::Header cameraInfoHeader;
  cameraInfoHeader.stamp = ros::Time(pinholeImageData->timestamp());
  cameraInfoHeader.frame_id = _sensor->frame()->name();
  cameraInfo.header = cameraInfoHeader;
  boss_logger::ImageBLOB* imageBlob = pinholeImageData->imageBlob().get();
  cameraInfo.height = imageBlob->cvImage().rows;
  cameraInfo.width = imageBlob->cvImage().cols;
  Eigen::VectorXd distParam = _sensor->distortionParameters();
  const double D[] = {distParam[0], distParam[1], distParam[2], distParam[3], distParam[4]};    
  cameraInfo.D = std::vector<double>(D, D + sizeof(D) / sizeof(D[0]));
  Eigen::Matrix3d cam = _sensor->cameraMatrix();  
  boost::array<int, 9> K = {{cam(0, 0), cam(0, 1), cam(0, 2),
			     cam(1, 0), cam(1, 1), cam(1, 2),
			     cam(2, 0), cam(2, 1), cam(2, 2)}};
  cameraInfo.K = K;
  boost::array<int, 9> R = {{1.0, 0.0, 0.0, 
			     0.0, 1.0, 0.0, 
			     0.0, 0.0, 1.0}};
  cameraInfo.R = R;
  boost::array<int, 12> P = {{cam(0, 0), cam(0, 1), cam(0, 2), 0.0,
			      cam(1, 0), cam(1, 1), cam(1, 2), 0.0,
			      cam(2, 0), cam(2, 1), cam(2, 2), 0.0}};
  cameraInfo.P = P;
  cameraInfo.binning_x = 0;
  cameraInfo.binning_y = 0;
  cameraInfo.roi.x_offset = 0;
  cameraInfo.roi.y_offset = 0;
  cameraInfo.roi.height = 0;
  cameraInfo.roi.width = 0;
  cameraInfo.roi.do_rectify = false;

  // Create image message
  cv_bridge::CvImage cvImage;
  std_msgs::Header header;
  string encoding;
  header.stamp = ros::Time(pinholeImageData->timestamp());
  header.frame_id = _sensor->frame()->name();
  if (imageBlob->cvImage().type() == 16) {
    encoding = "rgb8";
  }
  else if (imageBlob->cvImage().type() == 2) {
    encoding = "16UC1";
  }
  else {
    cerr << "WARNING: camera image format to publish not recognized, skipping" << endl;
    return;
  }
  cvImage = cv_bridge::CvImage(header, encoding, imageBlob->cvImage());

  // Publish messages
  _infoPub.publish(cameraInfo);
  _pub.publish(cvImage.toImageMsg());
  
  delete(imageBlob);
}

void RosPinholeImageDataMessageHandler::advertise() {
  cout << "publishing on topic: [" << _topicName << "]" << endl;
  _pub = _context->nodeHandle()->advertise<sensor_msgs::Image>(_topicName, _pubQueueSize);
  cout << "publishing on topic: [" << _infoTopicName << "]" << endl;
  _infoPub = _context->nodeHandle()->advertise<sensor_msgs::CameraInfo>(_infoTopicName, _pubQueueSize);
}

bool RosPinholeImageDataMessageHandler::configReady() const {
  return _sensor;
}
  
void RosPinholeImageDataMessageHandler::callback(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::CameraInfo::ConstPtr& info) {
  if  (! _sensor) {
    boss_logger::StringReferenceFrameMap::iterator it = _context->frameMap().find(info->header.frame_id);
    if (it == _context->frameMap().end()) {
      cerr << "missing transform for frame [" << info->header.frame_id << "], skipping" << endl;
      return;
    }
    _sensor = new boss_logger::PinholeImageSensor;
    _sensor->setTopic(_topicName);
    _sensor->setReferenceFrame(it->second);
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
  boss_logger::ReferenceFrame* newReferenceFrame = new boss_logger::ReferenceFrame("robotPose", robotTransform);
  // _context->messageQueue().push_back(newReferenceFrame);
  // we get the image from ROS

  cv_bridge::CvImagePtr ptr=cv_bridge::toCvCopy(img, img->encoding);
  boss_logger::ImageBLOB* imageBlob = new boss_logger::ImageBLOB();
  imageBlob->cvImage() = ptr->image.clone();
  imageBlob->adjustFormat();
  boss_logger::PinholeImageData* imageData = new boss_logger::PinholeImageData(_sensor);
  imageData->setTimestamp(img->header.stamp.toSec());
  imageData->setTopic(_sensor->topic());
  imageData->imageBlob().set(imageBlob);
  imageData->setRobotReferenceFrame(newReferenceFrame);
  _context->messageQueue().push_back(imageData);
  //delete imageData;
  //cerr << ".";
}
