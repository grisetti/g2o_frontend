#include "ros_pinhole_camera_publisher_handler.h"

#include <cv_bridge/cv_bridge.h>

RosPinholeCameraPublisherHandler::RosPinholeCameraPublisherHandler(ros::NodeHandle *nh_, boss_logger::BaseSensor *sensor_, string topic_) {
  _nh = nh_;
  _sensor = sensor_;
  _topic = topic_;
  _sequenceID = 0;
  _publishingQueueSize = 100;
  size_t index = _topic.rfind("/");
  _cameraInfoTopic = _topic.substr(0, index) + "/camera_info";
}

RosPinholeCameraPublisherHandler::~RosPinholeCameraPublisherHandler() {}

void RosPinholeCameraPublisherHandler::publish() {
  cout << "Pinhole camera sensor publishing data on topic " << _topic << endl;
  _publisher = _nh->advertise<sensor_msgs::Image>(_topic, _publishingQueueSize);
  cout << "Pinhole camera sensor publishing camera info on topic " << _cameraInfoTopic << endl;
  _cameraInfoPublisher = _nh->advertise<sensor_msgs::CameraInfo>(_cameraInfoTopic, _publishingQueueSize);
  _configured = true;
}

void RosPinholeCameraPublisherHandler::callback(PinholeImageData *data) {
  PinholeImageSensor *imageSensor = dynamic_cast<PinholeImageSensor*>(_sensor);
  if(!imageSensor) {
    cerr << "WARNING: pinhole camera publisher handler has a sensor which is not of type pinhole camera, skipping" << endl;
    return;
  }
  
  // Create camera info message
  sensor_msgs::CameraInfo cameraInfo;
  std_msgs::Header cameraInfoHeader;
  cameraInfoHeader.seq = _sequenceID;
  cameraInfoHeader.stamp = ros::Time(data->timestamp());
  cameraInfoHeader.frame_id = imageSensor->frame()->name();
  cameraInfo.header = cameraInfoHeader;
  ImageBLOB *imageBlob = data->imageBlob().get();
  cameraInfo.height = imageBlob->cvImage().rows;
  cameraInfo.width = imageBlob->cvImage().cols;
  Eigen::VectorXd distParam = imageSensor->distortionParameters();
  const double D[] = {distParam[0], distParam[1], distParam[2], distParam[3], distParam[4]};    
  cameraInfo.D = std::vector<double>(D, D + sizeof(D) / sizeof(D[0]));
  Eigen::Matrix3d cam = imageSensor->cameraMatrix();  
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
  header.seq = _sequenceID;
  header.stamp = ros::Time(data->timestamp());
  header.frame_id = imageSensor->frame()->name();
  if (imageBlob->cvImage().type() == 16) {
    encoding = "rgb8";
  }
  else if (imageBlob->cvImage().type() == 2) {
    encoding = "16UC1";
  }
  else {
    cerr << "WARNING: image format to publish not recognized, skipping" << endl;
    return;
  }
  cvImage = cv_bridge::CvImage(header, encoding, imageBlob->cvImage());

  // Publish messages
  _cameraInfoPublisher.publish(cameraInfo);
  _publisher.publish(cvImage.toImageMsg());
  _sequenceID++;

  delete(imageBlob);
}
