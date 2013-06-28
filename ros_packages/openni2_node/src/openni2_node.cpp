#include <vector>
#include <map>
#include <queue>
#include <cmath>
#include <zlib.h>

#include <OpenNI2/OpenNI.h>
using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <tf/transform_broadcaster.h>

using namespace openni;

// TODO: 
// - add dynamic selection of the video modes for each device (dynamic reconfigure)

struct SensorTopic {
  SensorTopic(const std::string & deviceTopicName, ros::NodeHandle& nh,
	      image_transport::ImageTransport& transport, 
	      openni::Device* device, openni::SensorType sensorType){
    
    this->deviceTopicName = deviceTopicName;
    this->deviceFrameId = deviceTopicName+"/base_frame";

    sensorTransform.setOrigin(tf::Vector3(0,0,0));
    sensorTransform.setRotation(tf::Quaternion(0.5,-0.5,0.5,-0.5)); // z axis in front, x to the left, y down
    switch(sensorType){
    case SENSOR_DEPTH: 
      sensorTopicName = deviceTopicName +"/depth";
      sensorFrameId = deviceTopicName +"/depth_frame";
      sensorTransform.setOrigin(tf::Vector3(0,-0.03,0));
      break;
    case SENSOR_COLOR:  
      sensorTopicName = deviceTopicName +"/color";
      sensorFrameId  = deviceTopicName +"/color_frame";
      sensorTransform.setOrigin(tf::Vector3(0,0.03,0));
      break;
    default:;
    }

    imageTopicName = sensorTopicName + "/image";
    this->device = device;
    this->sensorType = sensorType;

    publisher = transport.advertiseCamera(imageTopicName, 1);
    openni::Status rc =videoStream.create(*device,sensorType);
    cerr << "CREATING STREAM: " << imageTopicName; 
    if (rc != openni::STATUS_OK) {
	cerr << " FAIL, error: " << openni::OpenNI::getExtendedError() << endl;
      ok=false;
    } else {
      cerr << " SUCCESS" << endl;
      ok=true;
    }
    started = false;
    seq = 0;

    switch(sensorType){
    case SENSOR_DEPTH: setVideoMode(4);  break; //HACK
    case SENSOR_COLOR: setVideoMode(6);  break; //HACK
    default:;
    }
    
  }
  
  bool setVideoMode(int idx){
    stop();
    const Array<VideoMode>& videoModes = device->getSensorInfo(sensorType)->getSupportedVideoModes();
    cerr << "SETTING VIDEO MODE STREAM: " << imageTopicName; 
    Status rc = videoStream.setVideoMode(videoModes[idx]);
    if (rc != openni::STATUS_OK) {
      cerr << " FAIL, error: " << openni::OpenNI::getExtendedError() << endl;
      return false;
    } else {
      cerr << " SUCCESS" << endl;
    }
    start();
    return true;
  }
  
  bool start(){
    if (! started && ok) {
      cerr << "STARTING STREAM: " << imageTopicName; 
      openni::Status rc = videoStream.start();
      if (rc != openni::STATUS_OK) {
	cerr << " FAIL, error: " << openni::OpenNI::getExtendedError() << endl;
	return false;
      } else {
	cerr << " SUCCESS" << endl;
	started = true;
	return true;
      }
    }
    return false;
  }

  ~SensorTopic() {
    cerr << "CALLING DESTRUCTOR: " << imageTopicName << endl;
    stop();
    videoStream.destroy();
  }

  void stop() {
    cerr << "STOPPING STREAM: " << imageTopicName << endl; 
    videoStream.stop();
    started = false;
  }

  void handleRequests() {
    if (publisher.getNumSubscribers()>0 && ! started)
      start();
    if (publisher.getNumSubscribers()==0 && started)
      stop();
  }
  
  void publishFrame(){
    VideoStream& vs = videoStream;
    vs.readFrame(&ref);
    ros::Time currentTime = ros::Time::now();
    // image publishing
    
    image.header.seq = seq;
    image.header.stamp=currentTime;
    image.header.frame_id=sensorFrameId;
    if(ref.getSensorType() == SENSOR_DEPTH){
      image.encoding = "mono16";
    } else if(ref.getSensorType() == SENSOR_COLOR){
      image.encoding = "rgb8";
    }
    image.width = ref.getWidth();
    image.height = ref.getHeight();
    image.step = ref.getStrideInBytes();
    image.data.resize(ref.getDataSize());
    memcpy(&image.data[0] ,ref.getData(), ref.getDataSize());
    // camera info publishing
    float hFov = videoStream.getHorizontalFieldOfView();
    float vFov = videoStream.getVerticalFieldOfView();
    int minPixelValue = videoStream.getMinPixelValue();
    int maxPixelValue = videoStream.getMaxPixelValue();
    const VideoMode& videoMode = ref.getVideoMode();
    int horizontalRes = videoMode.getResolutionX();
    int verticalRes = videoMode.getResolutionY();
    float _cameraMatrix[] = {
      .5*horizontalRes / tan(hFov/2), 0,                            horizontalRes/2,
      0,                              .5*verticalRes / tan(vFov/2), verticalRes/2,
      0, 0, 1
    };

    cameraInfo.header=image.header;
    cameraInfo.height  = image.height;
    cameraInfo.width = image.width;
    cameraInfo.distortion_model = "plumb_bob";
    cameraInfo.D.resize(5);
    for (int i=0; i<5; i++)
      cameraInfo.D[i] = 0;
    for (int i=0; i<9; i++)
      cameraInfo.K[i] = _cameraMatrix [i];
    
    cameraInfo.binning_x = 1;
    cameraInfo.binning_y = 1;
    ref.release();
    publisher.publish(image,cameraInfo,currentTime);
    seq ++;
  }

  void sendTransform(tf::TransformBroadcaster br) {
    br.sendTransform(tf::StampedTransform(sensorTransform, ros::Time::now(), deviceFrameId, sensorFrameId));
  }

  std::string deviceTopicName;
  std::string sensorTopicName;
  std::string imageTopicName;
  std::string cameraInfoTopicName;
  std::string sensorFrameId;
  std::string deviceFrameId;
  openni::Device* device;
  openni::SensorType sensorType;
  openni::VideoStream videoStream;
  image_transport::CameraPublisher publisher;
  sensor_msgs::Image image;
  sensor_msgs::CameraInfo cameraInfo;
  VideoFrameRef ref;
  bool ok;
  bool started;
  int seq;
  tf::Transform sensorTransform;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "openni2_node");
  ros::NodeHandle nh;
  openni::Status rc = openni::STATUS_OK;
  rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK)
    {
      printf("%s: Initialize failed\n%s\n", argv[0], openni::OpenNI::getExtendedError());
      return 1;
    }

  image_transport::ImageTransport it(nh);
  openni::Array<openni::DeviceInfo> deviceList;
  openni::OpenNI::enumerateDevices(&deviceList);

  tf::TransformBroadcaster br;
  std::vector<openni::Device*> devices;
  std::vector<SensorTopic*> sensorTopics;
  // for each device, query the sensors
  for (int i=0; i<deviceList.getSize(); i++) {
    const DeviceInfo& deviceInfo=deviceList[i];
    cerr << "opening device " << i << " uri:[" << deviceInfo.getUri() << "]" << endl;
    Device* device = new Device();
    rc = device->open(deviceInfo.getUri());
    if (rc != openni::STATUS_OK) {
      printf("%s: Initialize failed\n%s\n", argv[0], openni::OpenNI::getExtendedError());
    } else {
      devices.push_back(device);
    }
    cerr << "ok" << endl;
    cerr << " device: [" << deviceInfo.getUri() << "]"   << endl;
    char buf[1024];
    if (devices[i]->hasSensor(SENSOR_DEPTH)){
      sprintf(buf, "/xtion_%02d",i);
      sensorTopics.push_back(new SensorTopic(buf, nh, it, devices[i], SENSOR_DEPTH));
    }
    if (devices[i]->hasSensor(SENSOR_COLOR)){
      sprintf(buf, "/xtion_%02d",i);
      sensorTopics.push_back(new SensorTopic(buf, nh, it, devices[i], SENSOR_COLOR));
    }
  }
  bool startOk = true;
  VideoStream** streams;
  int * reverseIndices;
  streams = new VideoStream*[sensorTopics.size()];
  reverseIndices = new int[sensorTopics.size()];
  for (size_t i = 0; i<sensorTopics.size(); i++){
    streams[i] = &(sensorTopics[i]->videoStream);
  }

  int seq = 0;
  VideoFrameRef ref;
  while (startOk && nh.ok()) {
    int k=0;
    for (size_t i = 0; i<sensorTopics.size(); i++){
      sensorTopics[i]->handleRequests();
      sensorTopics[i]->sendTransform(br);
      if(sensorTopics[i]->started){
	reverseIndices[k] = i;
	streams[k] = &sensorTopics[i]->videoStream;
	k++;
      }
    }
    if (k) {
      int changedIndex=-1;
      openni::Status rc = openni::OpenNI::waitForAnyStream(streams, k, &changedIndex, openni::TIMEOUT_FOREVER);
      if (rc != openni::STATUS_OK) {
	cerr << "WAIT FAILED, error: " << openni::OpenNI::getExtendedError() << endl;
	return 1;
      }
      int topicIndex=reverseIndices[changedIndex];
      SensorTopic* sensorTopic=sensorTopics[topicIndex];
      cerr << changedIndex;
      sensorTopic->publishFrame();
      //ros::Duration(0.1).sleep();
    } else {
      // sleep 10 ms
      ros::Duration(0.1).sleep();
    }
    ros::spinOnce();
  }
  cerr << "SHUTTING DOWN" << endl;
  for (size_t i=0; i<sensorTopics.size(); i++){
    delete sensorTopics[i];
  }
  delete [] streams;
  for (size_t i=0; i<devices.size(); i++) {
    if(devices[i]) {
      cerr << "CLOSING DEVICE: " << devices[i]->getDeviceInfo().getUri() << endl; 
      devices[i]->close();
    }
    delete devices[i];
  }

  OpenNI::shutdown();
}
