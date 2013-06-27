/*****************************************************************************
 *                                                                            *
 *  OpenNI 2.x Alpha                                                          *
 *  Copyright (C) 2012 PrimeSense Ltd.                                        *
 *                                                                            *
 *  This file is part of OpenNI.                                              *
 *                                                                            *
 *  Licensed under the Apache License, Version 2.0 (the "License");           *
 *  you may not use this file except in compliance with the License.          *
 *  You may obtain a copy of the License at                                   *
 *                                                                            *
 *      http://www.apache.org/licenses/LICENSE-2.0                            *
 *                                                                            *
 *  Unless required by applicable law or agreed to in writing, software       *
 *  distributed under the License is distributed on an "AS IS" BASIS,         *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
 *  See the License for the specific language governing permissions and       *
 *  limitations under the License.                                            *
 *                                                                            *
 *****************************************************************************/

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

using namespace openni;

struct SensorTopic {
  SensorTopic(const std::string & deviceTopic, image_transport::ImageTransport& transport, 
	      openni::Device* device, openni::SensorType sensorType){
    this->deviceTopic = deviceTopic;
    imageTopicName = deviceTopic + "/image";
    cameraInfoTopicName = deviceTopic + "/cameraInfo";
    this->device = device;
    this->sensorType = sensorType;
    imagePublisher = transport.advertise(imageTopicName, 1);
    cameraInfoPublisher = transport.advertise(cameraInfoTopicName, 1);
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
    if (imagePublisher.getNumSubscribers()>0 && ! started)
      start();
    if (imagePublisher.getNumSubscribers()==0 && started)
      stop();
  }
  
  std::string deviceTopic;
  std::string imageTopicName;
  std::string cameraInfoTopicName;
  openni::Device* device;
  openni::SensorType sensorType;
  openni::VideoStream videoStream;
  image_transport::Publisher imagePublisher;
  image_transport::Publisher cameraInfoPublisher;
  bool ok;
  bool started;
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
      sprintf(buf, "/xtion_%02d/depth",i);
      sensorTopics.push_back(new SensorTopic(buf, it, devices[i], SENSOR_DEPTH));
    }
    if (devices[i]->hasSensor(SENSOR_COLOR)){
      sprintf(buf, "/xtion_%02d/color",i);
      sensorTopics.push_back(new SensorTopic(buf, it, devices[i], SENSOR_COLOR));
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

  // cv::WImageBuffer3_b image( cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR) );
  // sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(), "bgr8");
  
  int seq = 0;
  VideoFrameRef ref;
  while (startOk && nh.ok()) {
    int k=0;
    for (size_t i = 0; i<sensorTopics.size(); i++){
      sensorTopics[i]->handleRequests();
      if(sensorTopics[i]->started){
	reverseIndices[k] = i;
	streams[k] = &sensorTopics[i]->videoStream;
	k++;
      }
    }
    if (k) {
      int changedIndex;
      openni::Status rc = openni::OpenNI::waitForAnyStream(streams, k, &changedIndex);
      if (rc != openni::STATUS_OK) {
	cerr << "WAIT FAILED, error: " << openni::OpenNI::getExtendedError() << endl;
	return 1;
      }
      int topicIndex=reverseIndices[changedIndex];
      SensorTopic* sensorTopic=sensorTopics[topicIndex];
      VideoStream& vs = sensorTopic->videoStream;
      vs.readFrame(&ref);
      int bpp=0;
      sensor_msgs::Image image;
      image.header.seq = seq++;
      image.header.stamp=ros::Time::now();
      if(ref.getSensorType() == SENSOR_DEPTH){
	image.header.frame_id=sensorTopic->deviceTopic +"/depth_frame";
      	image.encoding = "mono16";
	bpp = 2;
      } else if(ref.getSensorType() == SENSOR_COLOR){
	image.header.frame_id=sensorTopic->deviceTopic +"/rgb_frame";
	bpp = 3;
	image.encoding = "bgr8";
      }
      image.width = ref.getWidth();
      image.height = ref.getHeight();
      image.step = ref.getStrideInBytes();
      image.data.resize(ref.getDataSize());
      if (ref.getWidth() * ref.getHeight() * bpp!=ref.getDataSize()) {
	cerr << "fatal error, size mismatch" << ref.getWidth() * ref.getHeight() * bpp << "!=" << ref.getDataSize() << endl; 
      } else {
	memcpy(&image.data[0] ,ref.getData(), ref.getDataSize());
	sensorTopic->imagePublisher.publish(image);
      }
      
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
