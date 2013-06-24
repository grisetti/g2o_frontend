#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include <vector>
#include <map>
#include <queue>
#include <cmath>
#include <zlib.h>
#include <pthread.h>
#include <algorithm>

#include <OpenNI2/OpenNI.h>

#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <stdint.h>

#include "niframe.h"
#include "nidevicewithstreams.h"

using namespace std;
using namespace openni;


const char* fmt2string(const PixelFormat fmt){
    switch(fmt){
    case PIXEL_FORMAT_DEPTH_1_MM: return "DEPTH_1_MM"; 	
    case PIXEL_FORMAT_DEPTH_100_UM: return "DEPTH_100_UM"; 
    case PIXEL_FORMAT_SHIFT_9_2: return "SHIFT_9_2";  	
    case PIXEL_FORMAT_SHIFT_9_3: return "SHIFT_9_3";  	
    case  PIXEL_FORMAT_RGB888: 	 return "RGB888";  
    case PIXEL_FORMAT_YUV422:    return "YUV422"; 	
    case PIXEL_FORMAT_GRAY8: 	 return "GRAY8"; 
    case PIXEL_FORMAT_GRAY16: 	 return "GRAY16"; 
    case PIXEL_FORMAT_JPEG: 	 return "JPEG"; 
    default: return 0;
    }
}

int DeviceWithStreams::printDeviceInfo(Device& device) {
  const DeviceInfo& deviceInfo = device.getDeviceInfo();
  Status rc;
  cout  << "device name: [" << deviceInfo.getName() << "]" << endl;
  cout  << "uri        : [" << deviceInfo.getUri() << "]" << endl;
  

  const SensorInfo* irInfo = device.getSensorInfo(SENSOR_IR);
  if (irInfo){
    cout << "  IR:" << endl;
    const Array<VideoMode>& irVideoModes = irInfo->getSupportedVideoModes();
    for (int i = 0; i< irVideoModes.getSize(); i++){
      cout << "  id: " << i 
	   << " res: " << irVideoModes[i].getResolutionX() << "x" << irVideoModes[i].getResolutionY() 
	   << " fps: " << irVideoModes[i].getFps()  
	   << " fmt: " << fmt2string(irVideoModes[i].getPixelFormat())  << endl;
    }
  }
  
  const SensorInfo* depthInfo = device.getSensorInfo(SENSOR_DEPTH);
  if (depthInfo) {
    cout << "  Depth:" << endl;
    const Array<VideoMode>& depthVideoModes = depthInfo->getSupportedVideoModes();
    for (int i = 0; i< depthVideoModes.getSize(); i++){
      cout << "  id: " << i 
	   << " res: " << depthVideoModes[i].getResolutionX() << "x" << depthVideoModes[i].getResolutionY() 
	   << " fps: " << depthVideoModes[i].getFps()  
	   << " fmt: " << fmt2string(depthVideoModes[i].getPixelFormat())  << endl;
    }
  }

  const SensorInfo* colorInfo = device.getSensorInfo(SENSOR_COLOR);
  if (colorInfo){
    cout << "  Color:" << endl;
    const Array<VideoMode>& colorVideoModes = colorInfo->getSupportedVideoModes();
    for (int i = 0; i< colorVideoModes.getSize(); i++){
      cout << "  id: " << i 
	   << " res: " << colorVideoModes[i].getResolutionX() << "x" << colorVideoModes[i].getResolutionY() 
	   << " fps: " << colorVideoModes[i].getFps()  
	   << " fmt: " << fmt2string(colorVideoModes[i].getPixelFormat())  << endl;
    }
  }

  return 0;
}

void DeviceWithStreams::listModes(){
  Array<DeviceInfo> deviceInfoList;
  OpenNI::enumerateDevices(&deviceInfoList);
  printf("OpenNI, found %d devices\n",deviceInfoList.getSize());
  Device device;
  Status rc;
  for (int i=0; i<deviceInfoList.getSize(); i++){
    rc = device.open(deviceInfoList[i].getUri());
    if (rc != STATUS_OK) 
      {
	printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
	continue;
      }
    printDeviceInfo(device);
    device.close();
  }
}



DeviceWithStreams::DeviceWithStreams(Device* device_){
  _device = device_;
}

DeviceWithStreams::~DeviceWithStreams(){
  eraseAllStreams();
}

bool DeviceWithStreams::addStream(SensorType sensorType, int modeIdx){
  removeStream(sensorType);
  DeviceStream* stream= new DeviceStream(_device);
  if (! stream->init(sensorType, modeIdx)){
    delete stream;
    return false;
  }
  _streams.insert(make_pair(sensorType,stream));
  return true;
}

bool DeviceWithStreams::removeStream(SensorType sensorType){
  std::map<SensorType, DeviceStream*>::iterator it = _streams.find(sensorType);
  if (it!=_streams.end()){
    delete it->second;
    _streams.erase(it);
    return true;
  } 
  return false; 
}

DeviceStream* DeviceWithStreams::getStream(SensorType sensorType){
  std::map<SensorType, DeviceStream*>::iterator it = _streams.find(sensorType);
  if (it!=_streams.end()){
    return it->second;
  } 
  return 0;
}

void DeviceWithStreams::eraseAllStreams(){
  for(std::map<SensorType, DeviceStream*>::iterator it = _streams.begin(); it!=_streams.end(); it++)
    delete it->second;
  _streams.clear();
}
  
void DeviceWithStreams::startAllStreams(){
  cerr << "Device :" << _device->getDeviceInfo().getUri()  
       <<  " staring " << _streams.size() << " streams" << endl;
  for(std::map<SensorType, DeviceStream*>::iterator it = _streams.begin(); it!=_streams.end(); it++)
    it->second->start();
}

void DeviceWithStreams::stopAllStreams(){
  for(std::map<SensorType, DeviceStream*>::iterator it = _streams.begin(); it!=_streams.end(); it++)
    it->second->stop();
}

