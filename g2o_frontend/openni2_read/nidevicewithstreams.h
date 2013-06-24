#ifndef _NI_DEVICEWITHSTREAMS_H_
#define _NI_DEVICEWITHSTREAMS_H_

#include "nidevicestream.h"

struct DeviceWithStreams{
  DeviceWithStreams(openni::Device* device_);
  ~DeviceWithStreams();
  bool addStream(openni::SensorType sensorType, int modeIdx);
  bool removeStream(openni::SensorType sensorType);
  DeviceStream* getStream(openni::SensorType sensorType);
  void eraseAllStreams();  
  void startAllStreams();  
  void stopAllStreams();
  static int printDeviceInfo(openni::Device& device);
  static void listModes();
  openni::Device* _device;
  std::map<openni::SensorType, DeviceStream*> _streams;
};


#endif
