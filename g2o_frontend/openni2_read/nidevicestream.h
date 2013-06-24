#ifndef _NI_DEVICE_STREAM_H_
#define _NI_DEVICE_STREAM_H_

#include <OpenNI2/OpenNI.h>

struct DeviceStream{
  DeviceStream(openni::Device* device_);
  ~DeviceStream();  
  bool init(openni::SensorType sensorType_, int modeIdx_);
  bool start();
  bool stop();
  void cleanup();
  openni::Device* _device;
  openni::VideoStream* _stream;
  openni::VideoFrameRef* _ref;
  int _modeIdx;
  openni::SensorType _sensorType;
  float cameraMatrix[9];
};


#endif
