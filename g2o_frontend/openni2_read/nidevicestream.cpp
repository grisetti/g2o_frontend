#include "nidevicestream.h"
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
using namespace std;
//#include "../Common/OniSampleUtilities.h"
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <stdint.h>

using namespace std;
using namespace openni;



DeviceStream::DeviceStream(Device* device_){
  _device = device_;
  _modeIdx = -1;
  _sensorType = SENSOR_DEPTH;
  _ref = 0;
}


DeviceStream::~DeviceStream(){
  stop();
  cleanup();
}
  
bool DeviceStream::init(SensorType sensorType_, int modeIdx_){
  _modeIdx = modeIdx_;
  _sensorType = sensorType_;
  if (! _device)
    return false;
    
  const SensorInfo* sensorInfo = _device->getSensorInfo(sensorType_);
  if (! sensorInfo) {
    cerr << "Device :" << _device->getDeviceInfo().getUri()  << " unsupported sensor:" << _sensorType << endl;
    return false;
  }
    
  const Array<VideoMode>& videoModes = sensorInfo->getSupportedVideoModes();
  if (_modeIdx < 0 || _modeIdx >= videoModes.getSize()){
    cerr << "unspoorted video mode idx: " << _modeIdx << "for for sensor: " << _sensorType;
  }
    
  _stream = new VideoStream;
  Status rc = _stream->create(*_device, _sensorType);

  if (rc != STATUS_OK) 
    {
      cerr << "Device :" << _device->getDeviceInfo().getUri()  << " unnable to create the stream for sensor" << _sensorType << endl;
      cleanup();
      return false;
    }
    
  _stream->setVideoMode(videoModes[_modeIdx]);
  if (rc != STATUS_OK) 
    {
      cerr << "Device :" << _device->getDeviceInfo().getUri()  << " unnable to set video mode " << _modeIdx << " for the sensor:" << _sensorType << endl;
      cleanup();
      return false;
    } else {
    cerr << "Device :" << _device->getDeviceInfo().getUri()  << " succesfully set video mode " << _modeIdx << " for the sensor:" << _sensorType << endl;
  }

  _stream->setMirroringEnabled(false);

  float hFov = _stream->getHorizontalFieldOfView();
  float vFov = _stream->getVerticalFieldOfView();
  int minPixelValue = _stream->getMinPixelValue();
  int maxPixelValue = _stream->getMaxPixelValue();
  int horizontalRes = videoModes[_modeIdx].getResolutionX();
  int verticalRes = videoModes[_modeIdx].getResolutionY();
  cerr << "Fov: " << hFov <<"," << vFov 
       << " val: " << minPixelValue << "," << maxPixelValue
       << " res: " << horizontalRes << "x" << verticalRes << endl;
  cerr << " K: " << .5*horizontalRes / tan(hFov/2) 
       << " " << .5*verticalRes / tan(vFov/2)
       << " " << horizontalRes/2
       << " " << verticalRes/2;
  float _cameraMatrix[] = {
    .5*horizontalRes / tan(hFov/2), 0,                            horizontalRes/2,
    0,                              .5*verticalRes / tan(vFov/2), verticalRes/2,
    0, 0, 1
  };
  for(int i=0; i<9; i++)
    cameraMatrix[i] = _cameraMatrix[i];
  return true;
}

bool DeviceStream::start(){
  if (! _stream)
    return false;
  Status rc = _stream->start();
  if (rc != STATUS_OK) 
    {
      cerr << "Device :" << _device->getDeviceInfo().getUri()  << " unnable to start the stream for sensor" << _sensorType << endl;
      return false;
    }
  cerr << "Device :" << _device->getDeviceInfo().getUri()  << " stream started for sensor" << _sensorType << endl;
  return true;
}


bool DeviceStream::stop(){
  if (! _stream)
    return false;
  _stream->stop();
  cerr << "Device :" << _device->getDeviceInfo().getUri()  << " stream stopped for sensor" << _sensorType << endl;
  return true;
}

void DeviceStream::cleanup(){
  if (_stream){
    _stream->stop();
    _stream->destroy();
    _stream = 0;
  }
  if (_ref){
    delete _ref;
    _ref = 0;
  }
  _modeIdx = -1;
}

