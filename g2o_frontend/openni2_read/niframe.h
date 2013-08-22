#ifndef _NIFRAME_H
#define _NIFRAME_H_

#include <OpenNI.h>
#include <string>
#include <cstring>
#include <stdint.h>

struct Frame{
  Frame(int id, const uint8_t* buf, int width, int height, int bpp, uint64_t timestamp,
	openni::SensorType type, const std::string& uri, float* cameraMatrix) ;  

  ~Frame();

  std::string getSaveFilename(const char* prefix) const;

  std::string getLogLine(const char* prefix) const;

  void save(const char* prefix) const;  

  int id;
  openni::SensorType type;
  std::string uri;
  float cameraMatrix[9];
  uint8_t* buf;
  int width;
  int height;
  int bpp;
  uint64_t timestamp;

};

#endif
