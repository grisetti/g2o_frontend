#include "niframe.h"
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

gzFile open_dump(const char *filename)
{
  gzFile fp = gzopen(filename, "w9f");
  if (fp == NULL) {
    fprintf(stderr, "Error: Cannot open file [%s]\n", filename);
    exit(1);
  }
  //printf("%s\n", filename);
  return fp;
}

void dump_gray16(gzFile fp, void *data, unsigned int width, unsigned int height)
{
  unsigned char *c=(unsigned char*)data;
  unsigned char *cend=c+width * height * 2;

  gzprintf(fp, "P5 %d %d 65535\n", width, height);
  // correct the endianness
  for (;c<cend; c+=2){
    unsigned char d=*c;
    *c=*(c+1);
    *(c+1)=d;
  }
  gzwrite(fp, data, width * height * 2);
}

void dump_gray8(gzFile fp, void *data, unsigned int width, unsigned int height)
{
  unsigned char *c=(unsigned char*)data;
  unsigned char *cend=c+width * height * 2;

  gzprintf(fp, "P5 %d %d 255\n", width, height);
  gzwrite(fp, data, width * height);
}


void dump_rgb888(gzFile fp, void *data, unsigned int width, unsigned int height)
{
  unsigned char *c=(unsigned char*)data;
  unsigned char *cend=c+width * height;

  gzprintf(fp, "P6 %d %d \n 255\n", width, height);
  gzwrite(fp, data, width * height * 3);
}

void dump_rgb888_gray8(gzFile fp, void *data, unsigned int width, unsigned int height)
{
  unsigned char *c=(unsigned char*)data;
  unsigned char *d=(unsigned char*)data;
  unsigned char *cend=c+width * height*3;

  gzprintf(fp, "P5 %d %d \n255\n", width, height);
  for (;c<cend; c+=3, d++){
    unsigned int acc = *c + *(c+1) + *(c+2);
    *d = acc/3;
  }
  // correct the endianness
  gzwrite(fp, data, width * height);
}

void dump(gzFile fp, void* data, unsigned int width, unsigned int height, SensorType sensorType, int bpp){
  if (sensorType==SENSOR_DEPTH){
    dump_gray16(fp, data, width, height);
  } else
  if (sensorType==SENSOR_COLOR){
    if (bpp==1)
      dump_gray8(fp, data, width, height);
    else if (bpp==3)
      dump_rgb888(fp, data, width, height);
  } 

}

Frame::Frame(int id, const uint8_t* buf, int width, int height, int bpp, uint64_t timestamp,
	     SensorType type, const string& uri, float* cameraMatrix) {
  this->width = width;
  this->height = height;
  this->id = id;
  this->bpp = bpp;
  this->buf = new uint8_t[width*height*bpp];
  this->timestamp = timestamp;
  memcpy(this->buf, buf, width*height*bpp);
  this->type = type;
  this->uri = uri;
  memcpy(this->cameraMatrix, cameraMatrix, 9*sizeof(float));
}
  

Frame::~Frame(){
  delete [] buf;
}

string Frame::getSaveFilename(const char* prefix) const{
  char _name[1024];
  sprintf(_name,"%s-%s-%02d-%05d.pgm.gz",prefix, uri.c_str(), type, id);
  string name=_name;
  std::replace(name.begin(), name.end(), '/','_');
  return name;
}

string Frame::getLogLine(const char* prefix) const {
  ostringstream log;
  log << "OpenNiFrame { ";
  log << "Timestamp: " << timestamp << "; ";
  log << "Seq: " << id << "; ";
  log << "uri: \"" << uri << "\"; ";
  log << "Type: " << type << "; ";
  log << "CameraMatrix: [";
  for (int i=0; i<9; i++)
    log << cameraMatrix[i] << " ";
  log << "]; ";
  log << "Width: " << width << "; ";
  log << "Height: " << height << "; ";
  log << "Bpp: " << bpp << "; ";
  log << "Filename \"" << getSaveFilename(prefix) << "\"; }";
  return log.str();
}


void Frame::save(const char* prefix) const{
  const char* name=getSaveFilename(prefix).c_str();
  gzFile fp=open_dump(name);
  dump(fp, buf, width, height, type, bpp);
  gzclose(fp);
}
