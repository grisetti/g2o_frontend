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

#include "niframe.h"
#include "nidevicestream.h"
#include "nidevicewithstreams.h"
#include "nisaverthread.h"

int wasKeyboardHit()
{
        struct termios oldt, newt;
        int ch;
        int oldf;

        // don't echo and don't wait for ENTER
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        
        // make it non-blocking (so we can check without waiting)
        if (0 != fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK))
        {
                return 0;
        }

        ch = getchar();

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        if (0 != fcntl(STDIN_FILENO, F_SETFL, oldf))
        {
                return 0;
        }

        if(ch != EOF)
        {
                ungetc(ch, stdin);
                return 1;
        }

        return 0;
}

using namespace openni;

int main(int argc, char ** argv)
{
	
  Status rc = OpenNI::initialize();
  if (rc != STATUS_OK)
    {
      printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
      return 1;
    }
  DeviceWithStreams::listModes();
  
  if (argc < 2 ){
    OpenNI::shutdown();
    return 0;
  }

  int depthMode = 0;
  int colorMode = 0;
  std::string prefix="";
  int c = 1;
  while (c<argc){
    if (!strcmp(argv[c],"-d") && c<argc-1){
      c++;
      depthMode = atoi(argv[c]);
    }
    if (!strcmp(argv[c],"-c") && c<argc-1){
      c++;
      colorMode = atoi(argv[c]);
    }
    if ( !strcmp(argv[c],"-o") && c<argc-1){
      c++;
      prefix = argv[c];
    }
    c++;
  }
  
  cerr << "depth mode: " << depthMode 
       << " color mode: " << colorMode  
       << " filename: " << prefix << endl;

  ofstream log;
  if (prefix.length()==0){
    cerr << "no filename supplied, aborting" << endl;
    OpenNI::shutdown();
    return 0;
  } else {
    string logName = prefix+".log";
    log.open(logName.c_str());
    if (! log.good()){
      cerr << "cannot open log file [" << logName << "] aborting" << endl;
      OpenNI::shutdown();
      return 0;
    }
  }
  
 
  
  Device device;
  rc = device.open(ANY_DEVICE);
  if (rc != STATUS_OK) {
    cerr << "Couldn't open the device" << endl;
    return 0;
  }

  rc = device.setDepthColorSyncEnabled(true);
  if (rc != STATUS_OK) {
    cerr << "Couldn't set the depth synchronization" << endl;
    return 0;
  }


  
  DeviceWithStreams deviceStreams(&device);
  deviceStreams.addStream(SENSOR_DEPTH, depthMode);
  deviceStreams.addStream(SENSOR_COLOR, colorMode);
  deviceStreams.startAllStreams();

  int framecount = 0;
  struct timeval tStart, tEnd;
  VideoFrameRef ref;

  

  cerr << "starting the saver thread" << endl;
  std::vector<SaverThread*> saverThreads;
  int numSavers = 4;
  for(int i=0; i<numSavers; i++){
    SaverThread* saver= new SaverThread(prefix);
    saverThreads.push_back(saver);
    saver->start();
  }

  while (!wasKeyboardHit()){
    for (std::map<SensorType,DeviceStream*>::iterator it=deviceStreams._streams.begin();
	 it!=deviceStreams._streams.end(); it++){
      DeviceStream* stream = it->second;
      SensorType sensorType = it->first;
      rc = stream->_stream->readFrame(&ref);
      if (rc != STATUS_OK)
	{
	  printf("Wait failed\n");
	  continue;
	}
      if (! framecount) {
	gettimeofday(&tStart, 0);
      }
      int bpp = 2;
      switch(it->first) {
      case SENSOR_DEPTH: bpp=2; break;
      case SENSOR_COLOR: bpp=3; break;
      default: bpp=1;
      }

      Frame* frame = new Frame(framecount, (uint8_t*)ref.getData(), ref.getWidth(), ref.getHeight(), bpp, ref.getTimestamp(), sensorType, device.getDeviceInfo().getUri(), stream->cameraMatrix);

      ref.release();

      log << frame->getLogLine(prefix.c_str()) << endl;
      int numThread = framecount%numSavers;
      saverThreads[numThread]->addInQueue(frame);
    }
    framecount ++;
  }
  for(int i=0; i<numSavers; i++){
    saverThreads[i]->stop();
  }
  gettimeofday(&tEnd,0);
  double t1 = tStart.tv_sec*1000 + tStart.tv_usec*0.001;
  double t2 = tEnd.tv_sec*1000 + tEnd.tv_usec*0.001;
  cerr << "real fps: " << 1000*framecount/(t2-t1) << endl;
  log.close();
  deviceStreams.eraseAllStreams();
  device.close();
  OpenNI::shutdown();
  return 0;
}
