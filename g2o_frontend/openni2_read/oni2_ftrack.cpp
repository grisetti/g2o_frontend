#include <pthread.h>
#include <semaphore.h>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <OpenNI.h>
#include <fstream>

using namespace cv;
using namespace openni;
using namespace std;

std::string makeFilename(std::string filename, int index){
  char buf[1024];
  sprintf(buf, "%s-%06d.pgm",filename.c_str(), index);
  return buf;
}

bool saveFrame(std::string filename, int index, const Mat& frame){
  char buf[1024];
  if (frame.type()==CV_16U){
    sprintf(buf, "%s-%06d.pgm",filename.c_str(), index);
    cv::imwrite(buf, frame);
    return true;
  }
  if (frame.type()==CV_8UC3){
    sprintf(buf, "%s-%06d.pbm",filename.c_str(), index);
    cv::imwrite(buf, frame);
    return true;
  }
  return false;
} 

struct NiWrapper{
  Device device;        // Software object for the physical device i.e.
  // PrimeSense Device Class
  VideoStream depth;       // IR VideoStream Class Object
  VideoFrameRef depthf;    //IR VideoFrame Class Object
  VideoStream rgb;       // IR VideoStream Class Object
  VideoFrameRef rgbf;    //IR VideoFrame Class Object

  VideoMode depthMode;      // VideoMode Object
  VideoMode colorMode;      // VideoMode Object
  bool depthRegistration;
  
  volatile bool running;
  volatile int paramChanged;

  string filename;

  Mat depthFrame;              // OpenCV Matrix Object, also used to store images
  Mat rgbFrame;              // OpenCV Matrix Object, also used to store images

  Mat depthFrameCopy;              // OpenCV Matrix Object, also used to store images
  Mat rgbFrameCopy;              // OpenCV Matrix Object, also used to store images

  pthread_t thread;
  pthread_mutex_t mutex;
  sem_t sem;
  void setDepthRegistration(bool dr) {
    depthRegistration = dr;
    paramChanged = true;
  }

  NiWrapper(){
    paramChanged = false;
    depthRegistration = true;
    filename = "out";
    running = false;
    pthread_mutex_init(&mutex,0);
    sem_init(&sem,0,0);
  }
 
  ~NiWrapper(){
    stop();
    pthread_mutex_destroy(&mutex);
    sem_destroy(&sem);
  }

  void getFrame(cv::Mat& depth, cv::Mat& color){
    pthread_mutex_lock(&mutex);
    depth=depthFrame.clone();
    cvtColor(rgbFrame, color,CV_BGR2RGB);
    pthread_mutex_unlock(&mutex);
  }

  void waitForFrame() {
    sem_wait(&sem);
  }

  int start(){
    Status rc = STATUS_OK;
    rc = openni::OpenNI::initialize();    // Initialize OpenNI
    rc = device.open(openni::ANY_DEVICE); // Open the Device

    rc = device.setDepthColorSyncEnabled(true);
    if (rc != STATUS_OK) {
      cerr << "Couldn't set the depth synchronization" << endl;
      return 0;
    }
    rc = device.setDepthColorSyncEnabled(true);
    if (rc != STATUS_OK) {
      cerr << "Couldn't set the depth synchronization" << endl;
    return 0;
    }

    rc = depth.create(device, SENSOR_DEPTH);    // Create the VideoStream for IR
    if (rc != STATUS_OK) {
      cerr << "Couldn't create the depth stream" << endl;
      return 0;
    }
    rc = depth.setMirroringEnabled(false);
    if (rc != STATUS_OK) {
      cerr << "Couldn't disable the mirroring in depth stream" << endl;
      return 0;
    }

    rc = rgb.create(device, SENSOR_COLOR);    // Create the VideoStream for IR
    if (rc != STATUS_OK) {
      cerr << "Couldn't create the rgb stream" << endl;
      return 0;
    }
    rc = rgb.setMirroringEnabled(false);
    if (rc != STATUS_OK) {
      cerr << "Couldn't disable the mirroring in rgb stream" << endl;
      return 0;
    }
  
    const SensorInfo* sensorInfo = device.getSensorInfo(SENSOR_DEPTH);
    const Array<VideoMode>& depthVideoModes = sensorInfo->getSupportedVideoModes();
    rc = depth.setVideoMode(depthVideoModes[5]);
    if (rc != STATUS_OK) {
      cerr << "Couldn't set the desired depth video mode" << endl;
      return 0;
    }
  
    sensorInfo = device.getSensorInfo(SENSOR_COLOR);
    const Array<VideoMode>& colorVideoModes = sensorInfo->getSupportedVideoModes();
    rc = rgb.setVideoMode(colorVideoModes[6]);
    if (rc != STATUS_OK) {
      cerr << "Couldn't set the desired rgb video mode" << endl;
      return 0;
    }
    running = true;
    pthread_create(&thread, 0, (void* (*)(void*))threadFunction, (void *)this);
    return 1;
  }

  static void* threadFunction (NiWrapper* wrapper) {
    Status rc = STATUS_OK;
    rc = wrapper->depth.start();                      // Start the IR VideoStream
    rc = wrapper->rgb.start();                      // Start the IR VideoStream
 
    int h, w;               // Height and Width of the IR VideoFrame

    int mode = 0;
    int fcnt=0;
    VideoStream * streams[2] = {&wrapper->depth,&wrapper->rgb};
  
    bool depthReady = false;
    bool rgbReady = false;
    int idx = 0;
    while(wrapper->running) {             // Crux of this project
      OpenNI::waitForAnyStream(streams, 2, &idx);
      if (idx == 0){
	wrapper->depth.readFrame(&wrapper->depthf);
	VideoMode vmode = wrapper->depthf.getVideoMode();  // Get the IR VideoMode Info for this video stream.
	// This includes its resolution, fps and stream format.
	const uint16_t* imgBuf = (const uint16_t*)wrapper->depthf.getData();
	// PrimeSense gives the IR stream as 16-bit data output
	h=wrapper->depthf.getHeight();
	w=wrapper->depthf.getWidth();
	if (wrapper->depthFrame.rows!=h || wrapper->depthFrame.cols!=w || wrapper->depthFrame.type()!=CV_16U)
	  wrapper->depthFrame.create(h, w, CV_16U); // Create the OpenCV Mat Matrix Class Object
	// to receive the IR VideoFrames
	memcpy(wrapper->depthFrame.data, imgBuf, h*w*sizeof(uint16_t));
	if (!rgbReady)
	  depthReady = true;
	//cerr << "d";
      }
      if (idx == 1) {
	wrapper->rgb.readFrame(&wrapper->rgbf);
	VideoMode vmode = wrapper->rgbf.getVideoMode();  // Get the IR VideoMode Info for this video stream.
	// This includes its resolution, fps and stream format.
	const unsigned char* imgBuf = (const unsigned char*)wrapper->rgbf.getData();
	// PrimeSense gives the IR stream as 16-bit data output
	h=wrapper->rgbf.getHeight();
	w=wrapper->rgbf.getWidth();
	if (wrapper->rgbFrame.rows!=h || wrapper->rgbFrame.cols!=w || wrapper->rgbFrame.type()!=CV_16U)
	  wrapper->rgbFrame.create(h, w, CV_8UC3); // Create the OpenCV Mat Matrix Class Object
	memcpy(wrapper->rgbFrame.data, imgBuf, h*w*3*sizeof(unsigned char));
	if (depthReady)
	  rgbReady = true;
	pthread_mutex_lock(&wrapper->mutex);
	wrapper->depthFrameCopy=wrapper->depthFrame.clone();
	wrapper->rgbFrameCopy = wrapper->rgbFrame.clone();
	pthread_mutex_unlock(&wrapper->mutex);
	sem_post(&wrapper->sem);
	//cerr << "c";
      }

      if (wrapper->paramChanged) {
	if (wrapper->depthRegistration){
	  rc = wrapper->device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	} else {
	  rc = wrapper->device.setImageRegistrationMode(IMAGE_REGISTRATION_OFF);
	}
	if (rc != STATUS_OK) {
	  cerr << "Couldn't set the depth/color image registration [" << wrapper->depthRegistration << "]" << endl;
	  return 0;
	}
      }
    }
  }


  void stop() {
    if (running){
      running = false;
      void * result;
      pthread_join(thread, &result);
    }
    depth.stop();
    depth.destroy();
    rgb.stop();
    rgb.destroy();
    device.close();                         // Close the PrimeSense Device
  }
};

int main(int argc, char** argv)
{

  NiWrapper wrapper;
  wrapper.start();
  namedWindow("depth", 1);       // Create a named window
  namedWindow("color", 1);       // Create a named window
  namedWindow("topDepth", 1);       // Create a named window

  int idx = 0;
  Mat depth;
  Mat color;
  string prefix = "pino";
  bool saveThisFrame = false;
  bool saveAllFrames = false;

  Mat topDepth(600,640,CV_8UC1);
  ofstream g2oFile("out.g2o");
  g2oFile << "PARAMS_CAMERACALIB 0 0 0 0 -0.5 0.5 -0.5 0.5 570.342 570.342 320 240 " << endl;
  float f = 570.342;
  while(true) {             // Crux of this project
    wrapper.waitForFrame();
    memset(topDepth.data,0,topDepth.rows*topDepth.cols);
    wrapper.getFrame(depth,color);
    imshow("depth",depth);
    imshow("color",color);
    for (int j=0; j<depth.rows; j++){
      for (int i=0; i<depth.cols; i++){
	uint16_t d=depth.at<uint16_t>(j,i);
	if(d>0){
	  float z=1e-3*d;
	  float x=(i-320)*z/f;
	  float y=(i-240)*z/f;
	  x*=100;
	  z*=100;
	  int ix = x+320;
	  int iy = 240-y;
	  int iz = z;
	  if(ix>-1 && 
	     ix<topDepth.cols && 
		iz>-1 && iz<topDepth.rows)
	    topDepth.at<char>(iz,ix)=255;
	}
      }
    }
    imshow("topDepth",topDepth);
    //cerr << "R";
    char key = waitKey(10);
    if (key == 27) {
      wrapper.stop();
      break;
    }
    switch (key){
    case 'r': wrapper.setDepthRegistration(!wrapper.depthRegistration); break;
    case 's': saveThisFrame = true; break;
    case 'c': saveAllFrames = ! saveAllFrames; break;
    case '+': f+=0.01; cerr << "f:" << f; break;
    case '-': f-=0.01; cerr << "f:" << f; break;

    default:;
    }
    if (saveAllFrames)
      saveThisFrame = true;
    if (saveThisFrame){
      g2oFile << "VERTEX_SE3:QUAT " << idx << " 0 0 0 0 0 0 1 "<< endl;
      g2oFile << "RGBD_DATA 0 " << makeFilename(prefix,idx) << " 0 hostname 0 " << endl; 
      cerr << "S";
      
      saveFrame(prefix,idx,depth);
      //saveFrame(prefix,idx,color);
      saveThisFrame = false;
      idx ++;
    }
  }
}
