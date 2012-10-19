#include <queue>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/time.h"
#include "pcl/io/pcd_io.h"
#include "pcl/io/openni_grabber.h"
#include "pcl/visualization/pcl_visualizer.h"

#include "g2o/stuff/command_args.h"
     

#ifdef _WIN32
# define sleep(x) Sleep((x)*1000) 
#endif

using namespace std;

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif


typedef  pcl::PointXYZRGBA PointT;

struct OnLineCloudManager{
  OnLineCloudManager(){
    numClouds  = 0;
  }

  void new_cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr &cloud)
  {  
    if (! (numClouds%30))
      cerr << "c";
    boost::mutex::scoped_lock lock (_cloud_mutex);
    _cloud=cloud->makeShared();
    numClouds++;
  }

  pcl::PointCloud<PointT>::Ptr getCloudCopy(){
      boost::mutex::scoped_lock lock (_cloud_mutex);
      return _cloud;
  }

  pcl::PointCloud<PointT>::Ptr _cloud;
  boost::mutex _cloud_mutex;
  int numClouds;
};

inline Eigen::Vector3f xyz2uvd(const Eigen::Vector3f xyz, float focalLenght, float baseline){
  if (xyz.x()==std::numeric_limits<float>::quiet_NaN())
    return Eigen::Vector3f(
			   std::numeric_limits<float>::quiet_NaN(), 
			   std::numeric_limits<float>::quiet_NaN(), 
			   std::numeric_limits<float>::quiet_NaN());
  float iz = 1./xyz(2);
  float d = focalLenght*baseline*iz;
  float u = xyz(0)*focalLenght*iz;
  float v = xyz(1)*focalLenght*iz;
  return Eigen::Vector3f(u,v,d);
}

inline Eigen::Vector3f uvd2xyz(const Eigen::Vector3f uvd, float focalLenght, float baseline){
  if (uvd.x()==std::numeric_limits<float>::quiet_NaN())
    return Eigen::Vector3f(
			   std::numeric_limits<float>::quiet_NaN(), 
			   std::numeric_limits<float>::quiet_NaN(), 
			   std::numeric_limits<float>::quiet_NaN());
  float z=focalLenght*baseline/uvd(2);
  float x=uvd(0)/focalLenght*z;
  float y=uvd(1)/focalLenght*z;
  return Eigen::Vector3f(x,y,z);
}

bool computeSmoothedNormal(Eigen::Vector3f& normal, Eigen::Matrix3f& nOmega, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, 
			   const std::vector<Eigen::Matrix3f>& covariances,
			   int xmin, int ymin, int xmax, int ymax, int countThr, int kMax=4){
  // compute the mean
  
  Eigen::Vector3f sum(0.,0.,0.);
  Eigen::Matrix3f sumCov;
  sumCov.setZero();
  int count=0;
  for (int v=ymin; v<ymax; v++){
    int i=v*cloud->width+xmin;
    for (int u=xmin; u<xmax; ++u, ++i){
      pcl::PointXYZRGBA& p=cloud->at(i);
      if (p.x==std::numeric_limits<float>::quiet_NaN())
	continue;
      sumCov += covariances[i];
      sum.x()+=p.x;
      sum.y()+=p.y;
      sum.z()+=p.z;
      count++;
    }
  }
  if (count<countThr)
    return false;
  
  Eigen::Vector3f p0=sum*(1.0f/(float)count);
  Eigen::Matrix3f Sigma0=sumCov*(1.0f/(float)count);

  cerr << "p0: " << endl << p0 << endl;
  cerr << "Sigma0: " << endl << Sigma0 << endl;

  Eigen::Matrix3f H;
  Eigen::Vector3f b;
  Eigen::Vector3f n(0.,0.,-1.);

  for (int k=0; k<kMax;++k){
    float e=0;
    H.setZero();
    b.setZero();
    // construct the linear part of the system that does not change
    for (int v=ymin; v<ymax; v++){
      int i=v*cloud->width+xmin;
      for (int u=xmin; u<xmax; ++u, ++i){
	pcl::PointXYZRGBA& p=cloud->at(i);
	if (p.x==std::numeric_limits<float>::quiet_NaN())
	  continue;
	Eigen::Vector3f dp(p.x, p.y, p.z);
	dp-=p0;
	// project the intrinsic covariance of the points onto the current normal
	// to get the expected error
	Eigen::Matrix3f SigmaP=Sigma0+covariances[i];
	float sigma=n.transpose()*SigmaP*n;
	float omega=1./sigma;
	H+=(omega*dp)*dp.transpose();
	//H+=dp*dp.transpose();
	b -= dp*dp.transpose()*n*omega;
	e += pow(n.dot(dp),2)*omega;
      }
    }
    
    if (k<(kMax-1)){
      float gain =1000;
      H+=count*n*n.transpose()*4.*gain;
      b-=count*n*2.*gain*(n.dot(n)-1);
      Eigen::Vector3f dn=b;
      H.llt().solveInPlace(dn);
      n+=dn;
    }
    cerr << "E=" << e << endl;
  }
  // n contains the normal already
  normal=n;
  nOmega=H;
  return true;
}


void fillDisparity(std::vector<Eigen::Matrix3f>& covBuffer, std::vector<float>& buffer, int& width, int& height, 
                   float rgbFocalLenght, float depthFocalLenght, float baseline, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, bool computeCov=false){
  width=cloud->width;
  height=cloud->height;
  buffer.resize(width*height);
  covBuffer.resize(width*height);
  int cx=cloud->width/2;
  int cy=cloud->height/2;
  int idx=0;
  float fb=baseline*depthFocalLenght;
  Eigen::Matrix3f Sigma;
  Sigma.setIdentity();
  Sigma*=50;
  for (int v=-cy; v<cy; ++v)
    for (int u=-cx; u<cx; ++u, ++idx){
      const pcl::PointXYZRGBA& p=cloud->at(idx);
      if (p.z==std::numeric_limits<float>::quiet_NaN()){
        buffer[idx]=0;
        continue;        
      }
      float iz=1./p.z;
      float d = rgbFocalLenght*baseline*iz;
      float id = 1./d;
      float id2= id*id;
      float u = rgbFocalLenght*p.x*iz;
      float v = rgbFocalLenght*p.y*iz;
      buffer[idx]=d;
      Eigen::Matrix3f J;
      J << 
        baseline * id,     0.,              - baseline * u * id2,
        0.,                 baseline * id,  - baseline * v * id2,
        0.,                 0.,             - fb * id2;

      covBuffer[idx]=J*Sigma*J.transpose();

    }
}



void viewerKeyCallback (const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  pcl::visualization::KeyboardEvent* keyEvent=(pcl::visualization::KeyboardEvent*)cookie;
  *keyEvent=event;
}

typedef std::queue<pcl::PointCloud<PointT>::Ptr> CloudQueue;
void saveQueue(std::string baseName, boost::mutex* queueMutex, CloudQueue* q, bool* stop){
  int idx=0;
  while (! *stop){
    pcl::PointCloud<PointT>::Ptr c;
    bool saveThis = false;
    {
      if (! q->empty()){
	saveThis=true;
	boost::mutex::scoped_lock (*queueMutex);
	c=q->front();
	q->pop();
      }
      if (saveThis) {
	char buf[1024];
	sprintf(buf,"%s-%03d.pcd",baseName.c_str(), idx);
	pcl::io::savePCDFileBinary(buf, *c);
	cerr << "saved " << buf << endl;
	idx++;
      }
    }
    usleep(1000);
  }
}

inline int pIndex(int x, int y, int w, int h){
  return (y+h/2)*w+(x+h/2);
}

inline void pUV(int& x, int& y, int w, int h, int index){
  x=index%w-w/2;
  y=index/w-h/2;
}

int main (int argc, char** argv)
{

  int skipFrames;
  std::string baseName;
  g2o::CommandArgs arg;
  arg.param("skipFrames", skipFrames, 5, "save a frame each x when in continuous mode");
  arg.paramLeftOver("<basename>", baseName, "", "the name of the file that will be generated", true);
  arg.parseArgs(argc, argv);

  if (baseName==""){
    cerr << "You must provide a filename for saving the pcd" << endl;
    return 0;
  }
  

  pcl::OpenNIGrabber* interface = new pcl::OpenNIGrabber();

  boost::shared_ptr < openni_wrapper::OpenNIDevice > device = interface->getDevice();
  cerr << "Device supports hw synchronization: " << device->isSynchronizationSupported() << endl;
  if (0 && device->isSynchronizationSupported()){
    cerr << "Enabling hw synchronization" << endl;
    device->setSynchronization(true);
  }
  cerr << "Hw synchronization enabled:         " << device->isSynchronized() << endl;
  cerr << "Device supports depth registration: " << device->isDepthRegistrationSupported() << endl;
  if (device->isDepthRegistrationSupported()){
    cerr << "Enabling depth registration" << endl;
    device->setDepthRegistration(true);
  }
  cerr << "Depth registration enabled:         " << device->isDepthRegistered() << endl;
  cerr << "RGB Focal Lenght:                   " << device->getImageFocalLength() << endl;
  cerr << "Depth Focal Lenght:                 " << device->getDepthFocalLength() << endl;
  cerr << "Baseline:                           " << device->getBaseline() << endl;



  OnLineCloudManager *manager = new OnLineCloudManager();

  // make callback function from member function
  boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f =
    boost::bind (&OnLineCloudManager::new_cloud_cb_, manager, _1);
  
  // connect callback function for desired signal. In this case its a point cloud with color values
  boost::signals2::connection c = interface->registerCallback (f);
  // start receiving point clouds
  interface->start ();

  cerr << "staring kinect" << endl;
  
 
  pcl::visualization::PCLVisualizer* viewer=new pcl::visualization::PCLVisualizer("mtclv");

  // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
  pcl::visualization::KeyboardEvent keyEvent(/*bool action*/ false, 
					     /*const std::string& key_sym*/ "", 
					     /*unsigned char key*/ 0,
					     /*bool alt*/ false, 
					     /*bool ctrl*/ false,
					     /*bool shift*/ false);

  pcl::visualization::KeyboardEvent nullKeyEvent=keyEvent;

  boost::signals2::connection c2 = viewer->registerKeyboardCallback(&viewerKeyCallback,&keyEvent);

  boost::mutex queueMutex;
  std::queue<pcl::PointCloud<PointT>::Ptr> cloudQueue;

  bool quit = false;
  bool save = false;
  bool continuous = false;
  bool computeCovariances = false;
  cerr << "Saver Thread started" << endl;

  boost::thread saverThrd(boost::bind(&saveQueue, baseName, &queueMutex, &cloudQueue, &quit));

  cerr << "Entering in the main loop" << endl;
  int lastCloud = manager->numClouds;
  while(!quit){


    if (keyEvent.keyDown()){
      switch(keyEvent.getKeyCode()){
      case 's': 
	cerr << "saving" << endl; 
	save=true;
	break;
      case 'c': 
        if (continuous){
          cerr << endl << "continuous mode off" << endl;  
        } else {
          cerr << endl << "continuous mode on" << endl;  
        }
        continuous =  !continuous;
	break;
      case 'k': 
        if (computeCovariances){
          cerr << endl << "covariances mode off" << endl;  
        } else {
          cerr << endl << "covariances mode on" << endl;  
        }
        computeCovariances =  !computeCovariances;
	break;
      case 'a': 
	  cerr << "quitting"; 
	  quit=true;  
	break;
      }
    }
    keyEvent=nullKeyEvent;


    pcl::PointCloud<PointT>::Ptr currentCloud=manager->getCloudCopy();
    
    int w,h;
    std::vector<float> dbuf;
    std::vector<Eigen::Matrix3f> cbuf;
    fillDisparity(cbuf, dbuf, w, h,                   
                  device->getImageFocalLength(currentCloud->width), 
                  device->getDepthFocalLength(currentCloud->width), 
                  device->getBaseline(), 
                  currentCloud, computeCovariances);
    
    std::vector<int> mp;
    mp.push_back(pIndex(-w*.25,  -h*.25,  w, h));
    mp.push_back(pIndex(0,       -h*.25,  w, h));
    mp.push_back(pIndex(+w*.25,  -h*.25,  w, h));
    mp.push_back(pIndex(-w*.25,  0,       w, h));
    mp.push_back(pIndex(0,       0,       w, h));
    mp.push_back(pIndex(+w*.25,  0,       w, h));
    mp.push_back(pIndex(-w*.25,  h*.25,   w, h));
    mp.push_back(pIndex(0,       h*.25,   w, h));
    mp.push_back(pIndex(+w*.25,  h*.25,   w, h));

    if (computeCovariances && !(manager->numClouds%30)){
      Eigen::Vector3f normal;
      Eigen::Matrix3f nOmega;
      int nw=30;
      bool hasNiceNormal = computeSmoothedNormal(normal, nOmega, currentCloud, 
						 cbuf, w/2-nw, h/2-nw, w/2+nw, h/2+nw, 20, 10);
      if (hasNiceNormal){
	cerr << endl <<"normal: " << endl <<normal << endl;
	cerr << endl <<"nOmega: " << endl <<nOmega << endl;
      }
				 
      for (size_t i =0; i<1/*mp.size()*/; i++){
	int index=mp[i];
	int u,v;
	pUV(u,v,w,h,index);
	pcl::PointXYZRGBA pc = currentCloud->at(index);
	Eigen::Vector3f _p_xyz(pc.x, pc.y, pc.z);
	Eigen::Vector3f _p_uvd = xyz2uvd(_p_xyz, device->getImageFocalLength(currentCloud->width), device->getBaseline());
	Eigen::Vector3f _p_xyzr = uvd2xyz(_p_uvd, device->getImageFocalLength(currentCloud->width), device->getBaseline());
	cerr << endl
	     << " point: " << i << " index: " << index << endl 
	     << " ic:  " << u << "," << v << "," << dbuf[index] << endl
	     << " ric: " << _p_uvd(0) << "," << _p_uvd(1) << "," << _p_uvd(2) << endl
	     << " wc:  " << pc.x << "," << pc.y << "," << pc.z << endl
	     << " rwc: " << _p_xyzr(0) << "," << _p_xyzr(1) << "," << _p_xyzr(2) << endl;
	cerr << cbuf[index] << endl;
      }
    }
#if 1

    if (manager->numClouds>lastCloud){

      if (!viewer->updatePointCloud (currentCloud, "OpenNICloud"))
      {
        viewer->addPointCloud (currentCloud, "OpenNICloud");
        viewer->resetCameraViewpoint ("OpenNICloud");
      }

      if (save || continuous && !(manager->numClouds % skipFrames)) {
	boost::mutex::scoped_lock lock (queueMutex);
	cloudQueue.push(currentCloud);
	cerr << "cloudQueue.size()=" << cloudQueue.size() << endl;
	save=false;
      }
      viewer->spinOnce();
    }

#endif

    lastCloud=manager->numClouds;
  }
  saverThrd.join();
  // start receiving point clouds
  interface->stop ();
  delete manager;
  delete viewer;
}
