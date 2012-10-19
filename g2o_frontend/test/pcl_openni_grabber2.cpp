#include <queue>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/time.h"
#include "pcl/io/pcd_io.h"
#include "pcl/io/openni_grabber.h"
//#include "pcl/visualization/cloud_viewer.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/features/normal_3d.h"
#include "pcl/keypoints/sift_keypoint.h"
#include "pcl/features/integral_image_normal.h"
     

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
    newCloudReceived = false;
  }

  void new_cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr &cloud)
  {  
    cerr << "c";
    boost::mutex::scoped_lock lock (_cloud_mutex);
    _cloud=cloud->makeShared();
    newCloudReceived=true;
  }

  pcl::PointCloud<PointT>::Ptr getCloudCopy(){
      boost::mutex::scoped_lock lock (_cloud_mutex);
      return _cloud;
  }

  pcl::PointCloud<PointT>::Ptr _cloud;
  boost::mutex _cloud_mutex;
  bool newCloudReceived;
};




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

int main ()
{


  pcl::Grabber* interface = new pcl::OpenNIGrabber();
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

  string baseName("base");
  cerr << "Saver Thread started" << endl;

  boost::thread saverThrd(boost::bind(&saveQueue, baseName, &queueMutex, &cloudQueue, &quit));

  //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;


  const float min_scale = 0.0005; 
  const int nr_octaves = 4; 
  const int nr_scales_per_octave = 5; 
  const float min_contrast = 1; 
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;

  cerr << "Entering in the main loop" << endl;
  while(!quit){


    if (keyEvent.keyDown()){
      switch(keyEvent.getKeyCode()){
      case 's': 
	cerr << "saving" << endl; 
	save=true;
	break;
      case 'c': 
	cerr << "saving 100 images" << endl;  
	break;
      case 'a': 
	  cerr << "quitting"; 
	  quit=true;  
	break;
      }
    }
    keyEvent=nullKeyEvent;


    pcl::PointCloud<PointT>::Ptr currentCloud=manager->getCloudCopy();

#if 1

    if (manager->newCloudReceived){

      cerr << "here";


    {
      pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());;//new API
      pcl::PointCloud<pcl::PointWithScale>::Ptr sifts (new pcl::PointCloud<pcl::PointWithScale>);
      sift.setInputCloud(currentCloud);
      sift.setSearchMethod (tree);
      sift.setScales(min_scale, nr_octaves, nr_scales_per_octave);
      sift.setMinimumContrast(min_contrast);
      sift.compute (*sifts);
    
      cerr << "Computed " << sifts->points.size () << " SIFT Keypoints " << endl;
    }

      //ne.setInputCloud (currentCloud);

      // pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
      // ne.setSearchMethod (tree);
      // ne.setRadiusSearch (0.05);

      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

      // ne.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::COVARIANCE_MATRIX/*AVERAGE_3D_GRADIENT*/);
      // ne.setMaxDepthChangeFactor(0.02f);
      // ne.setNormalSmoothingSize(5.0f);
      // ne.setInputCloud(currentCloud);
      // ne.compute(*normals);


      if (!viewer->updatePointCloud (currentCloud, "OpenNICloud"))
      {
        viewer->addPointCloud (currentCloud, "OpenNICloud");
        viewer->resetCameraViewpoint ("OpenNICloud");
      }
      // // Render the data
      // viewer->removePointCloud ("normalcloud");
      // viewer->addPointCloudNormals<PointT, pcl::Normal> (currentCloud, normals, 10, 0.05, "normalcloud");

      if (save) {
	boost::mutex::scoped_lock lock (queueMutex);
	cloudQueue.push(currentCloud);
	cerr << "cloudQueue.size()=" << cloudQueue.size() << endl;
	save=false;
      }
      viewer->spinOnce();
    }

#endif

    manager->newCloudReceived = false;
  }
  saverThrd.join();
  // start receiving point clouds
  interface->stop ();
  delete manager;
  delete viewer;
}
