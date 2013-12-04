#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <sstream>
#include <queue>

#include <pthread.h>

#include "g2o/stuff/command_args.h"
#include "g2o_frontend/pwn_core/depthimage.h"
#include "g2o_frontend/pwn_core/depthimageconverter.h"
#include "g2o_frontend/pwn_core/pinholepointprojector.h"

#include "g2o_frontend/pwn_core/linearizer.h"
#include "g2o_frontend/pwn_core/correspondencefinder.h"
#include "g2o_frontend/pwn_core/aligner.h"

#include <unistd.h>

//#undef _PWN_USE_CUDA_

#ifdef _PWN_USE_CUDA_
#include "g2o_frontend/pwn_cuda/cualigner.h"
#define failneim "cooda.dat"
#else
#define failneim "no_cooda.dat"
#endif

using namespace Eigen;
using namespace g2o;
using namespace std;
using namespace pwn;

set<string> readDir(std::string dir){
  DIR *dp;
  struct dirent *dirp;
  struct stat filestat;
  std::set<std::string> filenames;
  dp = opendir( dir.c_str() );
  if (dp == NULL){
    return filenames;
  }
  
  while ((dirp = readdir( dp ))) {
    string filepath = dir + "/" + dirp->d_name;

    // If the file is a directory (or is in some way invalid) we'll skip it 
    if (stat( filepath.c_str(), &filestat )) continue;
    if (S_ISDIR( filestat.st_mode ))         continue;

    filenames.insert(filepath);
  }

  closedir( dp );
  return filenames;
}


struct ParallelCudaAligner{
  std::queue<Frame*> scenes;
  pthread_mutex_t insertionMutex;
  volatile bool run;
  Aligner* aligner;

  pthread_t alignerThread;
  ParallelCudaAligner(Aligner* aligner){
    pthread_mutex_init(&insertionMutex,0);
    this->aligner = aligner;
    run = false;
  }

  ~ParallelCudaAligner(){
    pthread_mutex_destroy(&insertionMutex);
    stop();
    Frame* s=0;
    do {
      s=popScene();
      if (s)
	delete s;
    } while (s);
  }

  void addScene(Frame* s){
    pthread_mutex_lock(&insertionMutex);
    scenes.push(s);
    pthread_mutex_unlock(&insertionMutex);
  }

  Frame* popScene(){
    Frame* s=0;
    pthread_mutex_lock(&insertionMutex);
    if (scenes.size()){
      s = scenes.front();
      scenes.pop();
    }
    pthread_mutex_unlock(&insertionMutex);
    return s;
  }
  
  
  static void* threadFn(ParallelCudaAligner* pr){
    Frame* previous = 0;
    ofstream os(failneim);
    while (pr->run){
      Frame * s = pr->popScene();
      if (s) {
	if (previous) {
	  cerr << "AAA" << endl;
      	  pr->aligner->setReferenceFrame(previous);
	  pr->aligner->setCurrentFrame(s);
	  pr->aligner->align();
	  os << "time: " << pr->aligner->totalTime() 
	     << " inliers: " << pr->aligner->inliers()
	     << " error: " << pr->aligner-> error() << endl;
	  delete previous;
	}
	previous = s;
      }
      usleep(1000);
    }
    os.close();
    if (previous)
      delete previous;
    return 0;
  }
  
  void start(){
    run  = true;
    pthread_create(&alignerThread, 0, (void * (*)(void *)) threadFn, (void*) this );
  }

  void stop(){
    run  = false;
    pthread_join(alignerThread, 0);
  }
 
};

int
 main(int argc, char** argv){
  string dirname;
  string sensorType;
  PinholePointProjector projector;
  StatsCalculator statsCalculator;
  PointInformationMatrixCalculator pointInformationMatrixCalculator;
  NormalInformationMatrixCalculator normalInformationMatrixCalculator;
  DepthImageConverter converter(&projector, 
				&statsCalculator,
				&pointInformationMatrixCalculator,
				&normalInformationMatrixCalculator);
  
  g2o::CommandArgs arg;
  arg.paramLeftOver("dirname", dirname, "", "", true);
  arg.param("sensorType", sensorType, "kinect", "sensor type: xtion640/xtion480/kinect");
  arg.parseArgs(argc, argv);
  cerr << "dirname " << dirname << endl;

  std::vector<string> filenames;
  std::set<string> filenamesset = readDir(dirname);
  for(set<string>::const_iterator it =filenamesset.begin(); it!=filenamesset.end(); it++) {
    filenames.push_back(*it);
  }


  Eigen::Matrix3f cameraMatrix;
  cameraMatrix.setIdentity();

  if (sensorType=="xtion640") {
    cameraMatrix << 
    570.342, 0,       320,
    0,       570.342, 240,
    0.0f, 0.0f, 1.0f;  
  }  else if (sensorType=="xtion320") {
    cameraMatrix << 
      570.342, 0,       320,
      0,       570.342, 240,
      0.0f, 0.0f, 1.0f;  
    cameraMatrix.block<2,3>(0,0)*=0.5;
  } else if (sensorType=="kinect") {
    cameraMatrix << 
      525.0f, 0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f, 0.0f, 1.0f;  
  } else {
    cerr << "unknown sensor type: [" << sensorType << "], aborting (you need to specify either xtion or kinect)" << endl;
    return 0;
  }

  Eigen::Matrix3f scaledCameraMatrix = cameraMatrix* 0.5;
    //Eigen::Matrix3f scaledCameraMatrix = cameraMatrix;
  cameraMatrix(2,2) = 1.0f;
  
  projector.setCameraMatrix(scaledCameraMatrix);
  
  DepthImage depthImage, scaledDepthImage;
  cerr << "there are " << filenames.size() << " files  in the pool" << endl; 

  Linearizer linearizer;
  CorrespondenceFinder correspondenceFinder;

#ifdef _PWN_USE_CUDA_
  CuAligner aligner;
  // Aligner aligner;
#else
  Aligner aligner;
#endif


  aligner.setProjector(&projector);
  projector.setCameraMatrix(scaledCameraMatrix);
  aligner.setLinearizer(&linearizer);
  linearizer.setAligner(&aligner);
  aligner.setCorrespondenceFinder(&correspondenceFinder);
  aligner.setOuterIterations(10);
  aligner.setInnerIterations(1);

  ParallelCudaAligner alignerThread(&aligner);
  alignerThread.start();

  for (size_t i=0; i<filenames.size(); i++) {
    cerr << ">>>>>>>>>>>>>>>>>>>>>>>> PROCESSING " << filenames[i] << " <<<<<<<<<<<<<<<<<<<<" <<  endl;
    cerr << "queueSize : " << alignerThread.scenes.size() << endl;
    if (!depthImage.load(filenames[i].c_str())){
      cerr << " skipping " << filenames[i] << endl;
      continue;
    }
    Frame* scene = new Frame();
    DepthImage::scale(scaledDepthImage, depthImage, 2);
    converter.compute(*scene, scaledDepthImage);
    correspondenceFinder.setSize(scaledDepthImage.rows(), scaledDepthImage.cols());
    while (alignerThread.scenes.size()>20) {
      usleep(10000);
    }
    alignerThread.addScene(scene);
    //delete scene;
  }
  cerr << "waiting for the queue to terminate" << endl;
  while(alignerThread.scenes.size()){
    usleep(1000);
  }
  alignerThread.stop();
}
