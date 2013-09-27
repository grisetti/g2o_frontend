#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/Marker.h>


#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "highgui.h"
#include <boost/bind.hpp>
#include <fstream>
#include <iostream>
#include "pwn_tracker.h"
using namespace std;
using namespace pwn;
using namespace boss;
using namespace pwn_tracker;



  template <typename T1, typename T2>
  void convertScalar(T1& dest, const T2& src){
    for (int i=0; i<src.matrix().cols(); i++)
      for (int j=0; j<src.matrix().rows(); j++)
	dest.matrix()(j,i) = src.matrix()(j,i);

  }

  float compareNormals(cv::Mat& m1, cv::Mat& m2){
    if (m1.rows!=m2.rows || m1.cols != m2.cols)
      return 0;
    return norm(abs(m1-m2));
  }

  float compareDepths(cv::Mat& m1, cv::Mat& m2){
    if (m1.rows!=m2.rows || m1.cols != m2.cols)
      return 0;
    // consides only the pixels that are >0;
    cv::Mat mask = (m1>0) & (m2>0);
    //cerr << "mask!" << endl;

    mask.convertTo(mask, m1.type());
    //cerr << "convert" << endl;

    cv::Mat diff = (m1-m2)&mask;

    //cerr << "diffMask" << endl;

    diff.convertTo(diff, CV_32FC1);
    return norm(diff);;
  }

cv::Mat normalsComparison;
cv::Mat depthsComparison;
std::vector<cv::Mat> normalImages;
std::vector<cv::Mat> depthImages;
int r = 0;
int c = 0;
float cutoff = 0;
float a=0.5;

void matchImages(Aligner* aligner, DepthImageConverter* converter, PwnTrackerFrame* from, PwnTrackerFrame* to){
  //pwn::Frame* fromCloud = from->cloud.get();
  //pwn::Frame* toCloud = to->cloud.get();
  boss_logger::ImageBLOB* fromDepthBlob = from->depthImage.get();
  boss_logger::ImageBLOB* toDepthBlob = to->depthImage.get();
  
  cerr << "loaded images" << endl;
  PinholePointProjector* projector = (PinholePointProjector*)converter->_projector;
  pwn::DepthImage fromDepth, toDepth;
  pwn::Frame fromCloud, toCloud; 
  Eigen::Isometry3f fromOffset, toOffset;
  Eigen::Matrix3f fromCameraMatrix, toCameraMatrix;
  int scale = 4;
  int r, c;
  
  cerr << "converted reference" << endl;
  {
    fromDepth.fromCvMat(fromDepthBlob->cvImage());
    convertScalar(fromOffset, from->sensorOffset);
    convertScalar(fromCameraMatrix, from->cameraMatrix);
    projector->setCameraMatrix(fromCameraMatrix);
    pwn::DepthImage scaledFromDepth;
    DepthImage::scale(scaledFromDepth, fromDepth, scale);
    projector->scale (1./scale);
    converter->compute(fromCloud, scaledFromDepth, fromOffset);
    fromCloud.save("from.pwn", 1 , true, Eigen::Isometry3f::Identity());
    cerr << "points 1: " << fromCloud.points().size();
  }

  cerr << "converted target" << endl;
  {
    toDepth.fromCvMat(toDepthBlob->cvImage());
    convertScalar(toOffset, to->sensorOffset);
    convertScalar(toCameraMatrix, to->cameraMatrix);
    projector->setCameraMatrix(toCameraMatrix);
    pwn::DepthImage scaledToDepth;
    DepthImage::scale(scaledToDepth, toDepth, scale);
    projector->scale (1./scale);
    converter->compute(toCloud, scaledToDepth, toOffset);
    r = projector->imageRows();
    c = projector->imageCols();
    toCloud.save("to.pwn", 1 , true, Eigen::Isometry3f::Identity());
    cerr << "points 2: " << toCloud.points().size();
  }
  
  cerr << "setting aligner" << endl;
  PinholePointProjector* alignerProjector = (PinholePointProjector*)aligner->projector();
  aligner->setReferenceSensorOffset(fromOffset);
  aligner->setCurrentSensorOffset(toOffset);
  aligner->setInitialGuess(Eigen::Isometry3f::Identity());
  alignerProjector->setImageSize(r,c);
  alignerProjector->setCameraMatrix(projector->cameraMatrix());
  aligner->correspondenceFinder()->setImageSize(r,c);
  aligner->setReferenceFrame(&fromCloud);
  aligner->setCurrentFrame(&toCloud);
  cerr << "aligning" << endl;
  aligner->align();
  cerr << "inliers: " << aligner->inliers() << endl;
  cerr << "chi2: " << aligner->error() << endl;
  cerr << "chi2/inliers: " << aligner->error()/aligner->inliers() << endl;
  cerr << "transform   : " << t2v(aligner->T()).transpose() << endl;

  pwn::Frame alignedCloud;
  alignedCloud = fromCloud;
  alignedCloud.add (toCloud, aligner->T());
  alignedCloud.save("aligned.pwn", 1 , true, Eigen::Isometry3f::Identity());
  cerr << "deleting things" << endl;

  DepthImage currentDepthThumb, referenceDepthThumb;
  DepthImage::scale(currentDepthThumb, aligner->correspondenceFinder()->currentDepthImage(),1);
  DepthImage::scale(referenceDepthThumb, aligner->correspondenceFinder()->referenceDepthImage(),1);
  cv::Mat currentRect, referenceRect;
  currentDepthThumb.toCvMat(currentRect);
  referenceDepthThumb.toCvMat(referenceRect);
  cv::Mat mask = (currentRect>0) & (referenceRect>0);
  currentRect.convertTo(currentRect, CV_32FC1);
  referenceRect.convertTo(referenceRect, CV_32FC1);
  mask.convertTo(mask, currentRect.type());
  cv::Mat diff = abs(currentRect-referenceRect)&mask;
  cerr << "MATCHING ERROR: " << norm(diff)/countNonZero(mask);
  diff = diff*(1000.0f/65535.0f);
  cv::imshow("rectDiff", diff);
  delete toDepthBlob;
  delete fromDepthBlob;
  cerr << "done" << endl;
}

void mouseEvent(int evt, int x, int y, int flags, void* param){
    if(evt==CV_EVENT_LBUTTONDOWN){
      r = x;
      c = y;
      cerr << "r,c:" << r << " " << c << endl;
    }
}

int main(int argc, char** argv){
  if (argc<3) {
    cerr << " u should provide a config file and an input file" << endl;
    return 0;
  }

  Aligner* aligner;
  DepthImageConverter* converter;
  std::vector<Serializable*> instances = readConfig(aligner, converter, argv[1]);

  Deserializer des;
  des.setFilePath(argv[2]);
  Serializable* o=0;
  std::vector<Serializable*> objects;
  std::vector<PwnTrackerFrame*> trackerFrames;
  boss_map::MapManager* manager = 0;
  while(o=des.readObject()){
    objects.push_back(o);
    boss_map::MapManager* m = dynamic_cast<boss_map::MapManager*>(o);
    if (m) {
      manager = m;
    }
    PwnTrackerFrame* f = dynamic_cast<PwnTrackerFrame*>(o);
    if (f) {
      trackerFrames.push_back(f);
    }
  }
  
  

  cerr << "read " << objects.size() << " elements";
  cerr << "aligner" << aligner << endl;
  cerr << "converter" << converter << endl;
  cerr << "manager: " << manager << endl;
  cerr << "# frames: " << trackerFrames.size() << endl;

  // load the images
  std::vector<cv::Mat> normalImages;
  std::vector<cv::Mat> depthImages;
  for(size_t i = 0; i<trackerFrames.size(); i++){
    cv::Mat normalImg;
    PwnTrackerFrame *n = trackerFrames[i];
    ImageBLOB* thumbnail = n->normalThumbnail.get();
    thumbnail->cvImage().convertTo(normalImg, CV_32FC3);
    normalImg=normalImg-127.0f;
    normalImg=normalImg*(1./255);
    normalImages.push_back(normalImg);
    delete thumbnail;

    thumbnail = n->depthThumbnail.get();
    cv::Mat depthImg;
    thumbnail->cvImage().convertTo(depthImg,CV_32FC1);
    depthImg = depthImg *(1./65535);
    depthImages.push_back(depthImg);
    delete thumbnail;
  }
 
  cerr << "images loaded" << endl;

  int numImages = normalImages.size();

  normalsComparison = cv::Mat(numImages, numImages, CV_32FC1);
  float maxNormalDifference = 0;
  for (int i = 0; i<normalsComparison.rows; i++)
    for (int j = 0; j<=i; j++){
      cv::Mat& m1  = normalImages[i];
      cv::Mat& m2  = normalImages[j];
      float f = compareNormals(m1,m2);
      //cerr << "val: " << i << " " << j << " " << f << endl;
      normalsComparison.at<float>(j,i)=0;
      normalsComparison.at<float>(i,j)=f;
      if (maxNormalDifference<f)
	maxNormalDifference = f;
    }
  cerr << "maxNormalDifference: " << maxNormalDifference;
  maxNormalDifference = 1./maxNormalDifference;
  for (int i = 0; i<normalsComparison.rows; i++)
    for (int j = 0; j<=i; j++){
      normalsComparison.at<float>(i,j)*=maxNormalDifference;
    }


 depthsComparison = cv::Mat(numImages, numImages, CV_32FC1);
  float maxDepthDifference = 0;
  for (int i = 0; i<depthsComparison.rows; i++)
    for (int j = 0; j<=i; j++){
      cv::Mat& m1  = depthImages[i];
      cv::Mat& m2  = depthImages[j];
      float f = compareDepths(m1,m2);
      //cerr << "val: " << i << " " << j << " " << f << endl;
      depthsComparison.at<float>(j,i)=0;
      depthsComparison.at<float>(i,j)=f;
      if (maxDepthDifference<f)
	maxDepthDifference = f;
    }
  cerr << "maxDepthDifference: " << maxDepthDifference;
  maxDepthDifference = 1./maxDepthDifference;
  for (int i = 0; i<depthsComparison.rows; i++)
    for (int j = 0; j<=i; j++){
      depthsComparison.at<float>(i,j)*=maxDepthDifference;
    }

  cvNamedWindow("similarity", 0); 
  cvNamedWindow("cutoff", 0); 
  cvNamedWindow("normals1", 0);
  cvNamedWindow("normals2", 0);
  cvNamedWindow("normalsDiff", 0);
  cvNamedWindow("depths1", 0);
  cvNamedWindow("depths2", 0);
  cvNamedWindow("depthsDiff", 0);
  cvNamedWindow("rectDiff", 0);
  cvSetMouseCallback("cutoff", mouseEvent, 0);
  //cvSetMouseCallback("depthsComparison", mouseEvent, 0);
  
  int ca = 0;
  while(ca!=27){
    cv::Mat mix = depthsComparison*a + normalsComparison*(1-a);
    cv::Mat shownImage = mix>cutoff;
    int nz = numImages*numImages -cv::countNonZero(shownImage);
    nz -= numImages*(numImages+1)/2;
    cv::circle(shownImage, cv::Point(r,c), 4, 0.0f);
    cv::imshow("cutoff", shownImage);
    cv::imshow("similarity", mix);
    ca = cv::waitKey();
    switch (ca) {
    case 1113938: c --; break;
    case 1113940: c ++; break;
    case 1113937: r --; break;
    case 1113939: r ++; break;
    case 1114027: cutoff += 0.01; break;
    case 1114029: cutoff -= 0.01; break;
    case 1048673: a+=0.01; break;
    case 1048674: a-=0.01; break;
    case 1048608: matchImages(aligner, converter, trackerFrames[r], trackerFrames[c]); break;
    default: std::cerr << ca << endl;
    }
    if (r>=numImages)
      r = numImages-1;
    if (c>=numImages)
      c = numImages-1;
    if (r<0)
      r = 0;
    if (c<0)
      c = 0;
    cv::Mat& m1 = normalImages[r];
    cv::Mat& m2 = normalImages[c];
    printf("frames: %d %d, a:  %f, cutoff: %f, nz: %d, normalDifference: %f, depthDifference: %f \n",
	   r,c, 
	   a, cutoff, nz,
	   normalsComparison.at<float>(c,r), 
	   depthsComparison.at<float>(c,r));
    cv::imshow("normals1", m1);
    cv::imshow("normals2", m2);
    cv::Mat diff = abs(m1-m2);
    cv::imshow("normalsDiff", diff);

    cv::imshow("depths1", depthImages[r]);
    cv::imshow("depths2", depthImages[c]);
    cv::imshow("depthsDiff", 100*abs(depthImages[r]-depthImages[c]));
  }

  return 0; 
  
}
