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
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "highgui.h"
#include <boost/bind.hpp>
#include <fstream>
#include <iostream>
#include "pwn_tracker.h"
using namespace std;
using namespace pwn;
using namespace boss;
using namespace pwn_tracker;



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
    return norm(diff);
  }



struct MatchingCandidate{
  int fromIdx;
  int toIdx;
  PwnTrackerRelation* relation;
  float normalDifference;
  float depthDifference;
  float reprojectionDistance;
  int outliers;
  int inliers;
  cv::Mat  diffRegistered;
  cv::Mat  normalsRegistered;

};

cv::Mat normalsComparison;
cv::Mat depthsComparison;
std::vector<boss::Serializable*> objects;
std::vector<PwnTrackerFrame*> trackerFrames;
std::vector<PwnTrackerRelation*> trackerRelations;
std::vector<cv::Mat> normalImages;
std::vector<cv::Mat> depthImages;
std::vector<MatchingCandidate> matchingCandidates;
int r = 0;
int c = 0;
float a = 0.0;
float cutoff = 0;
float inlierThreshold = 50;
int maxOutliers = 750;

MapManager* load(std::vector<PwnTrackerFrame*>& trackerFrames,
		 std::vector<PwnTrackerRelation*>& trackerRelations,
		 std::vector<Serializable*>& objects,
		 Deserializer& des){
  
  Serializable* o=0;
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
    PwnTrackerRelation* r = dynamic_cast<PwnTrackerRelation*>(o);
    if (r) {
      trackerRelations.push_back(r);
    }
  }
  cerr << "read " << objects.size() << " elements";
  cerr << "manager: " << manager << endl;
  cerr << "# frames: " << trackerFrames.size() << endl;
  cerr << "# relations: " << trackerRelations.size() << endl;
  return manager;
}

void save(std::vector<Serializable*>& objects,
	  MapManager* manager,
	  std::vector<MatchingCandidate>& candidates,
	  Serializer& ser,
	  int maxOutliers){
  for (size_t i = 0; i<objects.size(); i++)
    ser.writeObject(*objects[i]);
  for(size_t i = 0; i<candidates.size(); i++){
    MatchingCandidate& c = candidates[i];
    if (c.outliers<maxOutliers){
      manager->addRelation(c.relation);
      ser.writeObject(*c.relation);
    }
  }
}


void g2oSave(std::vector<PwnTrackerFrame*>& trackerFrames, 
	     std::vector<PwnTrackerRelation*>& trackerRelations,
	     std::vector<MatchingCandidate> candidates,
	     int maxOutliers){
  
  std::map<PwnTrackerFrame*, g2o::VertexSE3*> vertexMap;
  g2o::OptimizableGraph graph;

  for (size_t i=0; i<trackerFrames.size(); i++){
    PwnTrackerFrame* f = trackerFrames[i];
    g2o::VertexSE3* v = new g2o::VertexSE3;
    v->setId(i);
    v->setEstimate(f->transform());
    graph.addVertex(v);
    vertexMap.insert(make_pair(f,v));
  }

  for (size_t i = 0; i<trackerRelations.size(); i++){
    PwnTrackerRelation* rel = trackerRelations[i];
    g2o::EdgeSE3* e = new g2o::EdgeSE3;
    g2o::VertexSE3* vFrom = vertexMap[(PwnTrackerFrame*)rel->nodes()[0]];
    g2o::VertexSE3* vTo = vertexMap[(PwnTrackerFrame*)rel->nodes()[1]];
    e->setVertex(0,vFrom);
    e->setVertex(1,vTo);
    e->setMeasurement(rel->transform());
    e->setInformation(rel->informationMatrix()*1000);
    graph.addEdge(e);
  }

  for(size_t i = 0; i<candidates.size(); i++){
    MatchingCandidate& c = candidates[i];
    if (c.outliers<maxOutliers){
      PwnTrackerRelation* rel = c.relation;
      g2o::VertexSE3* vFrom = vertexMap[(PwnTrackerFrame*)rel->nodes()[0]];
      g2o::VertexSE3* vTo = vertexMap[(PwnTrackerFrame*)rel->nodes()[1]];
      // Eigen::Isometry3d delta=vFrom->estimate().inverse()*vTo->estimate()*rel->transform().inverse();
      // AngleAxisd aa(delta.linear());
      // if (aa.angle()>M_PI/2 || delta.translation().norm()>1)
      // 	continue;
      g2o::EdgeSE3* e = new g2o::EdgeSE3;
      e->setVertex(0,vFrom);
      e->setVertex(1,vTo);
      e->setMeasurement(rel->transform());
      e->setInformation(rel->informationMatrix());
      graph.addEdge(e);
      char filename[1024];
      sprintf(filename, "match-%05d-%05d.pgm", vFrom->id(), vTo->id());
      cv::imwrite(filename, c.diffRegistered);
    }
  }
  graph.save("out.g2o");
}

void fetchNormalThumbnails(std::vector<cv::Mat>& normalImages, 
			   std::vector<PwnTrackerFrame*>& trackerFrames,
			   Deserializer& des){
  // load the images
  for(size_t i = 0; i<trackerFrames.size(); i++){
    cv::Mat normalImg;
    PwnTrackerFrame *n = trackerFrames[i];
    ImageBLOB* thumbnail = n->normalThumbnail.get();
    thumbnail->cvImage().convertTo(normalImg, CV_32FC3);
    normalImg=normalImg-127.0f;
    normalImg=normalImg*(1./255);
    normalImages.push_back(normalImg);
    delete thumbnail;
  }
  cerr << "done normals" << endl;
}

void fetchDepthThumbnails(std::vector<cv::Mat>& depthImages, 
			  std::vector<PwnTrackerFrame*>& trackerFrames,
			  Deserializer& des){
  for(size_t i = 0; i<trackerFrames.size(); i++){
    PwnTrackerFrame *n = trackerFrames[i];
    ImageBLOB* thumbnail = n->depthThumbnail.get();
    cv::Mat depthImg;
    thumbnail->cvImage().convertTo(depthImg,CV_32FC1);
    depthImg = depthImg *(1./65535);
    depthImages.push_back(depthImg);
    delete thumbnail;
  }
  cerr << "done depths" << endl;
}

cv::Mat generateDepthComparison(std::vector<PwnTrackerFrame*>& trackerFrames,
				std::vector<cv::Mat>& depthImages){
  int numImages = trackerFrames.size();
  cv::Mat depthsComparison(numImages, numImages, CV_32FC1);
  float maxDepthDifference = 0;
  for (int i = 0; i<depthsComparison.rows; i++) {
    depthsComparison.at<float>(i,i)=0;
    for (int j = 0; j<i; j++){
      cv::Mat& m1  = depthImages[i];
      cv::Mat& m2  = depthImages[j];
      float f = compareDepths(m1,m2);
      //cerr << "val: " << i << " " << j << " " << f << endl;
      depthsComparison.at<float>(j,i)=f;
      depthsComparison.at<float>(i,j)=f;
      if (maxDepthDifference<f)
	maxDepthDifference = f;
    }
  }
  cerr << "maxDepthDifference: " << maxDepthDifference << endl;
  maxDepthDifference = 1./maxDepthDifference;
  for (int i = 0; i<depthsComparison.rows; i++)
    for (int j = 0; j<depthsComparison.rows; j++){
      depthsComparison.at<float>(i,j)*=maxDepthDifference;
    }
  return depthsComparison;
}

cv::Mat generateNormalComparison(std::vector<PwnTrackerFrame*>& trackerFrames,
				  std::vector<cv::Mat>& normalImages){
  int numImages = trackerFrames.size();
  cv::Mat normalsComparison(numImages, numImages, CV_32FC1);
  float maxNormalDifference = 0;
  for (int i = 0; i<normalsComparison.rows; i++) {
    normalsComparison.at<float>(i,i)=0;
    for (int j = 0; j<i; j++){
      cv::Mat& m1  = normalImages[i];
      cv::Mat& m2  = normalImages[j];
      float f = compareNormals(m1,m2);
      //cerr << "val: " << i << " " << j << " " << f << endl;
      normalsComparison.at<float>(j,i)=f;
      normalsComparison.at<float>(i,j)=f;
      if (maxNormalDifference<f)
	maxNormalDifference = f;
    }
  }
  cerr << "maxNormalDifference: " << maxNormalDifference << endl;
  maxNormalDifference = 1./maxNormalDifference;
  for (int i = 0; i<normalsComparison.rows; i++)
    for (int j = 0; j<normalsComparison.rows; j++){
      normalsComparison.at<float>(i,j)*=maxNormalDifference;
    }
  return normalsComparison;
}

void constructCandidates(std::vector<MatchingCandidate>& candidates, 
			 cv::Mat depthComparison, float depthThreshold,
			 cv::Mat normalsComparison, float normalsThreshold){
  int r = depthComparison.rows;
  int c = depthComparison.cols;
  for (int i=0; i<r; i++)
    for (int j=0; j<i; j++){
      float d = depthComparison.at<float>(i,j);
      float n = normalsComparison.at<float>(i,j);
      if (d<depthThreshold && n < normalsThreshold){
	MatchingCandidate mc;
	mc.fromIdx = i;
	mc.toIdx = j;
	mc.relation  = 0;
	mc.normalDifference = n;
	mc.depthDifference = d;
	candidates.push_back(mc);
      }
    }
}

void alignFrames(Aligner* aligner, DepthImageConverter* converter,
		 MatchingCandidate& candidate,
		 std::vector<PwnTrackerFrame*>& trackerFrames,
		 MapManager* manager){
  PwnTrackerFrame* from = trackerFrames[candidate.fromIdx];
  PwnTrackerFrame* to = trackerFrames[candidate.toIdx];
  if (from == to)
    return;

  //pwn::Frame* fromCloud = from->cloud.get();
  //pwn::Frame* toCloud = to->cloud.get();
  boss_logger::ImageBLOB* fromDepthBlob = from->depthImage.get();
  boss_logger::ImageBLOB* toDepthBlob = to->depthImage.get();
  
  PinholePointProjector* projector = (PinholePointProjector*)converter->_projector;
  pwn::DepthImage fromDepth, toDepth;
  pwn::Frame fromCloud, toCloud; 
  Eigen::Isometry3f fromOffset, toOffset;
  Eigen::Matrix3f fromCameraMatrix, toCameraMatrix;
  int scale = 8;
  int r, c;
  
  {
    fromDepth.fromCvMat(fromDepthBlob->cvImage());
    convertScalar(fromOffset, from->sensorOffset);
    convertScalar(fromCameraMatrix, from->cameraMatrix);
    projector->setCameraMatrix(fromCameraMatrix);
    pwn::DepthImage scaledFromDepth;
    DepthImage::scale(scaledFromDepth, fromDepth, scale);
    projector->scale (1./scale);
    converter->compute(fromCloud, scaledFromDepth, fromOffset);
  }

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
  }
  
  PinholePointProjector* alignerProjector = (PinholePointProjector*)aligner->projector();
  aligner->setReferenceSensorOffset(fromOffset);
  aligner->setCurrentSensorOffset(toOffset);
  aligner->setInitialGuess(Eigen::Isometry3f::Identity());
  alignerProjector->setImageSize(r,c);
  alignerProjector->setCameraMatrix(projector->cameraMatrix());
  aligner->correspondenceFinder()->setImageSize(r,c);
  aligner->setReferenceFrame(&fromCloud);
  aligner->setCurrentFrame(&toCloud);
  aligner->align();

  Eigen::Isometry3d relationMean;
  convertScalar(relationMean, aligner->T());
  PwnTrackerRelation* rel = new PwnTrackerRelation(manager);
  rel->setTransform(relationMean);
  rel->setInformationMatrix(Eigen::Matrix<double, 6,6>::Identity());
  rel->setTo(from);
  rel->setFrom(to);
  candidate.relation = rel;
  delete toDepthBlob;
  delete fromDepthBlob;

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
  candidate.diffRegistered = diff.clone();
  int nonZeros = countNonZero(mask);
  float sum=0;
  for (int i = 0; i<diff.rows; i++)
    for (int j = 0; j<diff.cols; j++){
      float d = candidate.diffRegistered.at<float>(i,j);
      sum +=d;
    }
  candidate.relation = rel;
  candidate.reprojectionDistance = sum/nonZeros;
}



void scoreCandidate(MatchingCandidate& candidate, float inlierThreshold){
  cv::Mat& diff = candidate.diffRegistered;
  int inliers = 0;
  int outliers = 0;
  //int region = 3;
  for (int i = 0; i<diff.rows; i++)
    for (int j = 0; j<diff.cols; j++){
      // int dmin = 2*inlierThreshold;
      // for (int k=-region; k<region; k++)
      // 	for (int l=-region; l<region; l++){
      // 	  int a = i+k;
      // 	  int b = j+l;
      // 	  if (a<0 || a>diff.rows)
      // 	    continue;
      // 	  if (b<0 || b>diff.rows)
      // 	    continue;
      // 	  float d = diff.at<float>(i,j);
      // 	  if (d>0 && dmin>d)
      // 	    dmin = d;
      // 	}
	  
      float d = diff.at<float>(i,j);
      if (d < inlierThreshold)
	inliers ++;
      else
	outliers++;
    }
  candidate.inliers = inliers;
  candidate.outliers = outliers;
}

void autoMatch(Aligner* aligner, DepthImageConverter* converter,
	       std::vector<MatchingCandidate>& candidates,
	       std::vector<PwnTrackerFrame*>& trackerFrames,
	       MapManager* manager){
  for (size_t i=0; i<candidates.size(); i++){
    alignFrames(aligner, converter, candidates[i], trackerFrames, manager);
    cerr << "matching frame " << i << "/" << candidates.size() 
	 << " indices: " << candidates[i].fromIdx << "," << candidates[i].toIdx
	 << " rd: " << candidates[i].reprojectionDistance << endl; 
  }
}
/*
std::vector<int> cluster(std::vector<MatchingCandidate>& candidates, std::vector<PwnTrackerFrame*>& trackerFrames){
  std::vector<int> parents(trackerFrames.size(), -1);
  for (size_t i = 0; i<candidates.size(); i++){
    MatchingCandidate& match;
    int from = match.fromIdx;
    int to = match.toIdx;
    int& pFrom = parents[from];
    int& pTo = parents[to];
    if (pFrom>-1 && pTo>-1){
      if (pFrom>pTo)
	pFrom = pTo;
      else
	pTo = pFrom;
    } else if (pFrom == -1 && pTo >-1) {
      pFrom = pTo;
    } else if (pFrom > -1 && pTo ==-1) {
      pTo = pFrom;
    } else {
      pFrom = -1;
      pTo = from;
    }
  }
  return parents;
  //; scan to find the clusters:
}
*/


void scoreCandidates(std::vector<MatchingCandidate>& candidates, float inlierThreshold){
  for (size_t i=0; i<candidates.size(); i++){
    scoreCandidate(candidates[i], inlierThreshold);
    cerr << "scoring frame " << i << "/" << candidates.size() 
	 << " indices: " << candidates[i].fromIdx << "," << candidates[i].toIdx
	 << " rd: " << candidates[i].reprojectionDistance 
	 << " inliers: " << candidates[i].inliers  
	 << " outliers: " << candidates[i].outliers << endl; 
  }
}

void mouseEvent(int evt, int x, int y, int flags, void* param){
    if(evt==CV_EVENT_LBUTTONDOWN){
      r = x;
      c = y;
      cerr << "r,c:" << r << " " << c << endl;
    }
}

int main(int argc, char** argv){
  if (argc<4) {
    cerr << " u should provide a config file and an input file" << endl;
    return 0;
  }

  Aligner* aligner;
  DepthImageConverter* converter;
  std::vector<Serializable*> instances = readConfig(aligner, converter, argv[1]);
  cerr << "config loaded" << endl;
  cerr << " aligner:" << aligner << endl;
  cerr << " converter:" << converter << endl;

  Deserializer des;
  Serializer  ser;
  des.setFilePath(argv[2]);
  // load the log
  MapManager* manager = load(trackerFrames, trackerRelations, objects, des);

  cerr << "generating fingerprints" << endl;
  fetchDepthThumbnails(depthImages, trackerFrames, des);
  fetchNormalThumbnails(normalImages, trackerFrames, des);

  cerr << "generating depth distance matrix" << endl;
  depthsComparison = generateDepthComparison(trackerFrames, depthImages);

  cerr << "generating normal distance matrix" << endl;
  normalsComparison = generateNormalComparison(trackerFrames, normalImages);


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
  
  cv::Mat regDiff;
      
  int numImages = trackerFrames.size();
  int ca = 0;
  while(ca!=27){
    cv::Mat mix = depthsComparison*a + normalsComparison*(1-a);
    cv::Mat shownImage = mix>cutoff;
    int nz = numImages*numImages-countNonZero(shownImage)-numImages;
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
    case 1179563: maxOutliers += 1; break;
    case 1179565: maxOutliers -= 1; break;
    case 1048673: a+=0.01; break; // 'a'
    case 1048674: a-=0.01; break; // 'b'
    case 1048608:
      {// ' space '
      MatchingCandidate cand;
      cand.fromIdx = r;
      cand.toIdx = c;
      alignFrames(aligner, converter,
		  cand, trackerFrames, manager);
      regDiff=cand.diffRegistered*(1000.0f/65535.0f);
      scoreCandidate(cand, inlierThreshold);
      cerr << " indices: " << cand.fromIdx << "," << cand.toIdx
	   << " rd: " << cand.reprojectionDistance 
	   << " inliers: " << cand.inliers  
	   << " outliers: " << cand.outliers 
	   << " dmax: " << *std::max_element(cand.diffRegistered.begin<float>(),cand.diffRegistered.end<float>()) <<  endl; 
      cv::imshow("rectDiff", regDiff);
      
      }
      break;
    case 1048675: // 'c'
      matchingCandidates.clear();
      constructCandidates(matchingCandidates, 
			  depthsComparison, 1, 
			  normalsComparison, cutoff);
      cerr << "found " << matchingCandidates.size() << " matching candidates" << endl;
      break;
    case 1048676: // 'd'
      autoMatch(aligner, converter,
		matchingCandidates,
		trackerFrames,
		manager);
	break;
    case 1048677: // 'e'
      scoreCandidates(matchingCandidates, inlierThreshold);
      break;
    case 1048678: // 'f'
      //ser.setFilePath(argv[3]);
      //save(objects,manager,matchingCandidates,ser,200);
      g2oSave(trackerFrames,trackerRelations,matchingCandidates,maxOutliers);
      break;
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
    printf("frames: %d %d, a:  %f, cutoff: %f, maxOutliers %d, nz: %d, normalDifference: %f, depthDifference: %f \n",
	   r,c, 
	   a, cutoff, maxOutliers, nz,
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
