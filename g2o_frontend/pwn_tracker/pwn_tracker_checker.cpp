
#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "opencv2/highgui/highgui.hpp"
#include "g2o_frontend/boss_map/boss_map_g2o_reflector.h"
#include <fstream>
#include <iostream>
#include <queue>
#include "pwn_tracker.h"
#include "g2o_frontend/boss_map/boss_map_utils.h"

using namespace std;
using namespace pwn;
using namespace boss;
using namespace boss_map;
using namespace pwn_tracker;

std::vector<boss::Serializable*> objects;
std::vector<PwnTrackerFrame*> trackerFrames;
std::vector<PwnTrackerRelation*> trackerRelations;

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


void seekForMatches(std::set<MapNode*> nodes, MapNode* current_){
  PwnTrackerFrame* current = dynamic_cast<PwnTrackerFrame*>(current_);

  cv::Mat currentNormalThumbnail;
  ImageBLOB* currentNormalThumbnailBLOB = current->normalThumbnail.get();
  currentNormalThumbnailBLOB->cvImage().convertTo(currentNormalThumbnail, CV_32FC3);
  currentNormalThumbnail=currentNormalThumbnail-127.0f;
  currentNormalThumbnail=currentNormalThumbnail*(1./255);

  cv::Mat currentDepthThumbnail;
  ImageBLOB* currentDepthThumbnailBLOB = current->depthThumbnail.get();
  currentDepthThumbnailBLOB->cvImage().convertTo(currentDepthThumbnail, CV_32FC3);
  currentDepthThumbnail=currentDepthThumbnail-127.0f;
  currentDepthThumbnail=currentDepthThumbnail*(1./255);

  ImageBLOB* currentDepthBLOB = current->depthImage.get();
  cv::Mat currentDepth = currentDepthBLOB->cvImage();

  for (std::set <MapNode*>::iterator it=nodes.begin(); it!=nodes.end(); it++){
    PwnTrackerFrame* other = dynamic_cast<PwnTrackerFrame*>(*it);
    if (other==current)
      continue;
    cv::Mat otherNormalThumbnail;
    ImageBLOB* otherNormalThumbnailBLOB = other->normalThumbnail.get();
    otherNormalThumbnailBLOB->cvImage().convertTo(otherNormalThumbnail, CV_32FC3);
    otherNormalThumbnail=otherNormalThumbnail-127.0f;
    otherNormalThumbnail=otherNormalThumbnail*(1./255);

    cv::Mat otherDepthThumbnail;
    ImageBLOB* otherDepthThumbnailBLOB = other->depthThumbnail.get();
    otherDepthThumbnailBLOB->cvImage().convertTo(otherDepthThumbnail, CV_32FC3);
    otherDepthThumbnail=otherDepthThumbnail-127.0f;
    otherDepthThumbnail=otherDepthThumbnail*(1./255);

    float dc = compareNormals(currentNormalThumbnail, otherNormalThumbnail);
    float nc = compareDepths(currentDepthThumbnail, otherDepthThumbnail);
    cerr << "matching" << other << " " << current << ", dc:" << dc << ", nc:" << nc << endl;
    delete otherDepthThumbnailBLOB;
    delete otherNormalThumbnailBLOB;
  }
  cerr << "done" << endl;

  delete currentDepthBLOB;
  delete currentDepthThumbnailBLOB;
  delete currentNormalThumbnailBLOB;
}

MapManager* load(std::vector<PwnTrackerFrame*>& trackerFrames,
		 std::vector<PwnTrackerRelation*>& trackerRelations,
		 std::vector<Serializable*>& objects,
		 Deserializer& des){
  
  DistancePoseAcceptanceCriterion criterion;
  criterion.setRotationalDistance(M_PI/8);
  criterion.setTranslationalDistance(.5);
  Serializable* o=0;
  boss_map::MapManager* manager = 0;
  PwnTrackerFrame* lastTrackerFrame = 0;
  int nodeCount = 0;
  while(o=des.readObject()){
    objects.push_back(o);
    boss_map::MapManager* m = dynamic_cast<boss_map::MapManager*>(o);
    if (m) {
      manager = m;
      criterion.setManager(m);
    }
    PwnTrackerFrame* f = dynamic_cast<PwnTrackerFrame*>(o);
    if (f) {
      trackerFrames.push_back(f);
      lastTrackerFrame = f;
      nodeCount++;
    }
    PwnTrackerRelation* r = dynamic_cast<PwnTrackerRelation*>(o);
    if (r) {
      // got a new relation, let's see how many clusters I have in the pool
      trackerRelations.push_back(r);

      std::set<MapNode*> selectedNodes;
      criterion.setReferencePose(lastTrackerFrame->transform());
      selectNodes(selectedNodes,&criterion);
      std::vector< std::set<MapNode*> > partitions;
      makePartitions(partitions, selectedNodes);
      cerr << "node: " << nodeCount 
	   << ", neighbors: " << selectedNodes.size() 
	   << "partitions: " << partitions.size() << endl;
      for (size_t i=0; i<partitions.size(); i++){
	cerr << "  " << i << "(" << partitions[i].size() << ")" << endl;
	seekForMatches(partitions[i], lastTrackerFrame);
      }
    }
    
  }
  cerr << "read " << objects.size() << " elements";
  cerr << "manager: " << manager << endl;
  cerr << "# frames: " << trackerFrames.size() << endl;
  cerr << "# relations: " << trackerRelations.size() << endl;
  return manager;
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

  return 0;
}
