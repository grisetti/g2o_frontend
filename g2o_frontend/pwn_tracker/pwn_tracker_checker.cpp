
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
#include "cache.h"

using namespace std;
using namespace pwn;
using namespace boss;
using namespace boss_map;
using namespace pwn_tracker;

std::vector<boss::Serializable*> objects;
std::vector<PwnTrackerRelation*> trackerRelations;
std::map<int, PwnTrackerFrame*> trackerFrames;

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


PwnCache* cache = 0;
Aligner* aligner = 0;



struct PwnTrackerClosureRelation: public PwnTrackerRelation {
  PwnTrackerClosureRelation(MapManager* manager=0, int id=-1, IdContext* context = 0):
    PwnTrackerRelation(manager, id, context){
    reprojectionInliers = 0;
    reprojectionOutliers = 0;
  }

  virtual void serialize(ObjectData& data, IdContext& context){
    PwnTrackerRelation::serialize(data,context);
    data.setInt("reprojectionInliers", reprojectionInliers);
    data.setInt("reprojectionOutliers", reprojectionOutliers);
  }

  virtual void deserialize(ObjectData& data, IdContext& context){
    PwnTrackerRelation::deserialize(data,context);
    reprojectionInliers = data.getInt("reprojectionInliers");
    reprojectionOutliers = data.getInt("reprojectionOutliers");
  }

  int reprojectionInliers;
  int reprojectionOutliers;
};

struct MatchingResult{
  PwnTrackerFrame* from;
  PwnTrackerFrame* to;
  PwnTrackerClosureRelation* relation;
  float normalDifference;
  float depthDifference;
  float reprojectionDistance;
  int outliers;
  int inliers;
  cv::Mat  diffRegistered;
  cv::Mat  normalsRegistered;
};

void matchFrames(MatchingResult& result,
		 Aligner* aligner, 
		 PwnTrackerFrame* from, PwnTrackerFrame* to, 
		 pwn::Frame* fromCloud, pwn::Frame* toCloud,
		 const Eigen::Isometry3d& initialGuess,
		 int scale=4){
  if (from == to)
    return; 
  MapManager* manager = from->manager();
   
  cerr <<  " matching frames: " << from->seq << " " << to->seq << endl;
    
  
  Eigen::Isometry3f fromOffset, toOffset;
  Eigen::Matrix3f fromCameraMatrix, toCameraMatrix;

  convertScalar(fromOffset, from->sensorOffset);
  convertScalar(fromCameraMatrix, from->cameraMatrix);

  convertScalar(toOffset, to->sensorOffset);
  convertScalar(toCameraMatrix, to->cameraMatrix);
    
  PinholePointProjector* projector = (PinholePointProjector*)aligner->projector();
  int r, c;
  
  aligner->setReferenceSensorOffset(fromOffset);
  aligner->setCurrentSensorOffset(toOffset);
  aligner->setInitialGuess(Eigen::Isometry3f::Identity());
  projector->setCameraMatrix(toCameraMatrix);
  projector->setImageSize(to->imageRows,to->imageCols);
  projector->scale(1./scale);
    
  // cerr << "cameraMatrix: " << endl;
  // cerr << projector->cameraMatrix() << endl;
  r = projector->imageRows();
  c = projector->imageCols();
  
  Eigen::Isometry3f ig;
  convertScalar(ig, initialGuess);
  aligner->setInitialGuess(ig);
  aligner->correspondenceFinder()->setImageSize(r,c);
  aligner->setReferenceFrame(fromCloud);
  aligner->setCurrentFrame(toCloud);
  aligner->align();
  
  Eigen::Isometry3d relationMean;
  convertScalar(relationMean, aligner->T());
  PwnTrackerClosureRelation* rel = new PwnTrackerClosureRelation(manager);
  rel->setTransform(relationMean);
  rel->setInformationMatrix(Eigen::Matrix<double, 6,6>::Identity());
  rel->setTo(from);
  rel->setFrom(to);
  result.relation = rel;

  DepthImage 
    currentDepthThumb = aligner->correspondenceFinder()->currentDepthImage(),
    referenceDepthThumb = aligner->correspondenceFinder()->referenceDepthImage();
  cv::Mat currentRect, referenceRect;
  currentDepthThumb.toCvMat(currentRect);
  referenceDepthThumb.toCvMat(referenceRect);
  cv::Mat mask = (currentRect>0) & (referenceRect>0);
  currentRect.convertTo(currentRect, CV_32FC1);
  referenceRect.convertTo(referenceRect, CV_32FC1);
  mask.convertTo(mask, currentRect.type());
  cv::Mat diff = abs(currentRect-referenceRect)&mask;
  result.diffRegistered = diff.clone();
  int nonZeros = countNonZero(mask);
  float sum=0;
  for (int i = 0; i<diff.rows; i++)
    for (int j = 0; j<diff.cols; j++){
      float d = result.diffRegistered.at<float>(i,j);
      sum +=d;
    }
  result.relation = rel;
  result.reprojectionDistance = sum/nonZeros;

  cerr << "  initialGuess : " << t2v(ig).transpose() << endl;
  cerr << "  transform    : " << t2v(aligner->T()).transpose() << endl;
  cerr << "  inliers      : " << aligner->inliers() <<  "/" << r*c << endl;
  cerr << "  reprojection : " << sum/nonZeros;
}

std::vector<MatchingResult> results;

void seekForMatches(std::set<MapNode*> nodes, MapNode* current_){
  PwnTrackerFrame* current = dynamic_cast<PwnTrackerFrame*>(current_);

  if (nodes.count(current)>0)
    return;
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

  Eigen::Isometry3d iT=current->transform().inverse();
  pwn::Frame* f=cache->get(current);
  cache->lock(current);
  cerr << "FRAME: " << current->seq << endl; 
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

    pwn::Frame* f2=cache->get(other);
    cerr << "INNER FRAME: " << other->seq << endl; 
  
    
    Eigen::Isometry3d ig=iT*other->transform();
    MatchingResult result;
    results.push_back(result);
    matchFrames(results.back(), aligner, current, other, f, f2, ig);
    
    //delete otherDepthThumbnailBLOB;
    //delete otherNormalThumbnailBLOB;
  }
  cache->unlock(current);
  cerr << "done" << endl;

  //delete currentDepthBLOB;
  //delete currentDepthThumbnailBLOB;
  //delete currentNormalThumbnailBLOB;
}


MapManager* load(std::map<int, PwnTrackerFrame*>& trackerFrames,
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
      trackerFrames.insert(make_pair(f->seq,f));
      cache->addEntry(f);
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

  DepthImageConverter* converter;
  std::vector<Serializable*> instances = readConfig(aligner, converter, argv[1]);
  cerr << "config loaded" << endl;
  cerr << " aligner:" << aligner << endl;
  cerr << " converter:" << converter << endl;

  Deserializer des;
  Serializer  ser;
  des.setFilePath(argv[2]);
  // load the log

  cache = new PwnCache(converter,4,50);
  MapManager* manager = load(trackerFrames, trackerRelations, objects, des);

  cerr << "Cache Statistics" << endl;
  cerr << "  hits:" << cache->hits() << endl;
  cerr << "  misses:" << cache->misses() << endl;

  return 0;
}
