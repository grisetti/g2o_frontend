
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


Frame* makeFrame(DepthImageConverter* converter, PwnTrackerFrame* trackerFrame, int scale = 8){
  boss_logger::ImageBLOB* depthBLOB = trackerFrame->depthImage.get();
  PinholePointProjector* projector = (PinholePointProjector*)converter->_projector;
  projector->setImageSize(trackerFrame->imageRows, trackerFrame->imageCols);
  pwn::DepthImage depth;
  pwn::Frame* cloud=new pwn::Frame;
  Eigen::Matrix3f cameraMatrix;
  Eigen::Isometry3f offset;
  depth.fromCvMat(depthBLOB->cvImage());
  convertScalar(offset, trackerFrame->sensorOffset);
  convertScalar(cameraMatrix, trackerFrame->cameraMatrix);
  projector->setCameraMatrix(cameraMatrix);
  pwn::DepthImage scaledDepth;
  DepthImage::scale(scaledDepth, depth, scale);
  projector->scale (1./scale);
  converter->compute(*cloud, scaledDepth, offset);
  delete depthBLOB;
  return cloud;
}

class PwnCache;


class PwnFrameCacheEntry {
public:
  PwnFrameCacheEntry(PwnCache* cache_, PwnTrackerFrame* frame_){
    _frame = frame_;
    _cache = cache_;
    _instance = 0;
    _isLocked = 0;
    _lastAccess = 0;
  }

  pwn::Frame* get(size_t access);
  bool isLoaded() const {return _instance;}
  inline bool release() {
    if (_instance && ! _isLocked) {
      delete _instance; 
      _instance = 0; 
      return true;
    } 
    return false;
  }

  inline bool isLocked() {
    return _instance && _isLocked;
  }

  void lock() {
    if (_instance) 
      _isLocked = true;
    else 
      _isLocked = false;
  }

  void unlock() {
    _isLocked = false; 
  }

  inline int lastAccess() const {return _lastAccess;}

protected:
  pwn::Frame* _instance;
  PwnTrackerFrame* _frame;
  PwnCache* _cache;
  bool _isLocked;
  size_t _lastAccess;
};

class PwnCache{
public:
  PwnCache(DepthImageConverter* converter_, int scale_, int maxActiveEntries){
    _converter = converter_;
    _scale = scale_;
    _lastAccess = 0;
    _hits = 0;
    _misses = 0;
    _maxElements = maxActiveEntries;
  }
  bool addEntry(PwnTrackerFrame* frame){
    //cerr << "adding entry for frame" << frame << endl;
    if (_entries.count(frame))
      return false;
    PwnFrameCacheEntry* entry = new PwnFrameCacheEntry(this, frame);
    _entries.insert(make_pair(frame, entry));
    //cerr << "DONE" << frame << endl;
    return true;
  }

  pwn::Frame* get(PwnTrackerFrame* frame) {
    // seek if you have it in the entries;
    std::map<PwnTrackerFrame*, PwnFrameCacheEntry*>::iterator it = _entries.find(frame);
    if (it==_entries.end())
      return 0;
    PwnFrameCacheEntry* entry = it->second;
    if (entry->isLoaded()){
      _hits++;
      return entry->get(_lastAccess++);
    } 
    if (_active.size()>_maxElements){
      if (! makeRoom() )
	throw std::runtime_error("no room in cache, no disposable element. Aborting");
    }
    //cerr << "inserting new entry in the pool" << endl;
      _misses++;
    _active.insert(entry);
    return entry->get(_lastAccess++);
  }

  inline DepthImageConverter* converter() {return _converter;}
  inline int scale() const {return _scale;}
  inline int hits() const { return _hits; }
  inline int misses() const { return _misses; }

  bool makeRoom() {
    // seek for the oldest element in the set that is not locked
    PwnFrameCacheEntry* oldest = 0;
    for (std::set<PwnFrameCacheEntry*>::iterator it=_active.begin(); it!=_active.end(); it++){
      PwnFrameCacheEntry* entry = *it;
      if (entry->isLocked())
	continue;
      if(! oldest) {
	oldest = entry;
	continue;
      }
      if (oldest->lastAccess()>entry->lastAccess())
	oldest = entry;
    }
    if (oldest) {
      oldest->release();
      _active.erase(oldest);
      return true;
    }
    return false;
  }
protected:
  std::map<PwnTrackerFrame*, PwnFrameCacheEntry*> _entries;
  std::set<PwnFrameCacheEntry*> _active;
  DepthImageConverter* _converter;
  int _scale;
  size_t _maxElements;
  size_t _lastAccess;
  size_t _hits;
  size_t _misses;
};


pwn::Frame* PwnFrameCacheEntry::get(size_t access) {
  if (_instance)
    return _instance;
  _instance = makeFrame(_cache->converter(), _frame, _cache->scale());
  //cerr << "loading frame: " << _instance  << endl;
  _lastAccess = access;
  return _instance;
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
  return norm(diff);
}


PwnCache* cache = 0;


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


  pwn::Frame* f=cache->get(current);
  cerr << "FRAME: " << f << endl; 
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
    cerr << "INNER FRAME: " << f2 << endl; 
 
    //delete otherDepthThumbnailBLOB;
    //delete otherNormalThumbnailBLOB;
  }
  cerr << "done" << endl;

  //delete currentDepthBLOB;
  //delete currentDepthThumbnailBLOB;
  //delete currentNormalThumbnailBLOB;
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

  cache = new PwnCache(converter,4,50);
  MapManager* manager = load(trackerFrames, trackerRelations, objects, des);

  cerr << "Cache Statistics" << endl;
  cerr << "  hits:" << cache->hits() << endl;
  cerr << "  misses:" << cache->misses() << endl;

  return 0;
}
