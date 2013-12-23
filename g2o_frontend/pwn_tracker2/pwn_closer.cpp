#include "pwn_closer.h"
#include "g2o_frontend/pwn_core/pwn_static.h"

namespace pwn_tracker {

  PwnCloserRelation::PwnCloserRelation(MapManager* manager, int id, IdContext* context):
    PwnTrackerRelation(manager, id, context){
  }

  void PwnCloserRelation::serialize(ObjectData& data, IdContext& context){
    PwnTrackerRelation::serialize(data,context);
    data.setBool("accepted", accepted);
    data.setInt("consensusCumInlier", consensusCumInlier);
    data.setInt("consensusCumOutlierTimes", consensusCumOutlierTimes);
    data.setInt("consensusTimeChecked", consensusTimeChecked);
  }

  void PwnCloserRelation::deserialize(ObjectData& data, IdContext& context){
    PwnTrackerRelation::deserialize(data,context);
    accepted = data.getBool("accepted");
    consensusCumInlier=data.getInt("consensusCumInlier");
    consensusCumOutlierTimes = data.getInt("consensusCumOutlierTimes");
    consensusTimeChecked = data.getInt("consensusTimeChecked");
  }


  PwnCloser::PwnCloser(PwnTracker* tracker_) : MapCloser(tracker_->manager()){
    _tracker=tracker_;
    cerr << "tracker: " << _tracker << endl;
    _cache = _tracker->cache();
    cerr << "cache: " << _cache << endl;
    _matcher = _tracker->matcher();
    cerr << "topic: " << _cache->topic() << endl;
    _matcher = _tracker->matcher();
    cerr << "matcher: " << _matcher << endl;
    _robotConfiguration = _tracker->robotConfiguration();
    cerr << "conf: " << _robotConfiguration << endl;
    _frameMinNonZeroThreshold = 3000;// was 3000
    _frameMaxOutliersThreshold = 100;
    _frameMinInliersThreshold = 1000; // was 1000
    _debug = false;
    _selector = 0;
    cerr << "constructed" << endl;
  }


  void PwnCloser::processPartition(std::list<MapNodeBinaryRelation*>& newRelations, 
				   std::set<MapNode*>& otherPartition, 
				   MapNode* current_){
    SensingFrameNode* current = dynamic_cast<SensingFrameNode*>(current_);
    if (otherPartition.count(current)>0)
      return;
    Eigen::Isometry3d iT=current->transform().inverse();
    PwnCloudCache::HandleType f_handle=_cache->get(current);
    //cerr << "FRAME: " << current->seq << endl; 
    for (std::set <MapNode*>::iterator it=otherPartition.begin(); it!=otherPartition.end(); it++){
      SensingFrameNode* other = dynamic_cast<SensingFrameNode*>(*it);
      if (other==current)
	continue;

      Eigen::Isometry3d ig=iT*other->transform();
      PwnCloserRelation* rel = registerNodes(current, other, ig);
      //cerr << "  framesMatched: " << rel << " dc:"  << dc << " nc:" << nc << endl;
      if (rel) {
	cerr << "o";
	newRelations.push_back(rel);
      } else 
	cerr << ".";
    }
    cerr << endl;
  }

  PwnCloserRelation* PwnCloser::registerNodes(SensingFrameNode* keyNode, SensingFrameNode* otherNode, const Eigen::Isometry3d& initialGuess_) {

    // fetch the clouds from the cache
    PwnCloudCache::HandleType _keyCloudHandler = _cache->get(keyNode);
    pwn::Cloud* keyCloud = _keyCloudHandler.get();
    PwnCloudCache::HandleType _otherCloudHandler = _cache->get(otherNode); 
    pwn::Cloud* otherCloud = _otherCloudHandler.get();

    Eigen::Isometry3d keyOffset_, otherOffset_;
    Eigen::Matrix3d   otherCameraMatrix_;
    {
      BaseSensorData* sdata = keyNode->sensorData(_cache->topic());
      if (! sdata) {
	std::cerr << "topic :[" << _cache->topic() << "]" << std::endl;
	throw std::runtime_error("unable to find the required topic for FROM node");
      }
      PinholeImageData* imdata = dynamic_cast<PinholeImageData*>(sdata);
      if (! imdata) {
	throw std::runtime_error("the required topic does not match the requested type");
      }
      keyOffset_ = _robotConfiguration->sensorOffset(imdata->sensor());
    }
    {
      BaseSensorData* sdata = otherNode->sensorData(_cache->topic());
      if (! sdata) {
	std::cerr << "topic :[" << _cache->topic() << "]" << std::endl;
	throw std::runtime_error("unable to find the required topic for TO node");
      }
      PinholeImageData* imdata = dynamic_cast<PinholeImageData*>(sdata);
      if (! imdata) {
	throw std::runtime_error("the required topic does not match the requested type");
      }
      otherOffset_ = _robotConfiguration->sensorOffset(imdata->sensor());
      otherCameraMatrix_ = imdata->cameraMatrix();
    }

    // convert double to float to call the matcher
    Eigen::Isometry3f keyOffset, otherOffset;//, initialGuess;
    Eigen::Matrix3f otherCameraMatrix;
    convertScalar(keyOffset, keyOffset_);
    convertScalar(otherOffset, otherOffset_);
    convertScalar(otherCameraMatrix, otherCameraMatrix_);
    otherCameraMatrix(2,2) = 1;

    PwnMatcherBase::MatcherResult result;
    _matcher->matchClouds(result, 
			  keyCloud, otherCloud, 
			  keyOffset, otherOffset,
			  otherCameraMatrix, _tracker->imageRows(), _tracker->imageCols(), 
			  initialGuess_);

    if(result.image_nonZeros < _frameMinNonZeroThreshold ||
       result.image_outliers > _frameMaxOutliersThreshold || 
       result.image_inliers  < _frameMinInliersThreshold)
      return 0;

    PwnCloserRelation* r=new PwnCloserRelation(_manager);
    r->nodes()[0]=keyNode;
    r->nodes()[1]=otherNode;
    r->fromResult(result);
    return r;
  }


  BOSS_REGISTER_CLASS(PwnCloserRelation);
}


