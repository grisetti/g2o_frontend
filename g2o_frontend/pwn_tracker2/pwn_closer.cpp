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


  PwnCloser::PwnCloser(PwnMatcherBase* matcher_,
	      PwnCloudCache* cache_,
	      MapManager* manager_,
	      RobotConfiguration* configuration_,
	      int id, boss::IdContext* context) : 
    MapCloser(0,id, context){
    _frameMinNonZeroThreshold = 3000;// was 3000
    _frameMaxOutliersThreshold = 100;
    _frameMinInliersThreshold = 1000; // was 1000
    _debug = false;
    _selector = 0;
    _enabled = true;
    _cache = cache_;
    setMatcher(matcher_);
    setManager(manager_);
    setRobotConfiguration(configuration_);
    setCache(cache_);
  }

  void PwnCloser::serialize(boss::ObjectData& data, boss::IdContext& context){
    MapCloser::serialize(data,context);
    data.setPointer("matcher", _matcher);
    data.setPointer("cache", _cache);
    data.setInt("frameMinNonZeroThreshold", _frameMinNonZeroThreshold);
    data.setInt("frameMaxOutliersThreshold", _frameMaxOutliersThreshold);
    data.setInt("frameMinInliersThreshold", _frameMinInliersThreshold);
  }
    
  
  void PwnCloser::deserialize(boss::ObjectData& data, boss::IdContext& context){
    MapCloser::deserialize(data,context);
    data.getReference("matcher").bind(_matcher);
    data.getReference("cache").bind(_cache);
    _frameMinNonZeroThreshold = data.getInt("frameMinNonZeroThreshold");
    _frameMaxOutliersThreshold = data.getInt("frameMaxOutliersThreshold");
    _frameMinInliersThreshold = data.getInt("frameMinInliersThreshold");
   }


  /*
  void PwnCloser::deserializeComplete(){
    boss::Identifiable::deserializeComplete();
    if (_tracker)
      setTracker(_tracker);
 }
  */
  void PwnCloser::process(Serializable* s) {
    if (_enabled)
      MapCloser::process(s);
    else
      put(s);
  }

  void PwnCloser::processPartition(std::list<MapNodeBinaryRelation*>& newRelations, 
				   std::set<MapNode*>& otherPartition, 
				   MapNode* current_){
    SyncSensorDataNode* current = dynamic_cast<SyncSensorDataNode*>(current_);
    if (otherPartition.count(current)>0)
      return;
    Eigen::Isometry3d iT=current->transform().inverse();
    PwnCloudCache::HandleType f_handle=_cache->get(current);
    //cerr << "FRAME: " << current->seq << endl; 
    for (std::set <MapNode*>::iterator it=otherPartition.begin(); it!=otherPartition.end(); it++){
      SyncSensorDataNode* other = dynamic_cast<SyncSensorDataNode*>(*it);
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

  PwnCloserRelation* PwnCloser::registerNodes(SyncSensorDataNode* keyNode, SyncSensorDataNode* otherNode, const Eigen::Isometry3d& initialGuess_) {

    // fetch the clouds from the cache
    PwnCloudCache::HandleType _keyCloudHandler = _cache->get(keyNode);
    CloudWithImageSize* keyCloud = _keyCloudHandler.get();
    PwnCloudCache::HandleType _otherCloudHandler = _cache->get(otherNode); 
    CloudWithImageSize* otherCloud = _otherCloudHandler.get();
    _scaledImageSize = otherCloud->imageRows*otherCloud->imageCols/(_matcher->scale()*_matcher->scale());
   
    Eigen::Isometry3d keyOffset_, otherOffset_;
    Eigen::Matrix3d   otherCameraMatrix_;
    {
      PinholeImageData* imdata = keyNode->sensorData()->sensorData<PinholeImageData>(_cache->topic());
      if (! imdata) {
	throw std::runtime_error("the required topic does not match the requested type");
      }
      keyOffset_ = _robotConfiguration->sensorOffset(imdata->sensor());
    }
    {
      PinholeImageData* imdata = otherNode->sensorData()->sensorData<PinholeImageData>(_cache->topic());
      if (! imdata) {
	throw std::runtime_error("the required topic does not match the requested type");
      }
      otherOffset_ = _robotConfiguration->sensorOffset(imdata->sensor());
      otherCameraMatrix_ = imdata->cameraMatrix();
    }

    // convert double to float to call the matcher
    Eigen::Isometry3f keyOffset, otherOffset;
    Eigen::Matrix3f otherCameraMatrix;
    convertScalar(keyOffset, keyOffset_);
    convertScalar(otherOffset, otherOffset_);
    convertScalar(otherCameraMatrix, otherCameraMatrix_);
    otherCameraMatrix(2,2) = 1;

    Eigen::Isometry3d ig=initialGuess_;
    double nt = ig.translation().norm();
    double clamp = .5;
    if (nt>clamp)
      ig.translation()*=(clamp/nt);

    _matcher->clearPriors();
    if (keyNode->imu() && otherNode->imu()){
      MapNodeUnaryRelation* imuData=otherNode->imu();
      Matrix6d info = imuData->informationMatrix();
      _matcher->addAbsolutePrior(keyNode->transform(), imuData->transform(), info);
    }

    PwnMatcherBase::MatcherResult result;
    _matcher->matchClouds(result, 
			  keyCloud, otherCloud, 
			  keyOffset, otherOffset,
			  otherCameraMatrix, otherCloud->imageRows, otherCloud->imageCols, 
			  ig);

    if(result.image_nonZeros < _frameMinNonZeroThreshold ||
       result.image_outliers > _frameMaxOutliersThreshold || 
       result.image_inliers  < _frameMinInliersThreshold) {
      //cerr << "nz: " << result.image_nonZeros << endl;
      //cerr << "out: " << result.image_outliers << endl;
      //cerr << "inl: " << result.image_inliers << endl;
      return 0;
    }

    PwnCloserRelation* r=new PwnCloserRelation(_manager);
    r->nodes()[0]=keyNode;
    r->nodes()[1]=otherNode;
    r->fromResult(result);
    return r;
  }


  BOSS_REGISTER_CLASS(PwnCloserRelation);
  BOSS_REGISTER_CLASS(PwnCloser);
}


