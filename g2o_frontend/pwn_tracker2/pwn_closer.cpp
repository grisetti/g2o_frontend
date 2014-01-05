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


  PwnCloser::PwnCloser(PwnTracker* tracker_, int id, IdContext* context) : 
    MapCloser(0,id, context){
    if (tracker_)
      setTracker(tracker_);
    _tracker=tracker_;
    _frameMinNonZeroThreshold = 3000;// was 3000
    _frameMaxOutliersThreshold = 100;
    _frameMinInliersThreshold = 1000; // was 1000
    _debug = false;
    _selector = 0;
    _enabled = true;
  }

  void PwnCloser::serialize(boss::ObjectData& data, boss::IdContext& context){
    boss::Identifiable::serialize(data,context);
    data.setFloat("consensusInlierTranslationalThreshold", _consensusInlierTranslationalThreshold);
    data.setFloat("consensusInlierRotationalThreshold", _consensusInlierRotationalThreshold);
    data.setInt("consensusMinTimesCheckedThreshold", _consensusMinTimesCheckedThreshold);
    data.setPointer("tracker", _tracker);
    data.setInt("frameMinNonZeroThreshold", _frameMinNonZeroThreshold);
    data.setInt("frameMaxOutliersThreshold", _frameMaxOutliersThreshold);
    data.setInt("frameMinInliersThreshold", _frameMinInliersThreshold);
    data.setInt("imageRows", _imageRows);
    data.setInt("imageCols", _imageCols);
  }
    
  
  void PwnCloser::deserialize(boss::ObjectData& data, boss::IdContext& context){
    boss::Identifiable::deserialize(data,context);
    _consensusInlierTranslationalThreshold = data.getFloat("consensusInlierTranslationalThreshold");
    _consensusInlierRotationalThreshold = data.getFloat("consensusInlierRotationalThreshold");
    _consensusMinTimesCheckedThreshold = data.getInt("consensusMinTimesCheckedThreshold");
    data.getReference("tracker").bind(_tracker);
    _frameMinNonZeroThreshold = data.getInt("frameMinNonZeroThreshold");
    _frameMaxOutliersThreshold = data.getInt("frameMaxOutliersThreshold");
    _frameMinInliersThreshold = data.getInt("frameMinInliersThreshold");
    _imageRows = data.getInt("imageRows");
    _imageCols = data.getInt("imageCols");
   }


  void PwnCloser::deserializeComplete(){
    boss::Identifiable::deserializeComplete();
    if (_tracker)
      setTracker(_tracker);
 }

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
			  otherCameraMatrix, otherCloud->imageRows, otherCloud->imageCols, 
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


  int PwnCloser::scale() const { return _matcher->scale(); }

  void PwnCloser::setScale (int scale_) {
    if (_matcher)
      _matcher->setScale(scale_);
    if (_cache)
      _cache->setScale(scale_);
    _scaledImageCols = _imageCols/scale_;
    _scaledImageRows = _imageRows/scale_;
    _scaledImageSize = _scaledImageRows * _scaledImageCols;
  }

  void PwnCloser::setImageSize(int imageRows_, int imageCols_){
    _imageRows = imageRows_;
    _imageCols = imageCols_;
    int s = scale();
    _scaledImageCols = _imageCols/s;
    _scaledImageRows = _imageRows/s;
    _scaledImageSize = _scaledImageRows * _scaledImageCols;
  }

  BOSS_REGISTER_CLASS(PwnCloserRelation);
  BOSS_REGISTER_CLASS(PwnCloser);
}


