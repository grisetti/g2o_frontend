#include "pwn_tracker.h"
#include "g2o_frontend/boss_map/sensor_data_node.h"
namespace pwn_tracker{
  using namespace boss_map_building;
  using namespace boss_map;

  PwnTrackerRelation::PwnTrackerRelation(MapManager* manager, int id, IdContext* context) :
    MapNodeBinaryRelation(manager, id, context){
    cloud_inliers = 0;
    image_nonZeros = 0;
    image_outliers = 0;
    image_inliers = 0;
    image_reprojectionDistance = 0;
}
  
  void PwnTrackerRelation::serialize(ObjectData& data, IdContext& context){
    MapNodeBinaryRelation::serialize(data,context);
    data.setInt("cloud_inliers", cloud_inliers);
    data.setInt("image_inliers", image_inliers);
    data.setInt("image_nonZeros", image_nonZeros);
    data.setInt("image_outliers", image_outliers);
    data.setFloat("image_reprojectionDistance", image_reprojectionDistance);
  }
  
  void PwnTrackerRelation::deserialize(ObjectData& data, IdContext& context){
    MapNodeBinaryRelation::deserialize(data,context);
    cloud_inliers = data.getInt("cloud_inliers");
    image_nonZeros = data.getInt("image_nonZeros");
    image_outliers = data.getInt("image_outliers");
    image_inliers = data.getInt("image_inliers");
    image_reprojectionDistance = data.getFloat("image_reprojectionDistance");
  }

  void PwnTrackerRelation::fromResult(PwnMatcherBase::MatcherResult& result){
    cloud_inliers = result.cloud_inliers;
    image_nonZeros = result.image_nonZeros;
    image_outliers = result.image_outliers;
    image_inliers = result.image_inliers;
    image_reprojectionDistance = result.image_reprojectionDistance;
    setTransform(result.transform);
    setInformationMatrix(result.informationMatrix);
  }

  PwnTrackerRelation::~PwnTrackerRelation(){}
  
  PwnTracker::PwnTracker(PwnMatcherBase* matcher_,
			 PwnCloudCache* cache_,
			 MapManager* manager_,
			 RobotConfiguration* configuration_):
    BaseTracker(manager_, configuration_){
    _cache = cache_;
    _matcher = matcher_;
    setTopic("/camera/depth_registered/image_rect_raw");
    _imageRows = 480;
    _imageCols = 640;
    _imageSize = _imageRows * _imageCols;
    setScale(4);
    _newFrameCloudInliersFraction = 0.4;
    _minCloudInliers = 1000;
    _frameMinNonZeroThreshold = 3000;// was 3000
    _frameMaxOutliersThreshold = 2000;
    _frameMinInliersThreshold = 500; // was 1000
    _enabled = true;
  }

  PwnTracker::~PwnTracker(){}

  
  int PwnTracker::scale() const { return _matcher->scale(); }

  void PwnTracker::setScale (int scale_) {
    _matcher->setScale(scale_);
    _cache->setScale(scale_);
    _scaledImageCols = _imageCols/scale_;
    _scaledImageRows = _imageRows/scale_;
    _scaledImageSize = _scaledImageRows * _scaledImageCols;
  }

  void PwnTracker::setImageSize(int imageRows_, int imageCols_){
    _imageRows = imageRows_;
    _imageCols = imageCols_;
    int s = scale();
    _scaledImageCols = _imageCols/s;
    _scaledImageRows = _imageRows/s;
    _scaledImageSize = _scaledImageRows * _scaledImageCols;
  }

  bool PwnTracker::shouldChangeKeyNode(MapNodeBinaryRelation* r_){
    PwnTrackerRelation* r=dynamic_cast<PwnTrackerRelation*>(r_);
    if (r->cloud_inliers > _newFrameCloudInliersFraction*_scaledImageSize)
      return false;
    return true;
  }

  MapNodeBinaryRelation* PwnTracker::registerNodes(MapNode* keyNode_, MapNode* otherNode_, const Eigen::Isometry3d&  initialGuess_) {
    SyncSensorDataNode * keyNode = dynamic_cast<SyncSensorDataNode*>(keyNode_);
    SyncSensorDataNode * otherNode = dynamic_cast<SyncSensorDataNode*>(otherNode_);
    if (! (keyNode && otherNode))
      return 0;

    // fetch the clouds from the cache
    PwnCloudCache::HandleType _keyCloudHandler = _cache->get(keyNode);
    pwn::Cloud* keyCloud = _keyCloudHandler.get();
    PwnCloudCache::HandleType _otherCloudHandler = _cache->get(otherNode); 
    pwn::Cloud* otherCloud = _otherCloudHandler.get();

    Eigen::Isometry3d keyOffset_, otherOffset_;
    Eigen::Matrix3d   otherCameraMatrix_;
    {
      PinholeImageData* imdata = keyNode->sensorData()->sensorData<PinholeImageData>(_topic);
      if (! imdata) {
	throw std::runtime_error("the required topic does not match the requested type");
      }
      keyOffset_ = _robotConfiguration->sensorOffset(imdata->sensor());
    }
    {
      PinholeImageData* imdata = otherNode->sensorData()->sensorData<PinholeImageData>(_topic);
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

    if (! _enabled)
      return 0;

    PwnMatcherBase::MatcherResult result;
    _matcher->matchClouds(result, 
			  keyCloud, otherCloud, 
			  keyOffset, otherOffset,
			  otherCameraMatrix, _imageRows, _imageCols, 
			  initialGuess_);
    //cerr << " key:" << keyNode->seq() << " other: " << otherNode->seq();
    //cerr << " cloud inliers: " << result.cloud_inliers;
    //cerr << " image_inliers: " << result.image_inliers;
    //cerr << " guess: " << t2v(initialGuess_).transpose();

    //cerr << endl;



    if(result.cloud_inliers < _minCloudInliers ||
       result.image_nonZeros < _frameMinNonZeroThreshold ||
       result.image_inliers  < _frameMinInliersThreshold ||
       result.image_outliers > _frameMaxOutliersThreshold )
      return 0;

    PwnTrackerRelation* r=new PwnTrackerRelation(_manager);
    r->nodes()[0]=keyNode;
    r->nodes()[1]=otherNode;
    r->fromResult(result);
       return r;
  }
  
  BOSS_REGISTER_CLASS(PwnTrackerRelation);

}
