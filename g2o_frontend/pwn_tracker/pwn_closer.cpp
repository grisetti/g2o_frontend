#include "pwn_closer.h"

namespace pwn_tracker {
  PwnCloserRelation::PwnCloserRelation(MapManager* manager, int id, IdContext* context):
    PwnTrackerRelation(manager, id, context){
    reprojectionInliers = 0;
    reprojectionOutliers = 0;
  }

  void PwnCloserRelation::serialize(ObjectData& data, IdContext& context){
    PwnTrackerRelation::serialize(data,context);
    data.setInt("reprojectionInliers", reprojectionInliers);
    data.setInt("reprojectionOutliers", reprojectionOutliers);
  }

  void PwnCloserRelation::deserialize(ObjectData& data, IdContext& context){
    PwnTrackerRelation::deserialize(data,context);
    reprojectionInliers = data.getInt("reprojectionInliers");
    reprojectionOutliers = data.getInt("reprojectionOutliers");
  }


  PwnCloser::PwnCloser(pwn::Aligner* aligner_, 
		       pwn::DepthImageConverter* converter_,
		       MapManager* manager_) {
    _aligner = aligner_;
    _converter = converter_;
    _manager = manager_;
    _scale = 4;
    _cache = new PwnCache(_converter, _scale, 50);
    _criterion.setRotationalDistance(M_PI/8);
    _criterion.setTranslationalDistance(.5);
    _criterion.setManager(_manager);
    _lastTrackerFrame = 0;
  }

  

  void PwnCloser::addFrame(PwnTrackerFrame* f) {
    _trackerFrames.insert(make_pair(f->seq,f));
    _cache->addEntry(f);
    _lastTrackerFrame = f;
  }
  
  void PwnCloser::addRelation(PwnTrackerRelation* r){
    _trackerRelations.push_back(r);
    if(r->nodes()[0]==_lastTrackerFrame || r->nodes()[1] == _lastTrackerFrame)
      process();
  }

  void PwnCloser::process(){
    if (!_lastTrackerFrame)
      return;
    std::set<MapNode*> selectedNodes;
    _criterion.setReferencePose(_lastTrackerFrame->transform());
    selectNodes(selectedNodes,&_criterion);
    std::vector< std::set<MapNode*> > partitions;
    makePartitions(partitions, selectedNodes);
    cerr << "node: " << _lastTrackerFrame->seq 
	 << ", neighbors: " << selectedNodes.size() 
	 << "partitions: " << partitions.size() << endl;
    for (size_t i=0; i<partitions.size(); i++){
      cerr << "  " << i << "(" << partitions[i].size() << ")" << endl;
      processPartition(partitions[i], _lastTrackerFrame);
    }
  }
  

  void PwnCloser::processPartition(std::set<MapNode*> nodes, MapNode* current_){
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
    pwn::Frame* f=_cache->get(current);
    _cache->lock(current);
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
      
      pwn::Frame* f2=_cache->get(other);
    
      Eigen::Isometry3d ig=iT*other->transform();
      MatchingResult result;
      bool framesMatched = matchFrames(result, current, other, f, f2, ig);
      if (framesMatched)
	_results.push_back(result);
      //delete otherDepthThumbnailBLOB;
      //delete otherNormalThumbnailBLOB;
    }
    _cache->unlock(current);
    cerr << "done" << endl;

    //delete currentDepthBLOB;
    //delete currentDepthThumbnailBLOB;
    //delete currentNormalThumbnailBLOB;
  }


  bool PwnCloser::matchFrames(MatchingResult& result,
			      PwnTrackerFrame* from, PwnTrackerFrame* to, 
			      pwn::Frame* fromCloud, pwn::Frame* toCloud,
			      const Eigen::Isometry3d& initialGuess){
    if (from == to)
      return false; 
    MapManager* manager = from->manager();
   
    cerr <<  "  matching  frames: " << from->seq << " " << to->seq << endl;
    
  
    Eigen::Isometry3f fromOffset, toOffset;
    Eigen::Matrix3f fromCameraMatrix, toCameraMatrix;

    convertScalar(fromOffset, from->sensorOffset);
    convertScalar(fromCameraMatrix, from->cameraMatrix);

    convertScalar(toOffset, to->sensorOffset);
    convertScalar(toCameraMatrix, to->cameraMatrix);
    
    PinholePointProjector* projector = (PinholePointProjector*)_aligner->projector();
    int r, c;
  
    _aligner->setReferenceSensorOffset(fromOffset);
    _aligner->setCurrentSensorOffset(toOffset);
    Eigen::Isometry3f ig;
    convertScalar(ig, initialGuess);
    _aligner->setInitialGuess(ig);
    projector->setCameraMatrix(toCameraMatrix);
    projector->setImageSize(to->imageRows,to->imageCols);
    projector->scale(1./_scale);
    
    // cerr << "cameraMatrix: " << endl;
    // cerr << projector->cameraMatrix() << endl;
    r = projector->imageRows();
    c = projector->imageCols();
  
    _aligner->correspondenceFinder()->setImageSize(r,c);
    _aligner->setReferenceFrame(fromCloud);
    _aligner->setCurrentFrame(toCloud);
    _aligner->align();
    Matrix6d omega;
    convertScalar(omega, _aligner->omega());
  
    
    DepthImage 
      currentDepthThumb = _aligner->correspondenceFinder()->currentDepthImage(),
      referenceDepthThumb = _aligner->correspondenceFinder()->referenceDepthImage();
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
	float d = diff.at<float>(i,j);
	sum +=d;
      }
    result.reprojectionDistance = sum/nonZeros;
    result.nonZeros = nonZeros;
    result.from = from;
    result.to = to;
    cerr << "  initialGuess         : " << t2v(ig).transpose() << endl;
    cerr << "  transform            : " << t2v(_aligner->T()).transpose() << endl;
    cerr << "  inliers              : " << _aligner->inliers() <<  "/" << r*c << endl;
    cerr << "  reprojectionDistance : " << sum/nonZeros << endl;
    cerr << "  nonZeros/total        : " << nonZeros << "/" << r*c  << endl;
    
    Eigen::Isometry3d relationMean;
    convertScalar(relationMean, _aligner->T());
    PwnCloserRelation* rel = new PwnCloserRelation(manager);
    rel->setTransform(relationMean);
    rel->setInformationMatrix(omega);
    rel->setTo(from);
    rel->setFrom(to);
    result.relation = rel;
    return true;
  }

  void PwnCloser::updateCache(){
    _cache->setConverter(_converter);
    _cache->setScale(_scale);
  }

  float  PwnCloser::compareNormals(cv::Mat& m1, cv::Mat& m2){
    if (m1.rows!=m2.rows || m1.cols != m2.cols)
      return 0;
    return norm(abs(m1-m2));
  }
  
  float PwnCloser::compareDepths(cv::Mat& m1, cv::Mat& m2){
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
  
}
