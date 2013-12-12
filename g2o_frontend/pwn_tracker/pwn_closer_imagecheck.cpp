#include "pwn_closer.h"
#include "g2o_frontend/pwn_core/pwn_static.h"

namespace pwn_tracker {

  PwnCloserRelation::PwnCloserRelation(MapManager* manager, int id, IdContext* context):
    PwnTrackerRelation(manager, id, context){

    normalDifference = 0;
    depthDifference = 0;
    reprojectionDistance = 0;
    nonZeros = 0;
    outliers = 0;
    inliers = 0;
  }

  void PwnCloserRelation::serialize(ObjectData& data, IdContext& context){
    PwnTrackerRelation::serialize(data,context);
    data.setFloat("normalDifference", normalDifference);
    data.setFloat("depthDifference", depthDifference);
    data.setFloat("reprojectionDistance", reprojectionDistance);
    data.setInt("nonZeros", nonZeros);
    data.setInt("outliers", outliers);
    data.setInt("inliers", inliers);

    data.setBool("accepted", accepted);
    data.setInt("consensusCumInlier", consensusCumInlier);
    data.setInt("consensusCumOutlierTimes", consensusCumOutlierTimes);
    data.setInt("consensusTimeChecked", consensusTimeChecked);
  }

  void PwnCloserRelation::deserialize(ObjectData& data, IdContext& context){
    PwnTrackerRelation::deserialize(data,context);
    normalDifference = data.getFloat("normalDifference");
    depthDifference = data.getFloat("depthDifference");
    reprojectionDistance = data.getFloat("reprojectionDistance");
    nonZeros = data.getInt("nonZeros");
    outliers = data.getInt("outliers");
    inliers  = data.getInt("inliers");

    accepted = data.getBool("accepted");
    consensusCumInlier=data.getInt("consensusCumInlier");
    consensusCumOutlierTimes = data.getInt("consensusCumOutlierTimes");
    consensusTimeChecked = data.getInt("consensusTimeChecked");
  }


  PwnCloser::PwnCloser(pwn::Aligner* aligner_, 
		       pwn::DepthImageConverter* converter_,
		       MapManager* manager_,
		       PwnCache* cache_) : MapCloser(manager_){
    _aligner = aligner_;
    _converter = converter_;
    _cache = cache_;
    _frameInlierDepthThreshold = 50;
    _frameMinNonZeroThreshold = 3000;// was 3000
    _frameMaxOutliersThreshold = 100;
    _frameMinInliersThreshold = 1000; // was 1000
    _debug = false;
    _selector = new PwnCloserActiveRelationSelector(_manager);
    setScale(4);
  }


  PwnCloserActiveRelationSelector::PwnCloserActiveRelationSelector(boss_map::MapManager* manager): MapRelationSelector(manager){}

  bool PwnCloserActiveRelationSelector::accept(MapNodeRelation* r) {
    if (!r)
      return false;
    {
      PwnCloserRelation* _r = dynamic_cast<PwnCloserRelation*>(r);
      if (_r){
	return _r->accepted;
      }
    }
    return dynamic_cast<PwnTrackerRelation*>(r);
  }
  


  void PwnCloser::processPartition(std::list<MapNodeBinaryRelation*>& newRelations, 
				   std::set<MapNode*>& otherPartition, 
				   MapNode* current_){
    PwnTrackerFrame* current = dynamic_cast<PwnTrackerFrame*>(current_);
    if (otherPartition.count(current)>0)
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
    PwnCache::HandleType f_handle=_cache->get(current);
    pwn::Frame* f=f_handle.get();
    //cerr << "FRAME: " << current->seq << endl; 
    for (std::set <MapNode*>::iterator it=otherPartition.begin(); it!=otherPartition.end(); it++){
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
      otherDepthThumbnailBLOB->cvImage().convertTo(otherDepthThumbnail, CV_32FC1);
      otherDepthThumbnail=otherDepthThumbnail-127.0f;
      otherDepthThumbnail=otherDepthThumbnail*(1./255);

      float dc = compareNormals(currentNormalThumbnail, otherNormalThumbnail);
      float nc = compareDepths(currentDepthThumbnail, otherDepthThumbnail);
      
      float _normalTuhmbnailThreshold = 1e3;
      if (nc<_normalTuhmbnailThreshold) {
	PwnCache::HandleType f2_handle=_cache->get(other);
	pwn::Frame* f2=f2_handle.get();
	
	Eigen::Isometry3d ig=iT*other->transform();
	PwnCloserRelation* rel = matchFrames(current, other, f, f2, ig);
	//cerr << "  framesMatched: " << rel << " dc:"  << dc << " nc:" << nc << endl;
	if (rel) {
	  rel->depthDifference = dc;
	  rel->normalDifference = nc;
	  cerr << "o";
	  // _results.push_back(rel);
	  // _manager->addRelation(rel);
	  newRelations.push_back(rel);
      	} else 
	  cerr << ".";
      }
      delete otherDepthThumbnailBLOB;
      delete otherNormalThumbnailBLOB;
    }
    cerr << endl;
    //delete currentDepthBLOB;
    delete currentDepthThumbnailBLOB;
    delete currentNormalThumbnailBLOB;
  }


  PwnCloserRelation* PwnCloser::matchFrames(PwnTrackerFrame* from, PwnTrackerFrame* to, 
				 pwn::Frame* fromCloud, pwn::Frame* toCloud,
				 const Eigen::Isometry3d& initialGuess){
    if (from == to)
      return 0; 
    
    //cerr <<  "  matching  frames: " << from->seq << " " << to->seq << endl;
    
  
    Eigen::Isometry3f fromOffset, toOffset;
    Eigen::Matrix3f fromCameraMatrix, toCameraMatrix;

    convertScalar(fromOffset, from->sensorOffset);
    convertScalar(fromCameraMatrix, from->cameraMatrix);
    convertScalar(toOffset, to->sensorOffset);
    convertScalar(toCameraMatrix, to->cameraMatrix);
    
    PinholePointProjector* projector = dynamic_cast<PinholePointProjector*>(_aligner->projector());
    int r, c;
    
    PwnCloserRelation* rel = new PwnCloserRelation(_manager);
    _aligner->setReferenceSensorOffset(fromOffset);
    _aligner->setCurrentSensorOffset(toOffset);
    Eigen::Isometry3f ig;
    convertScalar(ig, initialGuess);
    ig.translation().z() = 0;
    _aligner->setInitialGuess(ig);
    //cerr << "initialGuess: " << t2v(ig).transpose() << endl;
    projector->setCameraMatrix(toCameraMatrix);
    projector->setImageSize(to->imageRows,to->imageCols);
    projector->scale(1./_scale);
    
    // cerr << "cameraMatrix: " << endl;
    // cerr << projector->cameraMatrix() << endl;
    r = projector->imageRows();
    c = projector->imageCols();
    // char dbgName[1024];
    // sprintf(dbgName, "match-%06d-%06d",from->seq, to->seq);
    // _aligner->debugPrefix()=dbgName;
    _aligner->correspondenceFinder()->setImageSize(r,c);
    _aligner->setReferenceFrame(fromCloud);
    _aligner->setCurrentFrame(toCloud);
    _aligner->align();
    //_aligner->debugPrefix()=""; FICSMI

    // cerr << "_fromCloud.points():" << fromCloud->points().size() << endl;
    // cerr << "_toCloud.points():" << toCloud->points().size() << endl;
    // cerr << "AlInliers: " << _aligner->inliers() << endl;
    rel->setFrom(from);
    rel->setTo(to);
    Eigen::Isometry3d relationMean;
    convertScalar(relationMean, _aligner->T());
    rel->setTransform(relationMean);

    Matrix6d omega;
    convertScalar(omega, _aligner->omega());
    omega.setIdentity(); //HACK
    omega*=100;
    rel->setInformationMatrix(omega);
    rel->setTo(to);
    rel->setFrom(from);
    scoreMatch(rel);
    if ((rel->nonZeros<_frameMinNonZeroThreshold) || 
	(rel->outliers>_frameMaxOutliersThreshold) || 
	(rel->inliers<_frameMinInliersThreshold)) {
      delete rel;
      return 0;
    }
    return rel;
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

  void PwnCloser::scoreMatch(PwnCloserRelation* rel){
    DepthImage 
      currentDepthThumb = _aligner->correspondenceFinder()->currentDepthImage(),
      referenceDepthThumb = _aligner->correspondenceFinder()->referenceDepthImage();
    cv::Mat currentRect, referenceRect;
    DepthImage_convert_32FC1_to_16UC1(currentRect,currentDepthThumb); 
    //currentDepthThumb.toCvMat(currentRect);
    
    DepthImage_convert_32FC1_to_16UC1(referenceRect,referenceDepthThumb); 
    //referenceDepthThumb.toCvMat(referenceRect);

    cv::Mat mask = (currentRect>0) & (referenceRect>0);
    currentRect.convertTo(currentRect, CV_32FC1);
    referenceRect.convertTo(referenceRect, CV_32FC1);
    mask.convertTo(mask, currentRect.type());
    cv::Mat diff = abs(currentRect-referenceRect)&mask;
    int nonZeros = countNonZero(mask);
    float sum=0;
    int inliers = 0;
    for (int i = 0; i<diff.rows; i++)
      for (int j = 0; j<diff.cols; j++){
	float d = diff.at<float>(i,j);
	if (mask.at<float>(i,j) && d < _frameInlierDepthThreshold)
	  inliers ++;
	sum +=d;
      }
    //int imSize = diff.rows*diff.cols;
    rel->reprojectionDistance = sum/nonZeros;
    rel->nonZeros = nonZeros;
    if ( 0 && _debug) 
      rel->diffRegistered = diff.clone();

    rel->outliers = nonZeros-inliers;
    rel->inliers = inliers;
    
    if (_debug) {
      cerr << "  transform            : " << t2v(rel->transform()).transpose() << endl;
      cerr << "  inliers              : " << rel->inliers<< endl;
      cerr << "  reprojectionDistance : " << rel->reprojectionDistance << endl;
      cerr << "  nonZeros             : " << rel->nonZeros << endl;
    }
  }

// closure actions

  NewFrameCloserAdder::NewFrameCloserAdder(PwnCloser* closer, PwnTracker* tracker):
    PwnTracker::NewFrameAction(tracker){
    _closer = closer;
  }
  void NewFrameCloserAdder::compute (PwnTrackerFrame* frame) {
    _closer->addFrame(frame);
  }


  CloserRelationAdder::CloserRelationAdder(std::list<Serializable*>& objects_,
		      PwnCloser* closer, 
		      G2oWrapper* optimizer_, 
		      PwnTracker* tracker):
    PwnTracker::NewRelationAction(tracker),
    _objects(objects_) {
    _closer = closer;
    _optimizer = optimizer_;
  }

  void CloserRelationAdder::compute (PwnTrackerRelation* relation) {
    _closer->addRelation(relation);
    cerr << "CLOSER PARTITIONS: " << _closer->partitions().size() << endl;
    int cr=0;
    for(std::list<MapNodeBinaryRelation*>::iterator it=_closer->committedRelations().begin();
	it!=_closer->committedRelations().end(); it++){
      _objects.push_back(*it);
      cr++;
    }
    if (cr){
      cerr << "COMMITTED RELATIONS: " << cr << endl;
      _optimizer->optimize();
      // char fname[100];
      // sprintf(fname, "out-%05d.g2o", lastFrameAdded->seq);
      // optimizer->save(fname);
    }
  }

  BOSS_REGISTER_CLASS(PwnCloserRelation);
}
