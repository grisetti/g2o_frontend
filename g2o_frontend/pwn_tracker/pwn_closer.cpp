#include "pwn_closer.h"

namespace pwn_tracker {
  PwnCloserRelation::PwnCloserRelation(MapManager* manager, int id, IdContext* context):
    PwnTrackerRelation(manager, id, context){
    accepted = false;

    consensusCumInlier = 0;
    consensusCumOutlierTimes = 0;
    consensusTimeChecked = 0;

    reprojectionInliers = 0;
    reprojectionOutliers = 0;
    normalDifference = 0;
    depthDifference = 0;
    reprojectionDistance = 0;
    nonZeros = 0;
    outliers = 0;
    inliers = 0;
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
    _cache = new PwnCache(_converter, _scale, 100);
    _pendingTrackerFrame = 0;
    _lastTrackerFrame = 0;
    _frameInlierDepthThreshold = 50;
    _frameMinNonZeroThreshold = 3000;
    _frameMaxOutliersThreshold = 100;
    _frameMinInliersThreshold = 1000;
    _consensusInlierTranslationalThreshold = 0.5*0.5;
    _consensusInlierRotationalThreshold = 15.0f*M_PI/180.0f;
    _consensusMinTimesCheckedThreshold = 5;
    _debug = false;
  }

  
  class MyRelationSelector: public MapRelationSelector {
  public:
    MyRelationSelector(boss_map::MapManager* manager): MapRelationSelector(manager){}
    virtual bool accept(MapNodeRelation* r) {
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
  };

  void PwnCloser::addFrame(PwnTrackerFrame* f) {
    _trackerFrames.insert(make_pair(f->seq,f));
    _cache->addEntry(f);
    _lastTrackerFrame = _pendingTrackerFrame;
    _pendingTrackerFrame = f;
    
  }
  
  void PwnCloser::addRelation(PwnTrackerRelation* r){
    _trackerRelations.push_back(r);
    Eigen::Isometry3d dt;
    bool doSomething = false;
    if ( r->nodes()[0]==_lastTrackerFrame && r->nodes()[1]==_pendingTrackerFrame) {
      dt=r->transform();
      doSomething = true;
    } 
    if ( r->nodes()[1]==_lastTrackerFrame && r->nodes()[0]==_pendingTrackerFrame) {
      dt=r->transform().inverse();
      doSomething = true;
    }
    
    if (doSomething) {
      _pendingTrackerFrame->setTransform(_lastTrackerFrame->transform()*dt);
      process();
    }

  }

  void PwnCloser::process(){
    _committedRelations = 0;
    if (! _criterion)
      return;
    if (!_pendingTrackerFrame)
      return;
    std::set<MapNode*> selectedNodes;
    _criterion->setReferencePose(_pendingTrackerFrame->transform());
    selectNodes(selectedNodes,_criterion);
    std::vector< std::set<MapNode*> > partitions;
    MyRelationSelector selector(_manager);
    makePartitions(partitions, selectedNodes, &selector);
    cerr << "node: " << _pendingTrackerFrame->seq 
	 << ", neighbors: " << selectedNodes.size() 
	 << "partitions: " << partitions.size() << endl;

    std::set<MapNode*>* currentPartition = 0;
    for (size_t i=0; i<partitions.size(); i++){
      if (partitions[i].count(_pendingTrackerFrame)){
	currentPartition=&(partitions[i]); 
	break;
      }
    }
    if (! currentPartition) {
      throw std::runtime_error("no current partition");
    }
    for (size_t i=0; i<partitions.size(); i++){
      std::set<MapNode*>* otherPartition = &partitions[i];
      if (currentPartition == otherPartition)
	continue;
      cerr << "  " << i << "(" << otherPartition->size() << ")" << endl;
      processPartition(*otherPartition, _pendingTrackerFrame);
      validatePartitions(*otherPartition, *currentPartition);
    }
  }
  

  void PwnCloser::processPartition(std::set<MapNode*>& otherPartition, MapNode* current_){
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
    pwn::Frame* f=_cache->get(current);
    _cache->lock(current);
    cerr << "FRAME: " << current->seq << endl; 
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
	pwn::Frame* f2=_cache->get(other);
	
	Eigen::Isometry3d ig=iT*other->transform();
	PwnCloserRelation* rel = matchFrames(current, other, f, f2, ig);
	cerr << "  framesMatched: " << rel << " dc:"  << dc << " nc:" << nc << endl;
	if (rel) {
	  _results.push_back(rel);
	  _manager->addRelation(rel);
      	}
      }
      delete otherDepthThumbnailBLOB;
      delete otherNormalThumbnailBLOB;
    }
    _cache->unlock(current);
    cerr << "done" << endl;

    //delete currentDepthBLOB;
    delete currentDepthThumbnailBLOB;
    delete currentNormalThumbnailBLOB;
  }


  void validateRelation(float* translationalErrors,
			float* rotationalErrors ,
			std::vector<PwnCloserRelation*>& relations, 
			PwnTrackerRelation* r, 
			std::set<MapNode*>& current){
    Eigen::Isometry3d tc, to, tr;
    if (current.count(r->nodes()[0])) {
      tc=r->nodes()[0]->transform();
      to=r->nodes()[1]->transform();
      tr=r->transform(); 
    }
    else if (current.count(r->nodes()[1])) {
      to=r->nodes()[0]->transform();
      tc=r->nodes()[1]->transform();
      tr=r->transform().inverse();
    } else {
	throw std::runtime_error("node in current partition missing");
    }
 

    Eigen::Isometry3d tcInv = tc.inverse();
    Eigen::Isometry3d tfix = to*tr*tcInv;
    for (size_t i = 0; i<relations.size(); i++){
      PwnCloserRelation* r=relations[i];
      PwnTrackerFrame* f0=dynamic_cast<PwnTrackerFrame*>(r->nodes()[0]);
      PwnTrackerFrame* f1=dynamic_cast<PwnTrackerFrame*>(r->nodes()[1]);
      Eigen::Isometry3d tc, to, tr;
      if (current.count(f0)) {
	tc=f0->transform();
	to=f1->transform();
	tr=r->transform(); 
      } else if (current.count(f1)) {
	tc=f1->transform();
	to=f0->transform();
	tr=r->transform().inverse();
      } else{
 	throw std::runtime_error("node in current partition missing");
      }
      // compute the position of the node in current
      Eigen::Isometry3d tcp=tfix*tc;
      // compute the predicted position of the node
      Eigen::Isometry3d trp=to.inverse()*tcp;
      
      // compute the error
      Eigen::Isometry3d te=tr.inverse()*trp;
      
      // extract rotational and translational part
      float transErr=te.translation().squaredNorm();
      Eigen::AngleAxisd aa(te.linear());
      float rotErr=aa.angle();
      translationalErrors[i]=transErr;
      rotationalErrors[i]=rotErr;
    }
  }


  void PwnCloser::validatePartitions(std::set<MapNode*>& other, 
				     std::set<MapNode*>& current) {
    // scan for the pwn closure relations connecting a node in current and a node in others
    std::vector<PwnCloserRelation*> rels;
    for (std::set<MapNode*>::iterator it=other.begin(); it!=other.end(); it++){
      PwnTrackerFrame* n=dynamic_cast<PwnTrackerFrame*>(*it);
      if (!n)
	continue;
      std::set<MapNodeRelation*>& nrel=_manager->nodeRelations(n);
      for (std::set<MapNodeRelation*>::iterator rit= nrel.begin(); rit!=nrel.end(); rit++){
	PwnCloserRelation* r=dynamic_cast<PwnCloserRelation*>(*rit);
	if (! r)
	  continue;
	for (size_t i = 0; i<r->nodes().size(); i++){
	  if (current.count(r->nodes()[i])){
	    rels.push_back(r);
	    break;
	  }
	}
      }
    }
    if (rels.size()){
      cerr << "Checking " << rels.size() << " relations" << endl;
      cerr << "current: ";
      for (std::set<MapNode*>::iterator it=current.begin(); it!=current.end(); it++){
	PwnTrackerFrame* n=(PwnTrackerFrame*)(*it);
	cerr << n->seq << " ";
      }
      cerr<< endl;
      cerr << "other: ";
      for (std::set<MapNode*>::iterator it=other.begin(); it!=other.end(); it++){
	PwnTrackerFrame* n=(PwnTrackerFrame*)(*it);
	cerr << n->seq << " ";
      }
      cerr<< endl;

      Eigen::MatrixXf translationalErrors(rels.size(), rels.size());
      Eigen::MatrixXf rotationalErrors(rels.size(), rels.size());

      for (size_t i=0; i<rels.size(); i++){
      	validateRelation(translationalErrors.col(i).data(),
		      rotationalErrors.col(i).data() ,
		      rels, 
		      rels[i], 
		      current);
	rels[i]->consensusTimeChecked++;
      }
          
      // now get the matrix of consensus.
      // for each relation, count its consensus, add it to all inliers
      std::vector<bool> relInliers(rels.size());
      //for (size_t i=0; i<rels.size(); i++){
      // 	rels[i]->outlierCount=0;
      // 	rels[i]->inlierCount=0;
      // }
      for (size_t i=0; i<rels.size(); i++){
	//PwnCloserRelation* rOut=rels[i];
	int inliersCount=0;
	for (size_t j=0; j<rels.size(); j++){
	  float te=translationalErrors(j,i);
	  float re=fabs(rotationalErrors(j,i));
	  bool isIn=(te<_consensusInlierTranslationalThreshold) && (re<_consensusInlierRotationalThreshold);
	  relInliers[j]=isIn;
	  inliersCount+=isIn;
	}
	if (! inliersCount ) {
	  cerr << "te: " << endl;
	  cerr << translationalErrors << endl;
	  cerr << "re: " << endl;
	  cerr << rotationalErrors << endl;
	  throw std::runtime_error("no inliers");
        }
	for (size_t j=0; j<rels.size(); j++){
	  if (relInliers[j]){
	    rels[j]->consensusCumInlier+=inliersCount;
	  } else
	    rels[j]->consensusCumOutlierTimes+=1;
	}
      }

      for (size_t i=0; i<rels.size(); i++){
	PwnCloserRelation* r=rels[i];
	PwnTrackerFrame* n1=(PwnTrackerFrame*)(r->nodes()[0]);
	PwnTrackerFrame* n2=(PwnTrackerFrame*)(r->nodes()[1]);
	cerr << "r" << r << "(" 
	     << n1->seq << "," << n2->seq << "): nChecks= " << r->consensusTimeChecked << " inliers="
	     << r->consensusCumInlier << " outliers=" << r->consensusCumOutlierTimes;
	if(r->consensusTimeChecked<_consensusMinTimesCheckedThreshold) {
	  cerr << "skip" << endl;
	  continue;
	} 
	if (r->consensusCumInlier>r->consensusCumOutlierTimes){
	  r->accepted = true;
	  cerr << "accept" << endl;
	  _committedRelations++;
	} else {
	  _manager->removeRelation(r);
	  cerr << "delete" << endl;
	}
      }
      cerr << "done" << endl;
    }
  }



  PwnCloserRelation* PwnCloser::matchFrames(PwnTrackerFrame* from, PwnTrackerFrame* to, 
				 pwn::Frame* fromCloud, pwn::Frame* toCloud,
				 const Eigen::Isometry3d& initialGuess){
    if (from == to)
      return 0; 
    
    cerr <<  "  matching  frames: " << from->seq << " " << to->seq << endl;
    
  
    Eigen::Isometry3f fromOffset, toOffset;
    Eigen::Matrix3f fromCameraMatrix, toCameraMatrix;

    convertScalar(fromOffset, from->sensorOffset);
    convertScalar(fromCameraMatrix, from->cameraMatrix);

    convertScalar(toOffset, to->sensorOffset);
    convertScalar(toCameraMatrix, to->cameraMatrix);
    
    PinholePointProjector* projector = (PinholePointProjector*)_aligner->projector();
    int r, c;
    
    PwnCloserRelation* rel = new PwnCloserRelation(_manager);
    _aligner->setReferenceSensorOffset(fromOffset);
    _aligner->setCurrentSensorOffset(toOffset);
    Eigen::Isometry3f ig;
    convertScalar(ig, initialGuess);
    ig.translation().z() = 0;
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
    rel->setFrom(from);
    rel->setTo(to);
    Eigen::Isometry3d relationMean;
    convertScalar(relationMean, _aligner->T());
    rel->setTransform(relationMean);
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
    currentDepthThumb.toCvMat(currentRect);
    referenceDepthThumb.toCvMat(referenceRect);
    cv::Mat mask = (currentRect>0) & (referenceRect>0);
    currentRect.convertTo(currentRect, CV_32FC1);
    referenceRect.convertTo(referenceRect, CV_32FC1);
    mask.convertTo(mask, currentRect.type());
    cv::Mat diff = abs(currentRect-referenceRect)&mask;
    int nonZeros = countNonZero(mask);
    float sum=0;
    int inliers = 0;
    int outliers = 0;
    for (int i = 0; i<diff.rows; i++)
      for (int j = 0; j<diff.cols; j++){
	float d = diff.at<float>(i,j);
	if (mask.at<float>(i,j) && d < _frameInlierDepthThreshold)
	  inliers ++;
	sum +=d;
      }
    int imSize = diff.rows*diff.cols;
    rel->reprojectionDistance = sum/nonZeros;
    rel->nonZeros = nonZeros;
    if (_debug) 
      rel->diffRegistered = diff.clone();

    cerr << "  transform            : " << t2v(_aligner->T()).transpose() << endl;
    cerr << "  imsize               : " << imSize << endl;
    cerr << "  inliers              : " << _aligner->inliers()<< endl;
    cerr << "  reprojectionDistance : " << sum/nonZeros << endl;
    cerr << "  nonZeros             : " << nonZeros << endl;
    rel->outliers = nonZeros-inliers;
    rel->inliers = inliers;
    rel->reprojectionInliers=inliers;
    rel->reprojectionOutliers=outliers;
  }
  
}
