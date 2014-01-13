#include "map_closer.h"
#include "base_tracker.h"


namespace boss_map_building {
  using namespace std;

  ClosureInfo::ClosureInfo(){
    accepted = false;
    consensusCumInlier = 0;
    consensusCumOutlierTimes = 0;
    consensusTimeChecked = 0;
  }
  ClosureInfo::~ClosureInfo(){}

  void ClosureFoundMessage::serialize(ObjectData& data, IdContext&){
    ArrayData* adata = new ArrayData;
    for(std::list<MapNodeBinaryRelation*>::iterator it = closureRelations.begin(); 
	it!=closureRelations.end(); it++){
      adata->add(new PointerData(*it));
    }
    data.setField("closureRelations", adata);
  }

  void ClosureFoundMessage::deserialize(ObjectData& data, IdContext&){
    closureRelations.clear();
    ArrayData& adata = data.getField("closureRelations")->getArray();
    for (size_t i=0; i<adata.size(); i++){
      closureRelations.push_back(dynamic_cast<MapNodeBinaryRelation*>(adata[i].getPointer()));
    }
  }
  
  void ClosureScannedMessage::serialize(ObjectData& data, IdContext&){
    ArrayData* adata = new ArrayData;
    data.setInt("currentPartitionIndex", currentPartitionIndex);
    for (size_t i = 0; i < partitions.size(); i++){
      ArrayData* pdata = new ArrayData;
      for (std::set<MapNode*>::iterator it = partitions[i].begin(); it!=partitions[i].end(); it++)
	pdata->add(new PointerData(*it));
      adata->add(pdata);
    }
    data.setField("partitions", adata);
  }
  
  void ClosureScannedMessage::deserialize(ObjectData& data, IdContext&){
    partitions.clear();
    currentPartitionIndex = data.getInt("currentPartitionIndex");
    ArrayData& adata = data.getField("partitions")->getArray();
    for (int i=0; i<adata.size(); i++){
      std::set<MapNode*> pnodes;
      ArrayData& pdata = adata[i].getArray();
      for (size_t k=0; k<pdata.size(); k++)
	pnodes.insert(dynamic_cast<MapNode*>(pdata[k].getPointer()));
      partitions.push_back(pnodes);
    }
  }



  MapCloser::MapCloser(MapManager* manager_, int id, boss::IdContext* context):
    StreamProcessor(id,context){
    _manager = manager_;
    _consensusInlierTranslationalThreshold = 0.5*0.5;
    _consensusInlierRotationalThreshold = 15.0f*M_PI/180.0f;
    _consensusMinTimesCheckedThreshold = 5;
    _debug = false;
    _currentPartition = 0;
    _pendingTrackerFrame = 0;
    _lastTrackerFrame = 0;
    _criterion = 0;
    _selector = 0;
    autoProcess = true;
  }

  
  void MapCloser::serialize(boss::ObjectData& data, boss::IdContext& context){
    StreamProcessor::serialize(data, context);
    data.setPointer("manager", _manager);
    data.setPointer("poseAcceptanceCriterion", _criterion);
    data.setPointer("relationSelector", _selector);
    data.setFloat("consensusInlierTranslationalThreshold", _consensusInlierTranslationalThreshold);
    data.setFloat("consensusInlierRotationalThreshold", _consensusInlierRotationalThreshold);
    data.setInt("consensusMinTimesCheckedThreshold", _consensusMinTimesCheckedThreshold);
  }

  void MapCloser::deserialize(boss::ObjectData& data, boss::IdContext& context){
    StreamProcessor::deserialize(data,context);
    data.getReference("manager").bind(_manager);
    data.getReference("poseAcceptanceCriterion").bind(_criterion);
    data.getReference("relationSelector").bind(_selector);
    _consensusInlierTranslationalThreshold = data.getFloat("consensusInlierTranslationalThreshold");
    _consensusInlierRotationalThreshold = data.getFloat("consensusInlierRotationalThreshold");
    _consensusMinTimesCheckedThreshold = data.getInt("consensusMinTimesCheckedThreshold");
  }
  

  void MapCloser::setManager(MapManager* m){
    _manager = m;
    if (_criterion)
      _criterion->setManager(_manager);
    if (_selector)
      _selector->setManager(_manager);
  }
    
  void MapCloser::addKeyNode(MapNode* f) {
    _keyNodes.insert(make_pair(f->seq(),f));
    _lastTrackerFrame = _pendingTrackerFrame;
    _pendingTrackerFrame = f;
  }
  
  void MapCloser::addRelation(MapNodeRelation* r_){
    _relations.insert(r_);
    if (autoProcess){
      MapNodeBinaryRelation* r  =dynamic_cast<MapNodeBinaryRelation*>(r_);
      if (! r)
	return;
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
  }

  void MapCloser::process(){
    if (! _criterion) {
      throw std::runtime_error("no node selection criterion set");
    }
    if (! _selector) {
      throw std::runtime_error("no node selector set");
    }
    if (!_pendingTrackerFrame)
      return;
    std::set<MapNode*> selectedNodes;
    _criterion->setReferencePose(_pendingTrackerFrame->transform());
    selectNodes(selectedNodes,_criterion);
    _partitions.clear();
    makePartitions(_partitions, selectedNodes, _selector);
    cerr << "node: " << _pendingTrackerFrame->seq() 
	 << ", neighbors: " << selectedNodes.size() 
	 << "partitions: " << _partitions.size() << endl;

    if (! _criterion->accept(_pendingTrackerFrame)){
      cerr << "AAAA" << (_keyNodes.find(_pendingTrackerFrame->seq()) != _keyNodes.end()) << endl;
      cerr << _pendingTrackerFrame->transform().matrix() << endl;
      throw std::runtime_error("current frame is not accepted");
      
    }
    _currentPartition = 0;
    int cpi = -1;
    for (size_t i=0; i<_partitions.size(); i++){
      if (_partitions[i].count(_pendingTrackerFrame)){
	_currentPartition=&(_partitions[i]); 
	cpi = i;
	break;
      }
    }
    
    if (! _currentPartition) {
      throw std::runtime_error("no current partition");
    }
    
    ClosureScannedMessage* msg = new ClosureScannedMessage;
    msg->partitions = _partitions;
    msg->currentPartitionIndex = cpi;
    _outputQueue.push_back(msg);

    for (size_t i=0; i<_partitions.size(); i++){
      std::set<MapNode*>* otherPartition = &(_partitions[i]);
      if (_currentPartition == otherPartition)
	continue;
      cerr << "  " << i << "(" << otherPartition->size() << "): ";
      std::list<MapNodeBinaryRelation*> newRelations;
      processPartition(newRelations, *otherPartition, _pendingTrackerFrame);
      // add all new relations to the pool
      for(std::list<MapNodeBinaryRelation*>::iterator it = newRelations.begin(); 
	  it!=newRelations.end(); it++){
	_candidateRelations.push_back(*it);
	_results.push_back(*it);
	_manager->addRelation(*it);
	_relations.insert(*it);
      }
      validatePartitions(*otherPartition, *_currentPartition);
    }
  }
  

  

  void validateRelation(float* translationalErrors,
			float* rotationalErrors ,
			std::vector<MapNodeBinaryRelation*>& relations, 
			MapNodeBinaryRelation* r, 
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
      MapNodeBinaryRelation* r=relations[i];
      MapNode* f0 = r->nodes()[0];
      MapNode* f1 = r->nodes()[1];
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

  void MapCloser::flush(){
    while (! _outputQueue.empty()){
      Serializable* s = _outputQueue.front();
      _outputQueue.pop_front();
      put(s);
    }
  }

  void MapCloser::process(Serializable*s){
    _committedRelations.clear();
    _candidateRelations.clear();
    _outputQueue.push_back(s);
    NewKeyNodeMessage* km = dynamic_cast<NewKeyNodeMessage*>(s);
    if (km){
      addKeyNode(km->keyNode);
    }
    MapNodeRelation* rel = dynamic_cast<MapNodeRelation*>(s);
    if (rel){
      addRelation(rel);
    }
    if(km && _lastTrackerFrame)
      process();
    if (_committedRelations.size()){
      ClosureFoundMessage* closureFound = new ClosureFoundMessage;
      closureFound->closureRelations = _committedRelations;
      _outputQueue.push_back(closureFound);
    }
      
    flush();
  }

  void MapCloser::validatePartitions(std::set<MapNode*>& other, 
				     std::set<MapNode*>& current) {
    // scan for the pwn closure relations connecting a node in current and a node in others
    std::vector<MapNodeBinaryRelation*> rels;
    for (std::set<MapNode*>::iterator it=other.begin(); it!=other.end(); it++){
      MapNode* n=*it;
      if (!n)
	continue;
      std::set<MapNodeRelation*>& nrel=_manager->nodeRelations(n);
      for (std::set<MapNodeRelation*>::iterator rit= nrel.begin(); rit!=nrel.end(); rit++){
	MapNodeBinaryRelation* r=dynamic_cast<MapNodeBinaryRelation*>(*rit);
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
      if (_debug) {
	cerr << "   V( " << rels.size() << ")" << endl;
	cerr << "      current: ";
	for (std::set<MapNode*>::iterator it=current.begin(); it!=current.end(); it++){
	  MapNode* n= *it;
	  cerr << n->seq() << " ";
	}
	cerr<< endl;
	cerr << "      other: ";
	for (std::set<MapNode*>::iterator it=other.begin(); it!=other.end(); it++){
	  MapNode* n= *it;
	  cerr << n->seq() << " ";
	}
	cerr<< endl;
      }
      Eigen::MatrixXf translationalErrors(rels.size(), rels.size());
      Eigen::MatrixXf rotationalErrors(rels.size(), rels.size());

      for (size_t i=0; i<rels.size(); i++){
      	validateRelation(translationalErrors.col(i).data(),
		      rotationalErrors.col(i).data() ,
		      rels, 
		      rels[i], 
		      current);
	ClosureInfo* c = dynamic_cast<ClosureInfo*>(rels[i]);
	c->consensusTimeChecked++;
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
	    ClosureInfo* c = dynamic_cast<ClosureInfo*>(rels[j]);
	    c->consensusCumInlier+=inliersCount;
	  } else {
	    ClosureInfo* c = dynamic_cast<ClosureInfo*>(rels[j]);
	    c->consensusCumOutlierTimes+=1;
	  }
	}
      }

      for (size_t i=0; i<rels.size(); i++){
	MapNodeBinaryRelation* r=rels[i];
	ClosureInfo* c = dynamic_cast<ClosureInfo*>(rels[i]);
	MapNode* n1 = r->nodes()[0];
	MapNode* n2 = r->nodes()[1];
	if (_debug) {
	  cerr << "r" << r << "(" 
	       << n1->seq() << "," << n2->seq() << "): nChecks= " << c->consensusTimeChecked << " inliers="
	       << c->consensusCumInlier << " outliers=" << c->consensusCumOutlierTimes;
	}
	if(c->consensusTimeChecked<_consensusMinTimesCheckedThreshold) {
	  if (_debug) {
	    cerr << "skip" << endl;
	  }
	  continue;
	} 
	if (c->consensusCumInlier>c->consensusCumOutlierTimes){
	  c->accepted = true;
	  _outputQueue.push_back(r);
	  if (_debug) 
	    cerr << "accept" << endl;
	  _committedRelations.push_back(r);
	} else {
	  _manager->removeRelation(r);
	  if (_debug) 
	    cerr << "delete" << endl;
	}
      }
    }
  }

  KeyNodeAcceptanceCriterion::KeyNodeAcceptanceCriterion(MapCloser* closer_, 
							 MapManager* manager_, 
							 PoseAcceptanceCriterion* otherCriterion,
							 int id, boss::IdContext* context) : PoseAcceptanceCriterion(manager_, id, context){
    _closer= closer_;
    _otherCriterion = otherCriterion;
  }

  void KeyNodeAcceptanceCriterion::setReferencePose(const Eigen::Isometry3d& pose_){
    PoseAcceptanceCriterion::setReferencePose(pose_);
    if(_otherCriterion)
      _otherCriterion->setReferencePose(pose_);
  }

  void KeyNodeAcceptanceCriterion::serialize(boss::ObjectData& data, boss::IdContext& context){
    PoseAcceptanceCriterion::serialize(data,context);
    data.setPointer("otherCriterion", _otherCriterion);
    data.setPointer("closer", _closer);
  }

  void KeyNodeAcceptanceCriterion::deserialize(boss::ObjectData& data, boss::IdContext& context){
    PoseAcceptanceCriterion::deserialize(data,context);
    data.getReference("otherCriterion").bind(_otherCriterion);
    data.getReference("closer").bind(_closer);
  }

  bool KeyNodeAcceptanceCriterion::accept(MapNode* n) {
    if (_closer->keyNodes().find(n->seq())==_closer->keyNodes().end())
      return false;
    if (_otherCriterion)
      return _otherCriterion->accept(n);
    return true;
  }


  MapCloserActiveRelationSelector::MapCloserActiveRelationSelector(
								   MapCloser* closer_, 
								   boss_map::MapManager* manager,
								   int id, 
								   boss::IdContext* context): MapRelationSelector(manager, id, context){
    _closer=closer_;
  }

  void MapCloserActiveRelationSelector::serialize(boss::ObjectData& data, boss::IdContext& context){
    MapRelationSelector::serialize(data,context);
    data.setPointer("closer", _closer);
  }

  void MapCloserActiveRelationSelector::deserialize(boss::ObjectData& data, boss::IdContext& context){
    MapRelationSelector::deserialize(data,context);
    data.getReference("closer").bind(_closer);
  }

  bool MapCloserActiveRelationSelector::accept(MapNodeRelation* r) {
    if (_closer->relations().find(r) == _closer->relations().end())
     return false;
    ClosureInfo* cinfo = dynamic_cast<ClosureInfo*>(r);
    if (cinfo){
      return cinfo->accepted;
    }
    return true;
  }

  BOSS_REGISTER_CLASS(ClosureFoundMessage);
  BOSS_REGISTER_CLASS(ClosureScannedMessage);
  BOSS_REGISTER_CLASS(MapCloserActiveRelationSelector);
  BOSS_REGISTER_CLASS(KeyNodeAcceptanceCriterion);
}
