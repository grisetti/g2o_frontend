#include "pwn_closer_with_merger.h"
#include "g2o_frontend/pwn_core/pwn_static.h"
#include "pwn_closer.h"

#include <sstream>

namespace pwn_tracker {


  PwnCloserWithMerger::PwnCloserWithMerger(PwnMatcherBase* matcher_,
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
    _closureClampingDistance = 1e9;
    setMatcher(matcher_);
    setManager(manager_);
    setCache(cache_);
    setRobotConfiguration(configuration_);

  }

  void PwnCloserWithMerger::serialize(boss::ObjectData& data, boss::IdContext& context){
    MapCloser::serialize(data,context);
    data.setPointer("merger", _merger);
    data.setPointer("matcher", _matcher);
    data.setPointer("cache", _cache);
    data.setInt("frameMinNonZeroThreshold", _frameMinNonZeroThreshold);
    data.setInt("frameMaxOutliersThreshold", _frameMaxOutliersThreshold);
    data.setInt("frameMinInliersThreshold", _frameMinInliersThreshold);
    data.setFloat("closureClampingDistance", _closureClampingDistance);
  }
    
  
  void PwnCloserWithMerger::deserialize(boss::ObjectData& data, boss::IdContext& context){
    MapCloser::deserialize(data,context);
    data.getReference("merger").bind(_merger);
    data.getReference("matcher").bind(_matcher);
    data.getReference("cache").bind(_cache);
    _frameMinNonZeroThreshold = data.getInt("frameMinNonZeroThreshold");
    _frameMaxOutliersThreshold = data.getInt("frameMaxOutliersThreshold");
    _frameMinInliersThreshold = data.getInt("frameMinInliersThreshold");
    _closureClampingDistance = data.getFloat("closureClampingDistance");
   }


  /*
  void PwnCloserWithMerger::deserializeComplete(){
    boss::Identifiable::deserializeComplete();
    if (_tracker)
      setTracker(_tracker);
 }
  */
  void PwnCloserWithMerger::process(Serializable* s) {
    if (_enabled){
      MapCloser::process(s);
    }
    else
      put(s);
  }

	void PwnCloserWithMerger::processCurrentPartition(SyncSensorDataNode* current){
		_merger->clear();
		_currentPartitionImage=DepthImage::zeros(_merger->_r,_merger->_c);
		Eigen::Isometry3d  otherOffset_,tr;
		SyncSensorDataNode* other;
		int som=(int)floor(_currentPartition->size()/3);
		if (som==0) som=1;
		int iter=-1;
		//cerr<<"\n current partition : ";
		PinholeImageData* imdata = current->sensorData()->sensorData<PinholeImageData>(_cache->topic());
		otherOffset_ = _robotConfiguration->sensorOffset(imdata->sensor());
		tr=otherOffset_;
		mergeNode(_currentPartitionImage,current,tr);
		_currentPartitionActive.push_back(current);
		/*for (std::set <MapNode*>::iterator it=_currentPartition->begin(); it!=_currentPartition->end(); it++){
			iter++;
			if(((iter)%som)!=0)
				continue;

		       other = dynamic_cast<SyncSensorDataNode*>(*it);
		      if (other==current)
			continue;

			 PinholeImageData* imdata = other->sensorData()->sensorData<PinholeImageData>(_cache->topic());
			otherOffset_ = _robotConfiguration->sensorOffset(imdata->sensor());

			tr=other->transform().inverse()*current->transform()*otherOffset_;
			mergeNode(_currentPartitionImage,other,tr);
			if (_merger->_image_overlapping_points_count>5000) {
				cerr << "o";
				_currentPartitionActive.push_back(other);
		      } else {
				cerr << ".";
			}
		}*/
   	 cerr << endl;
	}

  void PwnCloserWithMerger::processPartition(std::list<MapNodeBinaryRelation*>& newRelations, 
				   std::set<MapNode*>& otherPartition, 
				   MapNode* current_){

	SyncSensorDataNode* current = dynamic_cast<SyncSensorDataNode*>(current_);
    if (otherPartition.count(current)>0)
      return;
    Eigen::Isometry3d iT=current->transform().inverse();
    //PwnCloudCache::HandleType f_handle=_cache->get(current);
    //CloudWithImageSize* currentCloud = f_handle.get();
    //cerr << "FRAME: " << current->seq << endl; 

	Eigen::Isometry3d  otherOffset_,tr;
	 Eigen::Isometry3f transformation;

	if(_current!=current){
		_current=current;
		_currentPartitionActive.clear();
		processCurrentPartition(current);
	}
	_merger->clear();
	_otherPartitionImage=DepthImage::zeros(_merger->_r,_merger->_c);
    	
	_nodeList.clear();
	SyncSensorDataNode* other;

	int som=(int)round(otherPartition.size()/8);
	if (som==0) som=1;
	int iter=-1;
    for (std::set <MapNode*>::iterator it=otherPartition.begin(); it!=otherPartition.end(); it++){
	iter++;
	if((iter%som)!=0)
		continue;

       other = dynamic_cast<SyncSensorDataNode*>(*it);
      if (other==current)
	continue;

	 PinholeImageData* imdata = other->sensorData()->sensorData<PinholeImageData>(_cache->topic());
	otherOffset_ = _robotConfiguration->sensorOffset(imdata->sensor());

	tr=other->transform().inverse()*current->transform()*otherOffset_;
	mergeNode(_otherPartitionImage,other,tr);
	if (_merger->_image_overlapping_points_count>4000) {
		cerr << "o";
		_nodeList.push_back(other);
      } else {
		cerr << ".";
	}
    }
    cerr << endl;
	
	if(/*_merger->_image_points_count>1000&&*/_nodeList.size()>0){
		//std::cout<<"_____p count ____"<<_merger->_image_points_count<<std::endl;

		tr=otherOffset_;
		convertScalar(transformation,tr);
		_merger->matchWithPartition(_currentPartitionImage, transformation, _otherPartitionImage);
	
		if(_merger->_result.image_nonZeros < _frameMinNonZeroThreshold/2 ||
			_merger->_result.image_outliers > _merger->_result.image_inliers/8|| 
			_merger->_result.image_inliers  < _frameMinInliersThreshold/2) {
			//cerr<<"non zero "<<_merger->_result.image_nonZeros<<endl;
			//cerr<<"im out "<<_merger->_result.image_outliers<<endl;
			//cerr<<"im in "<<_merger->_result.image_inliers<<endl;
			cerr << "XXXXXXXXXXXXXXXXXXX";
		}else{
			Eigen::Isometry3d current_ris_iT=current->transform()*_merger->_result.transform*iT;
			Eigen::Isometry3d current_ris_iT_nodo;

			for (std::list <MapNode*>::iterator it=_nodeList.begin(); it!=_nodeList.end(); it++){
				
				SyncSensorDataNode* nodo;
				nodo = dynamic_cast<SyncSensorDataNode*>(*it);
				current_ris_iT_nodo=current_ris_iT*nodo->transform();
				for (std::list <MapNode*>::iterator it2=_currentPartitionActive.begin(); it2!=_currentPartitionActive.end(); it2++){
					SyncSensorDataNode* nodo2;
					nodo2 = dynamic_cast<SyncSensorDataNode*>(*it2);

					Eigen::Isometry3d backup=_merger->_result.transform;
					tr=nodo2->transform().inverse()*current_ris_iT_nodo;
					
					_merger->_result.transform=tr;

					PwnCloserRelation* r=new PwnCloserRelation(_manager);
					r->nodes()[0]=nodo2;
					r->nodes()[1]=nodo;
					r->fromResult(_merger->_result);
					Matrix6d info = Matrix6d::Identity();
					info.block<3,3>(0,0) = Eigen::Matrix3d::Identity()*100;
					info.block<3,3>(3,3) = Eigen::Matrix3d::Identity()*1000;
					r->setInformationMatrix(info);
					newRelations.push_back(r);
					_merger->_result.transform=backup;
				}
			}
		}
		cerr << endl;
		
	}
  }

	void PwnCloserWithMerger::mergeNode(DepthImage& out, SyncSensorDataNode* other,Eigen::Isometry3d& T){
		PwnCloudCache::HandleType _otherCloudHandler = _cache->get(other); 
        	CloudWithImageSize* otherCloud = _otherCloudHandler.get();
		PinholePointProjector* cpp=dynamic_cast<PinholePointProjector*>(_merger->depthImageConverter()->projector());
		IntImage iim;
		DepthImage dim;
		Eigen::Isometry3f transformation;
	   	 convertScalar(transformation,T);
		cpp->setTransform(transformation);
		cpp->project(iim, 
			    dim, 
			    otherCloud->points());
		_merger->mergeDepthImage(out,dim);
		
	}


  BOSS_REGISTER_CLASS(PwnCloserWithMerger);
}


