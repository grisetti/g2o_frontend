#include "pwn_merger.h"
#include "g2o_frontend/pwn_core/pwn_static.h"

namespace pwn_tracker {

	PwnMerger::PwnMerger(PwnCloudCache* cache_,
			MapManager* manager_,
			RobotConfiguration* configuration_,
			int id, boss::IdContext* context):
			MapMerger(0,id, context){
		_cache=cache_;
		setManager(manager_);
		setRobotConfiguration(configuration_);
	}

	void PwnMerger::serialize(boss::ObjectData& data, boss::IdContext& context){
		MapMerger::serialize(data, context);
		data.setPointer("merger", _merger);
		data.setPointer("cache", _cache);
	}

	void PwnMerger::deserialize(boss::ObjectData& data, boss::IdContext& context){
		MapMerger::deserialize(data,context);
		data.getReference("merger").bind(_merger);
		data.getReference("cache").bind(_cache);
	}

	void PwnMerger::mergeNodeList(MapNode* big, std::list<MapNode*>& list){
		_merger->clearCloud();
		//_merger->clear();
		SyncSensorDataNode* other;
		CloudWithImageSize* otherCloud;
		SyncSensorDataNode* big_=dynamic_cast<SyncSensorDataNode*>(big);
		PwnCloudCache::HandleType bigCloudHandler = _cache->get(big_); 
        	CloudWithImageSize* bigCloud = bigCloudHandler.get();
		Eigen::Isometry3d iT=big_->transform().inverse();
		Eigen::Isometry3f transformation;
		Eigen::Isometry3f otherOffset;
		PinholeImageData* imdata1 = big_->sensorData()->sensorData<PinholeImageData>(_cache->topic());
		Eigen::Isometry3d otherOffset_ = _robotConfiguration->sensorOffset(imdata1->sensor());
		convertScalar(otherOffset,otherOffset_);


		for (std::list <MapNode*>::iterator it=list.begin(); it!=list.end(); it++){
			other=dynamic_cast<SyncSensorDataNode*>(*it);
			PwnCloudCache::HandleType otherCloudHandler = _cache->get(other); 
        		otherCloud = otherCloudHandler.get();

			Eigen::Isometry3d T=iT*other->transform();
			
			convertScalar(transformation,T);
			_merger->merge(transformation, otherOffset, otherCloud);
			
		}

		bigCloud->points()=_merger->_cloud_tot->points();
		bigCloud->normals()=_merger->_cloud_tot->normals();
		bigCloud->stats()=_merger->_cloud_tot->stats();
		bigCloud->pointInformationMatrix()=_merger->_cloud_tot->pointInformationMatrix();
		bigCloud->normalInformationMatrix()=_merger->_cloud_tot->normalInformationMatrix();
		bigCloud->gaussians()=_merger->_cloud_tot->gaussians();
	}

	BOSS_REGISTER_CLASS(PwnMerger);
}
