#include "map_merger.h"

namespace boss_map_building {
  using namespace std;

	MapMerger::MapMerger(MapManager* manager_, int id, boss::IdContext* context):
		StreamProcessor(id,context){
			_manager = manager_;
			_currentBigNode = 0;
			_pendingTrackerFrame = 0;
			_firstTrackerFrame = 0;
			_lastTrackerFrame=0;
			_previousTrackerFrame=0;
			_lastBigNode = 0;
			_counter=0;
			_listSize=0;
			_localT=Eigen::Isometry3d::Identity();
		}

	void MapMerger::serialize(boss::ObjectData& data, boss::IdContext& context){
		StreamProcessor::serialize(data, context);
		data.setPointer("manager", _manager);
		data.setInt("listSize", _listSize);
	}

	void MapMerger::deserialize(boss::ObjectData& data, boss::IdContext& context){
		StreamProcessor::deserialize(data,context);
		data.getReference("manager").bind(_manager);
		 _listSize = data.getInt("listSize");
	}

	void MapMerger::setManager(MapManager* m){
	 _manager = m;
	}

	void MapMerger::flushQueue(){
		while (! _outputQueue.empty()){
			put(_outputQueue.front());
			_outputQueue.pop_front();
		}
	}

	void MapMerger::process(Serializable* s){
		NewKeyNodeMessage* km = dynamic_cast<NewKeyNodeMessage*>(s);
		if (!km){
			return;
		}else{
			_pendingTrackerFrame=km->keyNode;
			if(_counter==0){
				init();
				_nodeList.clear();
				_nodeList.push_back(_pendingTrackerFrame);
				_currentBigNode=_pendingTrackerFrame;
				_first=_pendingTrackerFrame->transform();
			}else
				_nodeList.push_back(_pendingTrackerFrame);
			_counter++;
			if(_counter>_listSize){
				_counter=0;

				mergeNodeList(_currentBigNode, _nodeList);
				
				MapNodeBinaryRelation* r=computeRelation(_currentBigNode, _lastBigNode);
				
				//if(_lastBigNode)
				//	alignment(_currentBigNode, _lastBigNode);				
				_lastBigNode=_currentBigNode;
				
				if(!r){
					_outputQueue.push_back(new NewKeyNodeMessage(_lastBigNode));
				}else{
					_outputQueue.push_back(r);
					flushQueue();
					_outputQueue.push_back(new NewKeyNodeMessage(_lastBigNode));
					_manager->addRelation(r);
					flushQueue();

				}
				_previous=_first;
			}
		}
	}

	void MapMerger::mergeNodeList(MapNode* big, std::list<MapNode*>& list){

	}

	MapNodeBinaryRelation* MapMerger::computeRelation(MapNode* current, MapNode* last){
		//current->setTransform(_pendingTrackerFrame->transform());
		//current->setTransform(_firstTrackerFrame->transform());
		Eigen::Isometry3d tr;
		if (last){
			_localT=_previous.inverse()*_first;
			tr=(last->transform()*_localT);
		}
		else
			tr=_first;
		current->setTransform(tr);
		if(!last){
			return 0;
		}
		
		Eigen::Isometry3d T=last->transform().inverse()*current->transform();
		MapNodeBinaryRelation* r=new MapNodeBinaryRelation(_manager);
		r->nodes()[0]=last;
		r->nodes()[1]=current;
		r->setTransform(T);
		Matrix6d info = Matrix6d::Identity();
		info.block<3,3>(0,0) = Eigen::Matrix3d::Identity()*10;
		info.block<3,3>(3,3) = Eigen::Matrix3d::Identity()*100;
		r->setInformationMatrix(info);
		return r;
	}

	BOSS_REGISTER_CLASS(MapMerger);
}



