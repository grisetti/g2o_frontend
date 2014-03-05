#pragma once

#include "g2o_frontend/boss_map/map_utils.h"
#include "g2o_frontend/boss_map/stream_processor.h"
#include "base_tracker.h"

namespace boss_map_building {
  using namespace boss;
  using namespace boss_map;

	class MapMerger: public boss_map::StreamProcessor {
	public:
		MapMerger(MapManager* manager_=0, int id=-1, boss::IdContext* context=0);
		void serialize(boss::ObjectData& data, boss::IdContext& context);
    		void deserialize(boss::ObjectData& data, boss::IdContext& context);

		virtual void setManager(MapManager* manager);

		virtual void init(){}
		virtual void mergeNodeList(MapNode* big, std::list<MapNode*>& list);
		MapNodeBinaryRelation* computeRelation(MapNode* current, MapNode* last);

		virtual void process(Serializable* s);
		void flushQueue();
		
	protected:
		MapNode* _lastBigNode, *_pendingTrackerFrame, *_currentBigNode, *_firstTrackerFrame, *_lastTrackerFrame, *_previousTrackerFrame;
		int _counter;
		int _listSize;
		boss_map::MapManager* _manager;
		std::list<Serializable*> _outputQueue;
		Eigen::Isometry3d _localT,_previous, _first;
		std::list<MapNode*> _nodeList;
	};
}

