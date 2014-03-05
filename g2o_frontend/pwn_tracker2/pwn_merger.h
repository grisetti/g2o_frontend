#pragma once
#include "g2o_frontend/boss_map_building/map_merger.h"
#include "merger2.h"


namespace pwn_tracker {

	using namespace std;
	using namespace pwn;
	using namespace boss;
	using namespace boss_map;
	using namespace boss_map_building;
	using namespace pwn_tracker;

	class PwnMerger: public boss_map_building::MapMerger {
  	public:
		PwnMerger(PwnCloudCache* cache_ = 0,
			MapManager* manager_ = 0,
	     		 RobotConfiguration* configuration_=0,
			int id=0, boss::IdContext* context=0);

		virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
		virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
		virtual void init(){/*_merger->clear();*/}

		inline void setRobotConfiguration(RobotConfiguration* conf) {_robotConfiguration = conf; if (_cache) _cache->_robotConfiguration = conf;} 

		virtual void mergeNodeList(MapNode* big, std::list<MapNode*>& list);
	protected:
		RobotConfiguration* _robotConfiguration;
		PwnCloudCache* _cache;
		Merger2* _merger;
	};
}
