#ifndef _BOSS_LOCAL_MAP_H_
#define _BOSS_LOCAL_MAP_H_

#include "sensor.h"
#include "g2o_frontend/boss/blob.h"
#include <Eigen/Core>
#include "opencv2/highgui/highgui.hpp"

namespace boss_map {  

  class LocalMapData : public BaseSensorData {
  public:
    LocalMapData(int id=-1, IdContext* context = 0);
    ~LocalMapData();
    std::set<BaseSensorData*> _datas;
    std::set<ReferenceFrameRelation*> _originalRelations;
    std::set<ReferenceFrameRelation*> _localRelations;
    virtual BaseSensor* baseSensor() { return 0;}
    virtual const BaseSensor* baseSensor() const { return 0;}
  };

}

#endif
