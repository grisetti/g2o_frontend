#ifndef _BOSS_LASER_SENSOR_H_
#define _BOSS_LASER_SENSOR_H
#include "bsensor.h"
#include <Eigen/Core>
namespace boss {
  class LaserSensor : public BaseSensor {
  public:
    LaserSensor(int id=-1, IdContext* context = 0);
  };

  class LaserData : public SensorData<LaserSensor>  {
  public:
    LaserData(LaserSensor* sensor=0, 
	      int id=-1, 
	      IdContext* context = 0);
    virtual ~LaserData();
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    inline const std::vector<float>& ranges() const {return _ranges;}
    inline const std::vector<float>& remissions() const {return _remissions;}
    inline std::vector<float>& ranges() {return _ranges;}
    inline std::vector<float>& remissions() {return _remissions;}
    
    inline float minRange() const {return _minRange;}
    inline float maxRange() const {return _maxRange;}
    inline float fov() const {return _fov;}
    inline void setMinRange(float minRange_) {_minRange = minRange_;}
    inline void setMaxRange(float maxRange_) {_maxRange = maxRange_;}
    inline void setFov(float fov_) { _fov = fov_;}
    inline float minAngle() const {return -_fov/2;}
    inline int nBeams() const {return _ranges.size()>_remissions.size()? _ranges.size() : _remissions.size();}
    inline float angularResolution() const {return fov()/nBeams();}
  protected:
    float _minRange, _maxRange, _fov,;
    std::vector<float> _ranges;
    std::vector<float> _remissions;
  };

  
}

#endif
