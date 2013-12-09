#ifndef _BOSS_IMU_SENSOR_H_
#define _BOSS_IMU_SENSOR_H_

#include "sensor.h"
#include <Eigen/Core>

namespace boss_map {
  using namespace boss;

  class IMUSensor : public BaseSensor {
  public:
    IMUSensor(int id=-1, IdContext* context = 0);
  };

  class IMUData : public SensorData<IMUSensor>  {
  public:
    IMUData(IMUSensor* sensor=0, 
	    int id=-1, 
	    IdContext* context = 0);
    virtual ~IMUData();
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    inline const Eigen::Quaterniond& orientation() const {return _orientation;}
    inline const Eigen::Matrix3d& orientationCovariance() const {return _orientationCovariance;}
    inline const Eigen::Vector3d& angularVelocity() const {return _angularVelocity; }
    inline const Eigen::Matrix3d& angularVelocityCovariance() const {return _angularVelocityCovariance;}
    inline const Eigen::Vector3d& linearAcceleration() const {return _linearAcceleration;}
    inline const Eigen::Matrix3d& linearAccelerationCovariance() const {return _linearAccelerationCovariance;}

    inline void setOrientation(const Eigen::Quaterniond& orientation_) {_orientation = orientation_;}
    inline void setOrientationCovariance(const Eigen::Matrix3d& orientationCovariance_)  {_orientationCovariance=orientationCovariance_;}
    inline void setAngularVelocity(Eigen::Vector3d& angularVelocity_)  {_angularVelocity=angularVelocity_; }
    inline void setAngularVelocityCovariance(Eigen::Matrix3d&  angularVelocityCovariance_) {_angularVelocityCovariance=angularVelocityCovariance_;}
    inline void setLinearAcceleration(const Eigen::Vector3d& linearAcceleration_) {_linearAcceleration=linearAcceleration_;}
    inline void setLinearAccelerationCovariance(const Eigen::Matrix3d& linearAccelerationCovariance_) {_linearAccelerationCovariance = linearAccelerationCovariance_;}
  protected:
    Eigen::Quaterniond _orientation;
    Eigen::Matrix3d _orientationCovariance;
    Eigen::Vector3d _angularVelocity;
    Eigen::Matrix3d _angularVelocityCovariance;
    Eigen::Vector3d _linearAcceleration;
    Eigen::Matrix3d _linearAccelerationCovariance;
  };
  
}

#endif
