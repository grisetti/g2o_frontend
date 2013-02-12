#ifndef _SE3_OFFSET_ERROR_FUNCTION_H_
#define _SE3_OFFSET_ERROR_FUNCTION_H_

#include "bm_defs.h"
#include "bm_se3.h"
#include "multivariate_vector_function.h"

class SE3OffsetErrorFunction: public MultivariateVectorFunction<Vector6f, Vector6f> {
public:
  SE3OffsetErrorFunction(const Eigen::Isometry3f& odometryMeasurement_,
			 const Eigen::Isometry3f& currentTransform_=Eigen::Isometry3f::Identity(),
			 const Eigen::Isometry3f& sensorOffset_=Eigen::Isometry3f::Identity());

  inline const Eigen::Isometry3f& odometryMeasurement() const {return _odometryMeasurement;}
  inline const Eigen::Isometry3f& sensorOffset() const {return _sensorOffset;}
  inline const Eigen::Isometry3f& currentTransform() const {return _currentTransform;}


  void setOdometryMeasurement(const Eigen::Isometry3f& odometryMeasurement_);
  void setSensorOffset(const Eigen::Isometry3f& sensorOffset_);
  void setCurrentTransform(const Eigen::Isometry3f& currentTransform_);

  // this computes the error
  virtual Vector6f operator()(const Vector6f& x) const;
protected:
  void _updateTemporaries();
  
  Eigen::Isometry3f _sensorOffset;
  Eigen::Isometry3f _odometryMeasurement;
  Eigen::Isometry3f _inverseMeasurement;
  Eigen::Isometry3f _inverseSensorOffset;
  Eigen::Isometry3f _currentTransform;
  Eigen::Isometry3f _A,_B;
};


#endif
