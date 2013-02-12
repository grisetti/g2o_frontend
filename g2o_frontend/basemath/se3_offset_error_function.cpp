#include "se3_offset_error_function.h"

SE3OffsetErrorFunction::SE3OffsetErrorFunction(const Eigen::Isometry3f& odometryMeasurement_,
					       const Eigen::Isometry3f& currentTransform_,
					       const Eigen::Isometry3f& sensorOffset_){
  _odometryMeasurement = odometryMeasurement_;
  _currentTransform  = currentTransform_;
  _sensorOffset = sensorOffset_;
  _updateTemporaries();
}


void  SE3OffsetErrorFunction::setOdometryMeasurement(const Eigen::Isometry3f& odometryMeasurement_) {
  _odometryMeasurement = odometryMeasurement_;
  _updateTemporaries();
}

void SE3OffsetErrorFunction::setSensorOffset(const Eigen::Isometry3f& sensorOffset_) { 
  _sensorOffset  = sensorOffset_;
  _updateTemporaries();
}

void SE3OffsetErrorFunction::setCurrentTransform(const Eigen::Isometry3f& currentTransform_) { 
  _currentTransform  = currentTransform_;
  _updateTemporaries();
}


Vector6f SE3OffsetErrorFunction::operator()(const Vector6f& x) const {
  return t2v(_A*v2t(x)*_B);
}

void SE3OffsetErrorFunction::_updateTemporaries()  {
  _inverseMeasurement = _odometryMeasurement.inverse();
  _inverseSensorOffset = _sensorOffset.inverse();
  _A = _inverseMeasurement*_sensorOffset;
  _B = _currentTransform*_inverseSensorOffset;
}

