#ifndef _GAUSSIAN3_H_
#define _GAUSSIAN3_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include "pointwithnormal.h"

struct Gaussian3f {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Gaussian3f() {
    _covarianceMatrix = Eigen::Matrix3f::Zero();
    _informationMatrix = Eigen::Matrix3f::Zero();
    _informationVector = Eigen::Vector3f::Zero();
    _mean = Eigen::Vector3f::Zero();
  }
  
  Gaussian3f(const Eigen::Vector3f& v, const Eigen::Matrix3f& m, bool useInfoForm = false) {
    _momentsUpdated = _infoUpdated = false; 
    if (useInfoForm){
      _informationVector=v;
      _informationMatrix = m;
      _infoUpdated = true;
    } else {
      _mean=v;
      _covarianceMatrix = m;
      _momentsUpdated = true;
    }
  }

  inline Gaussian3f& addInformation(const Gaussian3f& g){
    _updateInfo();
    _informationMatrix += g.informationMatrix();
    _informationVector += g.informationVector();
    _momentsUpdated = false;
    return *this;
  }

  inline Gaussian3f& addNoise(const Gaussian3f& g){
    _updateMoments();
    _covarianceMatrix += g.covarianceMatrix();
    _mean += g.mean();
    _infoUpdated = false;
    return *this;
  }


  inline const Eigen::Matrix3f& covarianceMatrix() const {_updateMoments(); return _covarianceMatrix; }
  inline const Eigen::Vector3f& mean() const {_updateMoments(); return _mean; }
  inline const Eigen::Matrix3f& informationMatrix() const { _updateInfo(); return _informationMatrix; }
  inline const Eigen::Vector3f& informationVector() const { _updateInfo();return _informationVector; }
protected:
  inline void _updateMoments() const {
    if (_momentsUpdated)
      return;
    _covarianceMatrix = _informationMatrix.inverse();
    _mean = _covarianceMatrix*_informationVector;
    _momentsUpdated = true;
  }

  inline void _updateInfo() const {
    if (_infoUpdated)
      return;
    _informationMatrix = _covarianceMatrix.inverse();
    _informationVector = _informationMatrix*_mean;
    _infoUpdated = true;
  }

  mutable Eigen::Matrix3f _informationMatrix;
  mutable Eigen::Vector3f _informationVector;
  mutable Eigen::Matrix3f _covarianceMatrix;
  mutable Eigen::Vector3f _mean;
  mutable bool _momentsUpdated;
  mutable bool _infoUpdated;
};

inline Gaussian3f operator*(const Eigen::Isometry3f& t, const Gaussian3f& g){
  return Gaussian3f(t*g.mean(), t.linear() * g.covarianceMatrix() * t.linear().transpose(), false);
};


class Gaussian3fVector: public std::vector<Gaussian3f, Eigen::aligned_allocator<Gaussian3f> > {
public:
  Gaussian3fVector(size_t s=0, const Gaussian3f& p=Gaussian3f());
  void toDepthImage(Eigen::MatrixXf& depthImage, 
		    const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose, 
		    float dmax = std::numeric_limits<float>::max()) const;

  void fromDepthImage(const Eigen::MatrixXf& depthImage, 
		      const Eigen::Matrix3f& cameraMatrix, 
		      float dmax = std::numeric_limits<float>::max(), 
		      float baseline = 0.075, float alpha=0.1);

  void toIndexImage(Eigen::MatrixXi& indexImage, Eigen::MatrixXf& zBuffer, 
		    const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose, 
		    float dmax = std::numeric_limits<float>::max()) const;

  void toPointWithNormalVector(PointWithNormalVector& dest) const;
  //bool save(const char* filename, int step=1) const;
  //bool load(const char* filename);

};

#endif
