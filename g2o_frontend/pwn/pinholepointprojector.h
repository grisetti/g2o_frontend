#ifndef _PINHOLEPOINTPROJECTOR_H_
#define _PINHOLEPOINTPROJECTOR_H_

#include "pointprojector.h"

class PinholePointProjector: public PointProjector {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PinholePointProjector();
  virtual ~PinholePointProjector();
  

  inline const Eigen::Matrix3f& cameraMatrix() const {return _cameraMatrix;}
  virtual void setTransform(const Eigen::Isometry3f& transform_);
  virtual void setCameraMatrix(const Eigen::Matrix3f& cameraMatrix_);

  virtual void project(Eigen::MatrixXi& indexImage, 
			     Eigen::MatrixXf& depthImage, 
			     const HomogeneousPoint3fVector& points) const;

  void projectIntervals(Eigen::MatrixXi& intervalImage, 
			const Eigen::MatrixXf& depthImage, 
			float worldRadius) const;
  
  virtual void unProject(HomogeneousPoint3fVector& points, 
			 Eigen::MatrixXi& indexImage, 
                         const Eigen::MatrixXf& depthImage) const;

  virtual int projectInterval(int x, int y, float d, float worldRadius) const;
  virtual bool project(int& x, int&y, float&f, const HomogeneousPoint3f& p) const;
  virtual bool unProject(HomogeneousPoint3f& p, int x, int y, float d) const;
 
  //protected:
  Eigen::Matrix3f _cameraMatrix;
  Eigen::Matrix4f _KRt;
  Eigen::Matrix4f _iKRt;
  
  void _updateMatrices();

  inline bool _project(int& x, int&y, float&d, const HomogeneousPoint3f& p) const {
    Eigen::Vector4f ip=_KRt*p;
    d=ip.coeff(2);
    if (d<_minDistance || d>_maxDistance)
      return false;
    ip*= (1./d);
    x=(int)round(ip.coeff(0));
    y=(int)round(ip.coeff(1)); 
    return true;
  }
  
  inline  bool  _unProject(HomogeneousPoint3f& p, int x, int y, float d) const {
    if (d<_minDistance || d>_maxDistance)
      return false;
    p=_iKRt*Eigen::Vector4f(x*d,y*d,d,1.0);
    return true;
  }
  
  inline int _projectInterval(int, int, float d, float worldRadius) const {
    if (d<_minDistance || d>_maxDistance)
      return -1;
    Eigen::Matrix<float, 3,2> range;
    Eigen::Vector3f p=_cameraMatrix*Eigen::Vector3f(worldRadius,worldRadius,0);
    p*=(1./d);
    if (p.coeff(0)>p.coeff(1))
      return p.coeff(0);
    return p.coeff(1);
}
private:
  Eigen::Matrix3f _iK;
  Eigen::Matrix3f _KR;
  Eigen::Vector3f _Kt;
  Eigen::Matrix3f _iKR;
  Eigen::Vector3f _iKt;
};

#endif
