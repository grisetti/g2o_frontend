#ifndef POINT_WITH_NORMAL_STATISTICS_GENERATOR
#define POINT_WITH_NORMAL_STATISTICS_GENERATOR
#include "pointwithnormal.h"
#include "integralpointimage.h"

struct PointWithNormalSVD{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  friend class PointWithNormalStatistcsGenerator;
  PointWithNormalSVD(): 
    _singularValues(Eigen::Vector3f::Zero()),
    _U(Eigen::Matrix3f::Zero()),
    _n(0)
  {}
  inline const Eigen::Vector3f& singularValues() const {return _singularValues;}
  inline const Eigen::Matrix3f& U() const {return _U;}
  inline int n() const {return _n;}
  inline float curvature() const {
    if (_singularValues.squaredNorm()==0) return -1; 
    return _singularValues(0)/(_singularValues(0) + _singularValues(1) + _singularValues(2) );
  }
protected:
  Eigen::Vector3f _singularValues;
  Eigen::Matrix3f _U;
  int _n;
};

typedef std::vector<PointWithNormalSVD, Eigen::aligned_allocator<PointWithNormalSVD> > PointWithNormalSVDVector;

class PointWithNormalStatistcsGenerator{
public:
  PointWithNormalStatistcsGenerator();

  void computeNormalsAndSVD(PointWithNormalVector& points, PointWithNormalSVDVector& svds, const Eigen::MatrixXi& indices, const Eigen::Matrix3f& cameraMatrix);

  // void computeNormals(PointWithNormalVector& points, const Eigen::MatrixXi indices, const Eigen::Matrix3f& cameraMatrix);


  inline int step() const {return _step;}
  inline void setStep(int step_)  {_step=step_;}
  inline int minPoints() const {return _minPoints;}
  inline void setMinPoints(int minPoints_)  {_minPoints=minPoints_;}
  inline int imageRadius() const {return _imageRadius;}
  inline void setImageRadius(int imageRadius_)  {_imageRadius=imageRadius_;}
  inline float worldRadius() const {return _worldRadius;}
  inline void setWorldRadius(float worldRadius_)  {_worldRadius=worldRadius_;}
  inline float maxCurvature() const {return _maxCurvature;}
  inline void setMaxCurvature(float maxCurvature_)  {_maxCurvature=maxCurvature_;}
  inline const Eigen::Matrix3f& cameraMatrix() const {return _cameraMatrix;}
  inline void setCameraMatrix(Eigen::Matrix3f cameraMatrix_)  {_cameraMatrix=cameraMatrix_;}

protected:
  Eigen::Vector2i _range(int r, int c) const;

  int _step;
  int _imageRadius;
  int _minPoints;
  float _worldRadius;
  float _maxCurvature;
  Eigen::Matrix3f _cameraMatrix;
  IntegralPointImage _integralImage;
};

#endif
