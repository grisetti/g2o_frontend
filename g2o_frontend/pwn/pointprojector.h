#ifndef _POINTPROJECTOR_H_
#define _POINTPROJECTOR_H_

#include "homogeneousvector4f.h"
#include "depthimage.h"

class PointProjector {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PointProjector();
  virtual ~PointProjector();
  virtual const Eigen::Isometry3f& transform() const;
  virtual void setTransform(const Eigen::Isometry3f& transform_);

  inline float minDistance() const {return _minDistance;}
  inline void  setMinDistance(const float& minDistance_) {_minDistance = minDistance_;}
  inline float maxDistance() const {return _maxDistance;}
  inline void  setMaxDistance(const float& maxDistance_) {_maxDistance = maxDistance_;}
  virtual void project(Eigen::MatrixXi& indexImage, 
		       Eigen::MatrixXf& depthImage, 
		       const HomogeneousPoint3fVector& points) const;

  virtual void projectIntervals(Eigen::MatrixXi& intervalImage, 
			      const Eigen::MatrixXf& depthImage, 
			      float worldRadius) const;

  virtual void unProject(HomogeneousPoint3fVector& points, 
			 Eigen::MatrixXi& indexImage, 
                         const Eigen::MatrixXf& depthImage) const;


  virtual bool project(int& x, int&y, float&f, const HomogeneousPoint3f& p) const = 0;
  virtual int projectInterval(int x, int y, float d, float worldRadius) const = 0;
  virtual bool unProject(HomogeneousPoint3f& p, int x, int y, float d) const = 0;



protected:
  Eigen::Isometry3f _transform;
  float _minDistance, _maxDistance;
};


#endif
