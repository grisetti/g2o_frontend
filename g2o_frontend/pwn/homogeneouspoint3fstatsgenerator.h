#ifndef _HOMOGENEOUSPOINT3FSTATSGENERATOR_H_
#define _HOMOGENEOUSPOINT3FSTATSGENERATOR_H_

#include "homogeneouspoint3fstats.h"
#include "homogeneouspoint3fintegralimage.h"
#include "pointprojector.h"

/**
 *  Base class for 3D point stats computation. 
 *  This class allows the computation of the stats for 3D points 
 *  using the associated integral image and the index matrix containing the indices
 *  of the vector elements to fill.
 */

class HomogeneousPoint3fStatsGenerator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  /**
   *  Empty constructor.
   *  This constructor creates an HomogeneousPoint3fStatsGenerator object setting
   *  the world radius to 0.1 meters, the max and min image radius to 30 and 10 
   *  pixels respectively and the minimum number of points needed for points stats
   *  computation to 50.
   */
  HomogeneousPoint3fStatsGenerator();
  
  /**
   *  This method computes the stats for each point using the given points associated 
   *  integral image and index image. The interval image is used to get the region side 
   *  to use for each element.
   *  @param stats is a vector containing the stats computed for each point.
   *  @param _integralImage is the integral image associated to the matrix of points.
   *  @param _intervalImage is an image containing in each element the side of the region
   *  to use in the stats computation.
   *  @param _indexImage is a matrix containing the indices of the vector's elements
   *  to fill.
   */
  void compute(HomogeneousPoint3fStatsVector& stats, 
	       const HomogeneousPoint3fIntegralImage& _integralImage,
	       const Eigen::MatrixXi& _intervalImage,
	       const Eigen::MatrixXi& _indexImage);

 protected:
  /**
   *  Variable containing the max distance in the 3D euclidean space 
   *  where to take the points to use to compute the stats
   *  of the query point.
   */
  float _worldRadius;
  
  /**
   *  Variable containing the max side of the square region used to compute the stats.
   */
  int _maxImageRadius;
  
  /**
   *  Variable containing the min side of the square region used to compute the stats.
   */
  int _minImageRadius;
  
  /**
   *  Variable containing the minimum number of points needed to compute
   *  the stats of a query point.
   */
  int _minPoints;
};

#endif
