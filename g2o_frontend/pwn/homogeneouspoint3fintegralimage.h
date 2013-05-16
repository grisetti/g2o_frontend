#ifndef _HOMOGENEOUSPOINT3INTEGRALIMAGE_H_
#define _HOMOGENEOUSPOINT3INTEGRALIMAGE_H_

#include "homogeneouspoint3faccumulator.h"

/** \struct HomogeneousPoint3fIntegralImage
 *  \brief Base class for integral images representation.
 *
 *  This class allows to create and query integral images. Integral images
 *  are particular matrices where the value of each element is the sum of
 *  all the elements in the left upper block of the matrix, element itself
 *  included. The matrix used for this kind of integral images is composed 
 *  by HomogeneousPoint3fAccumulator. This class extends the Eigen Matrix class.
 */

struct HomogeneousPoint3fIntegralImage : public Eigen::Matrix<HomogeneousPoint3fAccumulator, Eigen::Dynamic, Eigen::Dynamic> {
  /**
   *  Empty constructor.
   *  This constructor creates an integral image using the constructor of the Eigen Matrix class.
   *  The matrix is initialized to have zero dimensions.
   */
  HomogeneousPoint3fIntegralImage();
  
  /**
   *  This method computes the integral images using a vector of points and an index image
   *  containing the indices of the points to use on each element position.
   *  @param pointIndices is the index image containing the indices of the points to use.
   *  @param points is the vector of homogeneous point to use to compute the integral image.
   */
  void compute(const Eigen::MatrixXi &pointIndices, const HomogeneousPoint3fVector &points);
  
  /**
   *  This method clears all the matrix elements.
   */
  void clear();
  
  /**
   *  This method given the four corners of a rectangular region, it computes the
   *  point accumulator of that region.
   *  @param xmin is the left upper corner index of the region.
   *  @param xmax is the left lower corner index of the region.
   *  @param ymin is the right upper corner index of the region.
   *  @param ymax is the right lower corner index of the region.
   *  @return the point accumulator of the region given in input.
   */
  HomogeneousPoint3fAccumulator getRegion(int xmin, int xmax, int ymin, int ymax) const;
};

#endif
