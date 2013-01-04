#ifndef _DEPTH_IMAGE_H_
#define _DEPTH_IMAGE_H_
#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic> MatrixXus;

class DepthImage: public Eigen::MatrixXf{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DepthImage(int r=0, int c=0);
  DepthImage(const MatrixXus& m);
  void toUnsignedShort(MatrixXus& m, float dmax = std::numeric_limits<float>::max()) const;
  void fromUnsignedShort(const MatrixXus& m);
  bool load(const char* filename); // from PGM
  bool save(const char* filename) const; // to PGM ;
};


#endif
