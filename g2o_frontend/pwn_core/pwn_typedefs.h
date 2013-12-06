#pragma once

#include <Eigen/Core>
#include "opencv2/core/core.hpp"
#include "../basemath/gaussian.h"

namespace pwn {

  typedef Eigen::DiagonalMatrix<float, 3> Diagonal3f;

  typedef cv::Mat_<unsigned char> UCharImage;
  typedef cv::Mat_<char> CharImage;
  typedef cv::Mat_<unsigned short int> UShortIntImage;
  typedef cv::Mat_<unsigned int> UIntImage;
  typedef cv::Mat_<int> IntImage;
  typedef cv::Mat_<float> FloatImage;
  typedef cv::Mat_<double> DoubleImage;
  typedef IntImage IndexImage;
  typedef FloatImage DepthImage;

  typedef struct Gaussian<float, 3> Gaussian3f;

}
