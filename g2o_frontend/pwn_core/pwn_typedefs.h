#pragma once

#include <Eigen/Core>
#include "opencv2/core/core.hpp"
#include "../basemath/gaussian.h"

namespace pwn {

  typedef Eigen::DiagonalMatrix<float, 3> Diagonal3f;

  typedef cv::Mat_<unsigned char> UnsignedCharImage;
  typedef cv::Mat_<char> CharImage;
  typedef cv::Mat_<unsigned short> UnsignedShortImage;
  typedef cv::Mat_<unsigned int> UnsignedIntImage;
  typedef cv::Mat_<int> IntImage;
  typedef cv::Mat_<float> FloatImage;
  typedef cv::Mat_<double> DoubleImage;
  typedef UnsignedShortImage RawDepthImage;
  typedef IntImage IndexImage;
  typedef FloatImage DepthImage;

  typedef struct Gaussian<float, 3> Gaussian3f;

}
