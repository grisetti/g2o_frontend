#include "pwn_typedefs.h"

namespace pwn {
  
  void DepthImage_scale(DepthImage &dest, const DepthImage &src, int step);
  
  void DepthImage_convert_32FC1_to_16UC1(cv::Mat &dest, const cv::Mat &src, float scale = 1000.0f );

  void DepthImage_convert_16UC1_to_32FC1(cv::Mat &dest, const cv::Mat &src, float scale = 0.001f );
}
