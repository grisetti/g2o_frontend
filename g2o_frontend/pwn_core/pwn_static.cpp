#include "pwn_static.h"

namespace pwn {
  
  void DepthImage_scale(DepthImage &dest, const DepthImage &src, int step) {
    int rows = src.rows / step;
    int cols = src.cols / step;
    dest.create(rows, cols);
    dest.setTo(cv::Scalar(0));
    for(int r = 0; r < dest.rows; r++) {
      for(int c = 0; c < dest.cols; c++) {
	float acc = 0;
	int np = 0;
	int sr = r * step;
	int sc = c * step;
	for(int i = 0; i < step; i++) {
	  for(int j = 0; j < step; j++) {
	    if(sr + i < src.rows && sc + j < src.cols) {
	      acc += src(sr + i, sc + j);
	      np += src(sr + i, sc + j) > 0;
	    }
	  }
	}
	if(np)
	  dest(r, c) = acc / np;
      }
    }
  }



  void DepthImage_convert_32FC1_to_16UC1(cv::Mat &dest, const cv::Mat &src, float scale) {
 assert(type2str(src.type()) != "32FC1" && "DepthImage_convert_32FC1_to_16UC1: source image of different type from 32FC1");
    cv::Mat m = src * scale;
    m.convertTo(dest, CV_16UC1);
  }

  void DepthImage_convert_16UC1_to_32FC1(cv::Mat &dest, const cv::Mat &src, float scale) {
    assert(type2str(src.type()) != "16UC1" && "DepthImage_convert_16UC1_to_32FC1: source image of different type from 16UC1");
    cv::Mat m;
    src.convertTo(m, CV_32FC1);
    dest = m*scale;
  }
  
}
