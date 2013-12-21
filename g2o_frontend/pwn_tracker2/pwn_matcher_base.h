#pragma once

#include "g2o_frontend/pwn_core/cloud.h"
#include "g2o_frontend/pwn_core/pinholepointprojector.h"
#include "g2o_frontend/pwn_core/depthimageconverterintegralimage.h"
#include "g2o_frontend/pwn_core/aligner.h"


namespace pwn_tracker {
  using namespace pwn;

  struct PwnMatcherBase{
    struct MatcherResult{
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      Eigen::Isometry3d transform;
      Matrix6d informationMatrix;
      int cloud_inliers;
      int image_nonZeros;
      int image_outliers;
      int image_inliers;
      float image_reprojectionDistance;
    };

    PwnMatcherBase(pwn::Aligner* aligner_, pwn::DepthImageConverter* converter_);

    inline int scale() const {return _scale;}
    inline void setScale (int scale_) {_scale = scale_;}

    inline pwn::Aligner* aligner() { return _aligner;}
    inline void setAligner(pwn::Aligner* aligner_) { _aligner=aligner_;}

    inline pwn::DepthImageConverter* converter() { return _converter;}
    inline void setConverter(pwn::DepthImageConverter* converter_) { _converter=converter_;}


    pwn::Cloud* makeCloud(int& r, int& c,
			  Eigen::Matrix3f& cameraMatrix, 
			  const Eigen::Isometry3f& sensorOffset, 
			  const DepthImage& depthImage);

    void matchClouds(PwnMatcherBase::MatcherResult& result, 
		     pwn::Cloud* fromCloud, pwn::Cloud* toCloud,
		     const Eigen::Isometry3f& fromOffset_, const Eigen::Isometry3f& toOffset_, 
		     const Eigen::Matrix3f& toCameraMatrix_,
		     int toRows, int toCols,
		     const Eigen::Isometry3d& initialGuess=Eigen::Isometry3d::Identity());

    void makeThumbnails(cv::Mat& depthThumbnail, cv::Mat& normalThumbnail, 
			pwn::Cloud* f, int r, int c, 
			const Eigen::Isometry3f& offset, 
			const Eigen::Matrix3f& cameraMatrix,
			float scale);

    float _frameInlierDepthThreshold;

    double cumTime;
    int numCalls;
  protected:
    Aligner* _aligner;
    DepthImageConverter* _converter;
    int _scale;
  };



  template <typename T1, typename T2>
  void convertScalar(T1& dest, const T2& src){
    for (int i=0; i<src.matrix().cols(); i++)
      for (int j=0; j<src.matrix().rows(); j++)
	dest.matrix()(j,i) = src.matrix()(j,i);

  }


}
