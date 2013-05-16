#include <omp.h>

#include "pixelmapper.h"
PixelMapper::PixelMapper(){
  _cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  _transform.setIdentity();
  _KRt.setIdentity();
  _iKRt.setIdentity();
  updateTransform();
}

void PixelMapper::updateTransform(){
  Eigen::Isometry3f t=_transform.inverse();
  Eigen::Matrix3f iK=_cameraMatrix.inverse();
  _KR = _cameraMatrix*t.linear();
  _Kt = _cameraMatrix*t.translation();
  _iKR = _transform.linear()*iK;
  _iKt = _transform.translation();
  _KRt.block<3,3>(0,0) = _KR; _KRt.block<3,1>(0,3) = _Kt;
  _iKRt.block<3,3>(0,0) = _iKR; _iKRt.block<3,1>(0,3) = _iKt;
}

void PixelMapper::mergeProjections(Eigen::MatrixXf& depthImage, Eigen::MatrixXi& indexImage,
				   Eigen::MatrixXf* depths, Eigen::MatrixXi* indices, int numImages){
  assert (numImages>0);
  int rows=depths[0].rows();
  int cols=depths[0].cols();
  depthImage.resize(indexImage.rows(), indexImage.cols());
  depthImage.fill(std::numeric_limits<float>::max());
  indexImage.resize(rows, cols);
  indexImage.fill(-1);

  #pragma omp parallel for
  for (int c=0; c<cols; c++){
    int* destIndexPtr = &indexImage.coeffRef(0,c);
    float* destDepthPtr = &depthImage.coeffRef(0,c);
    int* srcIndexPtr[numImages];
    float* srcDepthPtr[numImages];
    for (int i=0; i<numImages; i++){
      srcIndexPtr[i] = &indices[i].coeffRef(0,c);
      srcDepthPtr[i] = &depths[i].coeffRef(0,c);
    }
    for (int r=0; r<rows; r++){
      for (int i=0; i<numImages; i++){
	if (*destDepthPtr>*srcDepthPtr[i]){
	  *destDepthPtr = *srcDepthPtr[i];
	  *destIndexPtr = *srcIndexPtr[i];
	}
	srcDepthPtr[i]++;
	srcIndexPtr[i]++;
      }
      destDepthPtr++;
      destIndexPtr++;
    }
  }

}
