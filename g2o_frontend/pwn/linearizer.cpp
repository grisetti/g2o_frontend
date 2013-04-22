#include "aligner.h"
#include <iostream>
using namespace std;

#include <omp.h>

// inline void _computeHb_tq(Matrix4f& Htt, Matrix4f& Htr, Matrix4f& Hrr,
// 			  Vector4f& bt, Vector4f& br, 
// 			  const HomogeneousPoint3f& p, const HomogeneousNormal3f& n, 
// 			  const Vector4f& pointError, const Vector4f& normalError, 
// 			  const HomogeneousPoint3fOmega& pointOmega, const HomogeneousPoint3fOmega& normalOmega) {
//     Matrix4f Sp = skew(p);
//     Matrix4f Sn = skew(n);
//     Eigen::Matrix4f omegaP = pointOmega;
//     Eigen::Matrix4f omegaN = normalOmega;

//     Htt += omegaP;
//     Htr += omegaP*Sp;
//     Hrr +=Sp.transpose()*omegaP*Sp + Sn.transpose()*omegaN*Sn;
//     const HomogeneousNormal3f ep = omegaP*pointError;
//     const HomogeneousNormal3f en = omegaN*normalError;
//     bt += ep;
//     br += Sp.transpose()*ep + Sn.transpose()*en;
//   }

float Linearizer::update() {
  // Variables initialization.
  _b = Vector6f::Zero();
  _H = Matrix6f::Zero();
  float error = 0.0f;
  int inliers = 0;
  HomogeneousPoint3fOmegaVector* pointOmegas = _aligner->currentPointOmegas();
  HomogeneousPoint3fOmegaVector* normalOmegas = _aligner->currentNormalOmegas();

  //double et = 0, en = 0;

  // allocate the variables for the sum reduction;
  int numThreads = omp_get_max_threads();
  Matrix4f _Htt[numThreads], _Htr[numThreads], _Hrr[numThreads];
  Vector4f _bt[numThreads], _br[numThreads];
  int _inliers[numThreads];
  float _errors[numThreads];
  int iterationsPerThread = _aligner->numCorrespondences()/numThreads;
  #pragma omp parallel
  {
    int threadId=omp_get_thread_num();
    int imin = iterationsPerThread*threadId;
    int imax = imin + iterationsPerThread;
    if (imax > _aligner->numCorrespondences())
      imax = _aligner->numCorrespondences();

    Eigen::Matrix4f& Htt= _Htt[threadId];
    Eigen::Matrix4f& Htr= _Htr[threadId];
    Eigen::Matrix4f& Hrr= _Hrr[threadId];
    Eigen::Vector4f& bt = _bt[threadId];
    Eigen::Vector4f& br = _br[threadId];
    Htt=Matrix4f::Zero(), 
    Htr=Matrix4f::Zero(), 
    Hrr=Matrix4f::Zero();
    bt=Vector4f::Zero(), 
    br=Vector4f::Zero();

    int& inliers = _inliers[threadId];
    float& error=_errors[threadId];
    error = 0;
    inliers = 0;
    
    for(int i = imin; i < imax; i++) {
      __asm__("#here the loop begins");
      const Correspondence& correspondence = _aligner->correspondences()->at(i);
      const HomogeneousPoint3f referencePoint = _T*_aligner->referencePoints()->at(correspondence.referenceIndex);
      const HomogeneousNormal3f referenceNormal = _T*_aligner->referenceNormals()->at(correspondence.referenceIndex);
      const HomogeneousPoint3f& currentPoint = _aligner->currentPoints()->at(correspondence.currentIndex);
      const HomogeneousNormal3f& currentNormal = _aligner->currentNormals()->at(correspondence.currentIndex);
      const HomogeneousPoint3fOmega& omegaP = pointOmegas->at(correspondence.currentIndex);
      const HomogeneousPoint3fOmega& omegaN = normalOmegas->at(correspondence.currentIndex);
      
      const Vector4f pointError = referencePoint - currentPoint;
      const Vector4f normalError = referenceNormal - currentNormal;
      const Vector4f ep = omegaP*pointError;
      const Vector4f en = omegaN*normalError;

      float localError = pointError.dot(ep) + normalError.dot(en);
      if(localError > _inlierMaxChi2) 	continue;
      inliers++;
      error += localError;
      Matrix4f Sp = skew(referencePoint);
      Matrix4f Sn = skew(referenceNormal);
      Htt.noalias() += omegaP;
      Htr.noalias() += omegaP*Sp;
      Hrr.noalias() +=Sp.transpose()*omegaP*Sp + Sn.transpose()*omegaN*Sn;
      bt.noalias() += ep;
      br.noalias() += Sp.transpose()*ep + Sn.transpose()*en;
      __asm__("#here the loop ends");
    }
  }
  // now do the reduce

  Eigen::Matrix4f Htt =  Eigen::Matrix4f::Zero();
  Eigen::Matrix4f Htr =  Eigen::Matrix4f::Zero();
  Eigen::Matrix4f Hrr =  Eigen::Matrix4f::Zero();
  Eigen::Vector4f bt  =  Eigen::Vector4f::Zero();
  Eigen::Vector4f br  =  Eigen::Vector4f::Zero();
  for (int t =0; t<numThreads; t++){
    Htt += _Htt[t];
    Htr += _Htr[t];
    Hrr += _Hrr[t];
    bt  += _bt[t];
    br  += _br[t];
    inliers += _inliers[t];
    error += _errors[t];
  }
  _H.block<3,3>(0,0) = Htt.block<3,3>(0,0);
  _H.block<3,3>(0,3) = Htr.block<3,3>(0,0);
  _H.block<3,3>(3,3) = Hrr.block<3,3>(0,0);
  _H.block<3,3>(3,0) = _H.block<3,3>(0,3).transpose();
  _b.block<3,1>(0,0) = bt.block<3,1>(0,0);
  _b.block<3,1>(3,0) = br.block<3,1>(0,0);
  //cerr << "et: " << et << " en: " << en << " inliers: " << inliers << endl;
  return error;
}
