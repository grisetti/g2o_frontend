#include "aligner.h"
#include <iostream>
#include <sys/time.h>

using namespace std;

Aligner::Aligner() {
  _linearizer = 0;
  _correspondenceGenerator = 0;
  _referenceScene = 0;
  _currentScene = 0;
  _outerIterations = 0;
  _innerIterations = 0;
  _T = Eigen::Isometry3f::Identity();
  _initialGuess = Eigen::Isometry3f::Identity();
  _sensorOffset = Eigen::Isometry3f::Identity();
  _totalTime = 0;
  _error = 0;
  _inliers = 0;
};


void Aligner::addRelativePrior(const Eigen::Isometry3f& mean, const Matrix6f& informationMatrix){
  _priors.push_back(new SE3RelativePrior(mean,informationMatrix));
}
void Aligner::addAbsolutePrior(const Eigen::Isometry3f& referenceTransform, const Eigen::Isometry3f& mean, const Matrix6f& informationMatrix){
  _priors.push_back(new SE3AbsolutePrior(referenceTransform, mean,informationMatrix));
}

void Aligner::clearPriors(){
  for (size_t i=0; i<_priors.size(); i++){
    delete _priors[i];
  }
  _priors.clear();
}


void Aligner::align() {
  struct timeval tvStart, tvEnd;
  gettimeofday(&tvStart,0);

  if (! _projector || !_linearizer || !_correspondenceGenerator){
    cerr << "I do nothing since you did not set all required algorithms" << endl;
    return;
  }
  // the current points are seen from the frame of the sensor
  _projector->setTransform(_sensorOffset);
  _projector->project(_correspondenceGenerator->currentIndexImage(),
		      _correspondenceGenerator->currentDepthImage(),
		      _currentScene->points());
  _T = _initialGuess;
  for(int i = 0; i < _outerIterations; i++) {
    //cout << "********************* Iteration " << i << " *********************" << endl;
    
    /************************************************************************
     *                         Correspondence Computation                   *
     ************************************************************************/
    //cout << "Computing correspondences...";
    
    // compute the indices of the current scene from the point of view of the sensor
    _projector->setTransform(_T*_sensorOffset);
    _projector->project(_correspondenceGenerator->referenceIndexImage(),
			_correspondenceGenerator->referenceDepthImage(),
			_referenceScene->points());
    
    // Correspondences computation.    
    _correspondenceGenerator->compute(*_referenceScene, *_currentScene, _T.inverse());

    //cout << " done." << endl;
    //cout << "# inliers found: " << _correspondenceGenerator->numCorrespondences() << endl;

    /************************************************************************
     *                            Alignment                                 *
     ************************************************************************/

    Eigen::Isometry3f invT = _T.inverse();
    for (int k = 0; k < _innerIterations; k++) {      
      invT.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
      Matrix6f H;
      Vector6f b;

      _linearizer->setT(invT);
      _linearizer->update();
      H = _linearizer->H() + Matrix6f::Identity() * 10.0f;
      b = _linearizer->b();
      
      // add the priors
      for (size_t i=0; i<_priors.size(); i++){
	const SE3Prior* prior = _priors[i];
	Vector6f priorError = prior->error(invT);
	Matrix6f priorJacobian = prior->jacobian(invT);
	Matrix6f priorInformationRemapped = prior->errorInformation(invT);

	Matrix6f Hp = priorJacobian.transpose()*priorInformationRemapped*priorJacobian;
	Vector6f bp = priorJacobian.transpose()*priorInformationRemapped*priorError;

	H += Hp;
	b += bp;
      }


      Vector6f dx = H.ldlt().solve(-b);
      Eigen::Isometry3f dT = v2t(dx);
      invT = dT * invT;
    }
    _T = invT.inverse();
    _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
  }

  gettimeofday(&tvEnd, 0);
  double tStart = tvStart.tv_sec*1000+tvStart.tv_usec*0.001;
  double tEnd = tvEnd.tv_sec*1000+tvEnd.tv_usec*0.001;
  _totalTime = tEnd - tStart;
  _error = _linearizer->error();
  _inliers = _linearizer->inliers();
}
