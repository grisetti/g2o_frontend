#include "aligner.h"
#include <iostream>

using namespace std;

void Aligner::align() {
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
    cout << "********************* Iteration " << i << " *********************" << endl;
    
    /************************************************************************
     *                         Correspondence Computation                   *
     ************************************************************************/
    cout << "Computing correspondences...";
    
    // compute the indices of the current scene from the point of view of the sensor
    _projector->setTransform(_T*_sensorOffset);
    _projector->project(_correspondenceGenerator->referenceIndexImage(),
			_correspondenceGenerator->referenceDepthImage(),
			_referenceScene->points());
    
    // Correspondences computation.    
    _correspondenceGenerator->compute(*_referenceScene, *_currentScene, _T.inverse());

    cout << " done." << endl;
    cout << "# inliers found: " << _correspondenceGenerator->numCorrespondences() << endl;

    /************************************************************************
     *                            Alignment                                 *
     ************************************************************************/
    for (int k = 0; k < _innerIterations; k++) {      
      Matrix6f H;
      Vector6f b;
      _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
      _linearizer->setT(_T.inverse());
      _linearizer->update();
      H = _linearizer->H() + Matrix6f::Identity() * 10.0f;
      b = _linearizer->b();
      Vector6f dx = H.ldlt().solve(-b);
      Eigen::Isometry3f dT = v2t(dx);
      _T = (dT * _T.inverse()).inverse();
      //_T = _T.inverse();
      _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
    }
  }
}
