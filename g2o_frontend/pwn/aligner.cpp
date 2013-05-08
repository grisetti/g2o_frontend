#include "aligner.h"
#include <iostream>

using namespace std;

void Aligner::align() {
  _projector->setTransform(Isometry3f::Identity());
  _projector->project(*_correspondenceGenerator.currentIndexImage(),
		      *_correspondenceGenerator.currentDepthImage(),
		      _currentScene->points());
  _T = _initialGuess;
  for(int i = 0; i < _outerIterations; i++) {
    cout << "********************* Iteration " << i << " *********************" << endl;
    
    /************************************************************************
     *                         Correspondence Computation                   *
     ************************************************************************/
    cout << "Computing correspondences...";
    
    _projector->setTransform(_T);
    _projector->project(*_correspondenceGenerator.referenceIndexImage(),
			*_correspondenceGenerator.referenceDepthImage(),
			_referenceScene->points());
    
    // Correspondences computation.    
    _correspondenceGenerator.compute(*_referenceScene, *_currentScene, _T.inverse());

    cout << " done." << endl;
    cout << "# inliers found: " << _correspondenceGenerator.numCorrespondences() << endl;

    /************************************************************************
     *                            Alignment                                 *
     ************************************************************************/
    for (int k = 0; k < _innerIterations; k++) {      
      Matrix6f H;
      Vector6f b;
      _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
      _linearizer.setT(_T.inverse());
      _linearizer.update();
      H = _linearizer.H() + Matrix6f::Identity() * 10.0f;
      b = _linearizer.b();
      Vector6f dx = H.ldlt().solve(-b);
      Eigen::Isometry3f dT = v2t(dx);
      _T = dT.inverse() * _T;
      _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
    }    
  }
  _T = _sensorOffset * _T;
}
