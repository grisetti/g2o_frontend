#include "aligner.h"
#include <iostream>

using namespace std;

void Aligner::align() {
  _projector->setTransform(Isometry3f::Identity());
  _projector->project(_currentScene->indexImage(),
		      _currentScene->depthImage(),
		      _currentScene->points());
  _T = _initialGuess;
  for(int i = 0; i < _outerIterations; i++) {
    cout << "********************* Iteration " << i << " *********************" << endl;
    
    /************************************************************************
     *                         Correspondence Computation                   *
     ************************************************************************/
    cout << "Computing correspondences...";
    
    _projector->setTransform(_T.inverse());
    _projector->project(_referenceScene->indexImage(),
			_referenceScene->depthImage(),
			_referenceScene->points());
    
    // Correspondences computation.    
    _correspondenceGenerator.compute(_correspondences,
				     _referenceScene->points(), _currentScene->points(),
				     _referenceScene->normals(), _currentScene->normals(),
				     _referenceScene->indexImage(), _currentScene->indexImage(),
				     _referenceScene->stats(), _currentScene->stats(),
				     _T);

    cout << " done." << endl;
    _numCorrespondences = _correspondenceGenerator.numCorrespondences();
    cout << "# inliers found: " << _numCorrespondences << endl;
 
    /************************************************************************
     *                            Alignment                                 *
     ************************************************************************/
    for (int k = 0; k < _innerIterations; k++) {      
      Matrix6f H;
      Vector6f b;
      _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
      _linearizer->setT(_T);
      _linearizer->update();
      H = _linearizer->H() + Matrix6f::Identity() * 10.0f;;
      b = _linearizer->b();
      Vector6f dx = H.ldlt().solve(-b);
      Eigen::Isometry3f dT = v2t(dx);
      _T = dT * _T;
      _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
    }    
  }
  _T = _sensorOffset * _T.inverse();
}
