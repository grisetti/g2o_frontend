#include "aligner.h"
#include <iostream>
#include "pinholepointprojector.h"

using namespace std;

void Aligner::align() {
  _projector->setTransform(Isometry3f::Identity());
  _projector->project(_currentIndexImage,
		      _currentDepthImage,
		      *_currentPoints);

  _T = _initialGuess;

  for(int i = 0; i < _outerIterations; i++) {
    _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
      
    cout << "********************* Iteration " << i << " *********************" << endl;
    
    /************************************************************************
     *                         Correspondence Computation                   *
     ************************************************************************/
    cout << "Computing correspondences...";
    
    _projector->setTransform(_T.inverse());
    _projector->project(_referenceIndexImage,
    			_referenceDepthImage,
    			*_referencePoints);
    
    // Correspondences computation.    
    _correspondenceGenerator.compute(_correspondences,
    				     *_referencePoints, *_currentPoints,
    				     *_referenceNormals, *_currentNormals,
    				     _referenceIndexImage, _currentIndexImage,
    				     *_referenceStats, *_currentStats,
    				     _T);
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
  _T = _sensorOffset * _T;
}
