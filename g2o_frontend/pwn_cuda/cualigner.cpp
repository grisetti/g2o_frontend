#include "cualigner.h"
#include <iostream>
#include "cudaaligner.h"
#include "g2o_frontend/pwn/pinholepointprojector.h"

namespace CudaAligner {
  using namespace std;

static CudaAligner::AlignerContext* context = 0 ;

void CuAligner::align() {
  AlignerStatus status;
  char buf[1024];
  if (! context){
    status = createContext(&context, 640*480, 640*480, 640, 480);
    status.toString(buf);
    cerr << "STATUS: " << buf << endl; 
  }
  _projector->setTransform(Isometry3f::Identity());
  _projector->project(_currentIndexImage,
		      _currentDepthImage,
		      *_currentPoints);
  Vector6f v;

  // v << .1, .1, .1, .2, .2, .2;

  // _T = v2t(v);

  _T = _initialGuess;

  float referenceCurvatures[_referenceStats->size()];
  float currentCurvatures[_currentStats->size()];
  for (int i=0; i<_referenceStats->size(); i++)
    referenceCurvatures[i]=_referenceStats->at(i).curvature();
  for (int i=0; i<_currentStats->size(); i++)
    currentCurvatures[i]=_currentStats->at(i).curvature();
  {

    PinholePointProjector *pprojector = (PinholePointProjector *) _projector;
    cerr << "INIT" << endl;
    cerr << "camera amtrix: " << pprojector->cameraMatrix() << endl;
    status = initComputation(context,
					  &(pprojector->cameraMatrix().coeffRef(0,0)),
					  &(_referencePoints->at(0).coeffRef(0)),
					  &(_referenceNormals->at(0).coeffRef(0)),
					  referenceCurvatures,
					  _referencePoints->size(),
					  &(_currentPoints->at(0).coeffRef(0)),
					  &(_currentNormals->at(0).coeffRef(0)),
					  currentCurvatures,
					  &(_currentPointOmegas->at(0).coeffRef(0,0)),
					  &(_currentNormalOmegas->at(0).coeffRef(0,0)),
					  _currentPoints->size());
    
    status.toString(buf);
    cerr << "STATUS: " << buf << endl; 
  }
  for(int i = 0; i < _outerIterations; i++) {
    _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
      
    cout << "********************* Iteration " << i << " *********************" << endl;
    
    /************************************************************************
     *                         Correspondence Computation                   *
     ************************************************************************/
    //cout << "Computing correspondences...";
    
    // _projector->setTransform(_T.inverse());
    // _projector->project(_referenceIndexImage,
    // 			_referenceDepthImage,
    // 			*_referencePoints);
    
    // // Correspondences computation.    
    // _correspondenceGenerator.compute(_correspondences,
    // 				     *_referencePoints, *_currentPoints,
    // 				     *_referenceNormals, *_currentNormals,
    // 				     _referenceIndexImage, _currentIndexImage,
    // 				     *_referenceStats, *_currentStats,
    // 				     _T);

    Matrix6f myH;
    Vector6f myb;
    {
      cerr << "ITERATE" << endl;
      status = simpleIteration(context,
					    &(_T.matrix().coeffRef(0,0)));
      /*
      for (int i=0; i<_referenceIndexImage.cols(); i++)
      	for (int j=0; j<_referenceIndexImage.rows(); j++){
      	  cout << _referenceIndexImage.coeffRef(j,i) << " ";
      	}
      */
      status.toString(buf);
      cerr << "STATUS: " << buf << endl; 
      Eigen::Matrix4f Htt, Htr, Hrr;
      Eigen::Vector4f bt, br;
      getHb(context,
			 &(Htt.coeffRef(0,0)), 
			 &(Htr.coeffRef(0,0)), 
			 &(Hrr.coeffRef(0,0)), 
			 &(bt.coeffRef(0,0)), 
			 &(br.coeffRef(0,0)));
      myH.block<3,3>(0,0) = Htt.block<3,3>(0,0);
      myH.block<3,3>(0,3) = Htr.block<3,3>(0,0);
      myH.block<3,3>(3,3) = Hrr.block<3,3>(0,0);
      myH.block<3,3>(3,0) = myH.block<3,3>(0,3).transpose();
      myb.block<3,1>(0,0) = bt.block<3,1>(0,0);
      myb.block<3,1>(3,0) = br.block<3,1>(0,0);
    }
    int acc = 0;
    _numCorrespondences = _correspondenceGenerator.numCorrespondences();
    cout << "# inliers found: " << _numCorrespondences << endl;
 
    /************************************************************************
     *                            Alignment                                 *
     ************************************************************************/
    for (int k = 0; k < _innerIterations; k++) {      
      Matrix6f H;
      Vector6f b;
      _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
      // _linearizer->setT(_T);
      // _linearizer->update();
      // H = _linearizer->H() + Matrix6f::Identity() * 10.0f;;
      // b = _linearizer->b();


      H = myH;
      b = myb;
      Vector6f dx = H.ldlt().solve(-b);
      Eigen::Isometry3f dT = v2t(dx);
      _T = dT * _T;
      _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
      //cerr << "Hreal: " << endl << H-myH << endl;
      //cerr << "breal: " << endl << b-myb << endl;
    }    
  }
  _T = _sensorOffset * _T;

  {
    cerr << "Destroy" << endl;
    status = destroyContext(context);
    status.toString(buf);
    cerr << "STATUS: " << buf << endl; 
  }
  
}

}
