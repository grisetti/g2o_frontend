#include "cualigner.h"
#include <iostream>
#include "cudaaligner.h"
#include "g2o_frontend/pwn/pinholepointprojector.h"

namespace CudaAligner {
  using namespace std;

  CuAligner::CuAligner() {
    int _rows = 640;
    int _cols = 480;
    AlignerStatus status;
    cerr << "initializing cuda context" << endl;
    status = createContext(&_context, _rows*_cols*4, _rows*_cols*4, _rows, _cols);
    char buf[1024];
    status.toString(buf);
    cerr << "STATUS: " << buf << endl; 
  }

  CuAligner::~CuAligner() {
    AlignerStatus status;

    cerr << "destroying cuda context" << endl;
    status = destroyContext(_context);
    char buf[1024];
    status.toString(buf);
    cerr << "STATUS: " << buf << endl; 

  }

void CuAligner::align() {
  AlignerStatus status;
  char buf[1024];
  _T = _initialGuess;
  float referenceCurvatures[_referenceScene->stats().size()];
  float currentCurvatures[_currentScene->stats().size()];
  for (int i=0; i<_referenceScene->stats().size(); i++)
    referenceCurvatures[i]=_referenceScene->stats().at(i).curvature();
  for (int i=0; i<_currentScene->stats().size(); i++)
    currentCurvatures[i]=_currentScene->stats().at(i).curvature();

  
  PinholePointProjector *pprojector = (PinholePointProjector *) _projector;
  cerr << "initializing context computation" << endl;
  cerr << "camera: " << pprojector->cameraMatrix() << endl;
  status = initComputation(_context,
			   &(pprojector->cameraMatrix().coeffRef(0,0)),
			   &(_referenceScene->points().at(0).coeffRef(0)),
			   &(_referenceScene->normals().at(0).coeffRef(0)),
			   referenceCurvatures,
			   _referenceScene->points().size(),
			   &(_currentScene->points().at(0).coeffRef(0)),
			   &(_currentScene->normals().at(0).coeffRef(0)),
			   currentCurvatures,
			   &(_currentScene->pointOmegas().at(0).coeffRef(0,0)),
			   &(_currentScene->normalOmegas().at(0).coeffRef(0,0)),
			   _currentScene->points().size());
  
  status.toString(buf);
  cerr << "STATUS: " << buf << endl; 
    
  Eigen::Isometry3f invT = _T.inverse();
  for(int i = 0; i < _outerIterations; i++) {
    cout << " done." << endl;
    cout << "********************* Iteration " << i << " *********************" << endl;
    invT.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
    
    
    cerr << "ITERATE" << endl;
    status = simpleIteration(_context, &(invT.matrix().coeffRef(0,0)));
    status.toString(buf);
    cerr << "STATUS: " << buf << endl; 
    Eigen::Matrix4f Htt, Htr, Hrr;
    Eigen::Vector4f bt, br;
    getHb(_context,
	  &(Htt.coeffRef(0,0)), 
	  &(Htr.coeffRef(0,0)), 
	  &(Hrr.coeffRef(0,0)), 
	  &(bt.coeffRef(0,0)), 
	  &(br.coeffRef(0,0)));
    Matrix6f H;
    Vector6f b;
    H.block<3,3>(0,0) = Htt.block<3,3>(0,0);
    H.block<3,3>(0,3) = Htr.block<3,3>(0,0);
    H.block<3,3>(3,3) = Hrr.block<3,3>(0,0);
    H.block<3,3>(3,0) = H.block<3,3>(0,3).transpose();
    b.block<3,1>(0,0) = bt.block<3,1>(0,0);
    b.block<3,1>(3,0) = br.block<3,1>(0,0);

    int acc = 0;
    //_numCorrespondences = _correspondenceGenerator.numCorrespondences();
    //cout << "# inliers found: " << _numCorrespondences << endl;
    H += Matrix6f::Identity() * 10.0f;
    
    Vector6f dx = H.ldlt().solve(-b);
    Eigen::Isometry3f dT = v2t(dx);
    cerr << "dt = " << t2v(dT).transpose() << endl;
					
    invT =  dT * invT ;
    //cerr << "Hreal: " << endl << H-myH << endl;
    //cerr << "breal: " << endl << b-myb << endl;
  }
  _T = invT.inverse();
  _T = _sensorOffset * _T;
  cerr << "T = " << t2v(_T).transpose() << endl;
}

}
