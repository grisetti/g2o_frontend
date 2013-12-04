#include "cualigner.h"
#include <iostream>
#include "cudaaligner.h"
#include <sys/time.h>
#include "g2o_frontend/pwn_core/pinholepointprojector.h"

namespace pwn {
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
  struct timeval tvStart, tvEnd;
  gettimeofday(&tvStart,0);

  _error = 0;
  _inliers = 0;
  _totalTime = 0;
  AlignerStatus status;
  char buf[1024];
  _T = _initialGuess;
  float referenceCurvatures[_referenceFrame->stats().size()];
  float currentCurvatures[_currentFrame->stats().size()];
  for (size_t i=0; i<_referenceFrame->stats().size(); i++)
    referenceCurvatures[i]=_referenceFrame->stats().at(i).curvature();
  for (size_t i=0; i<_currentFrame->stats().size(); i++)
    currentCurvatures[i]=_currentFrame->stats().at(i).curvature();

  Eigen::Isometry3f inverseSensorOffset= _referenceSensorOffset.inverse();
  inverseSensorOffset.matrix().row(3)<< 0,0,0,1;
  PinholePointProjector *pprojector = (PinholePointProjector *) _projector;
  //cerr << "initializing context computation" << endl;
  //cerr << "camera: " << pprojector->cameraMatrix() << endl;
  //cerr << "inverseSensorOffset: " << endl << inverseSensorOffset.matrix() << endl;
  status = initComputation(_context,
			   &(pprojector->cameraMatrix().coeffRef(0,0)),
			   &(inverseSensorOffset.matrix().coeffRef(0,0)),
			   &(_referenceFrame->points().at(0).coeffRef(0)),
			   &(_referenceFrame->normals().at(0).coeffRef(0)),
			   referenceCurvatures,
			   _referenceFrame->points().size(),
			   &(_currentFrame->points().at(0).coeffRef(0)),
			   &(_currentFrame->normals().at(0).coeffRef(0)),
			   currentCurvatures,
			   &(_currentFrame->pointInformationMatrix().at(0).coeffRef(0,0)),
			   &(_currentFrame->normalInformationMatrix().at(0).coeffRef(0,0)),
			   _currentFrame->points().size());
  
  status.toString(buf);
  //cerr << "STATUS: " << buf << endl; 
    
  Eigen::Isometry3f invT = _T.inverse();
  for(int i = 0; i < _outerIterations; i++) {
    //cout << " done." << endl;
    //cout << "********************* Iteration " << i << " *********************" << endl;
    invT.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
    
    
    //cerr << "ITERATE" << endl;
    status = simpleIteration(_context, &(invT.matrix().coeffRef(0,0)), &_inliers, &_error);
    status.toString(buf);
    //cerr << "STATUS: " << buf << endl; 
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


    for (size_t j=0; j<_priors.size(); j++){
      const SE3Prior* prior = _priors[j];
      Vector6f priorError = prior->error(invT);
      Matrix6f priorJacobian = prior->jacobian(invT);
      Matrix6f priorInformationRemapped = prior->errorInformation(invT);
      
      Matrix6f Hp = priorJacobian.transpose()*priorInformationRemapped*priorJacobian;
      Vector6f bp = priorJacobian.transpose()*priorInformationRemapped*priorError;
      //cerr << "prior" << endl;
      //cerr << "Hp: " << endl;
      //cerr << Hp << endl;
      //cerr << "bp: " << endl;
      //cerr << bp << endl;
      
      
      H += Hp;
      b += bp;
    }
    //int acc = 0;
    //_numCorrespondences = _correspondenceGenerator.numCorrespondences();
    //cout << "# inliers found: " << _numCorrespondences << endl;
    H += Matrix6f::Identity() * 10.0f;

    
    Vector6f dx = H.ldlt().solve(-b);
    Eigen::Isometry3f dT = v2t(dx);
    //cerr << "dt = " << t2v(dT).transpose() << endl;
					
    invT =  dT * invT ;
    //cerr << "Hreal: " << endl << H-myH << endl;
    //cerr << "breal: " << endl << b-myb << endl;
  }
  gettimeofday(&tvEnd,0);

  _T = invT.inverse();
  _totalTime = (tvEnd.tv_sec*1000+tvEnd.tv_usec*0.001) - (tvStart.tv_sec*1000+tvStart.tv_usec*0.001) ;
}

}
