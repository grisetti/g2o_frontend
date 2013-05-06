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
  int _rows = _correspondenceGenerator.rows();
  int _cols = _correspondenceGenerator.cols();
  
  if (! context){
    status = createContext(&context, _rows*_cols, _rows*_cols, _rows, _cols);
    status.toString(buf);
    cerr << "STATUS: " << buf << endl; 
  }

  _T = _initialGuess;
  float referenceCurvatures[_referenceScene.stats().size()];
  float currentCurvatures[_currentScene.stats().size()];
  for (int i=0; i<_referenceScene.stats().size(); i++)
    referenceCurvatures[i]=_referenceScene.stats().at(i).curvature();
  for (int i=0; i<_currentScene.stats().size(); i++)
    currentCurvatures[i]=_currentScene.stats().at(i).curvature();
  {

    PinholePointProjector *pprojector = (PinholePointProjector *) _projector;
    cerr << "INIT" << endl;
    cerr << "camera amtrix: " << pprojector->cameraMatrix() << endl;
    status = initComputation(context,
			     &(pprojector->cameraMatrix().coeffRef(0,0)),
			     &(_referenceScene.points().at(0).coeffRef(0)),
			     &(_referenceScene.normals().at(0).coeffRef(0)),
			     referenceCurvatures,
			     _referenceScene.points().size(),
			     &(_currentScene.points().at(0).coeffRef(0)),
			     &(_currentScene.normals().at(0).coeffRef(0)),
			     currentCurvatures,
			     &(_currentScene.pointOmegas().at(0).coeffRef(0,0)),
			     &(_currentScene.normalOmegas().at(0).coeffRef(0,0)),
			     _currentScene.points().size());
    
    status.toString(buf);
    cerr << "STATUS: " << buf << endl; 
  }
  for(int i = 0; i < _outerIterations; i++) {
    _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
 
    _projector->setTransform(_T.inverse());
    // _projector->project(_correspondenceGenerator.referenceIndexImage(),
    // 		       _correspondenceGenerator.referenceDepthImage(),
    // 		       _referenceScene.points());
    
    // Correspondences computation.    
    //_correspondenceGenerator.compute(_referenceScene, _currentScene, _T);

    cout << " done." << endl;
    cout << "********************* Iteration " << i << " *********************" << endl;
    
    
    Matrix6f H;
    Vector6f b;
    cerr << "ITERATE" << endl;
    status = simpleIteration(context, &(_T.matrix().coeffRef(0,0)));
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
    H.block<3,3>(0,0) = Htt.block<3,3>(0,0);
    H.block<3,3>(0,3) = Htr.block<3,3>(0,0);
    H.block<3,3>(3,3) = Hrr.block<3,3>(0,0);
    H.block<3,3>(3,0) = H.block<3,3>(0,3).transpose();
    b.block<3,1>(0,0) = bt.block<3,1>(0,0);
    b.block<3,1>(3,0) = br.block<3,1>(0,0);

    int acc = 0;
    //_numCorrespondences = _correspondenceGenerator.numCorrespondences();
    //cout << "# inliers found: " << _numCorrespondences << endl;
 
    Vector6f dx = H.ldlt().solve(-b);
    Eigen::Isometry3f dT = v2t(dx);
    _T = dT * _T;
    _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
    //cerr << "Hreal: " << endl << H-myH << endl;
    //cerr << "breal: " << endl << b-myb << endl;
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
