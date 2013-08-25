#include "aligner.h"
#include <iostream>
#include <sys/time.h>
#include <fstream>

#include "g2o_frontend/basemath/bm_se3.h"

using namespace std;

namespace pwn {
  using namespace boss;

  Aligner::Aligner(int id, IdContext* context): Identifiable(id,context){
    _linearizer = 0;
    _correspondenceFinder = 0;
    _referenceFrame = 0;
    _currentFrame = 0;
    _outerIterations = 0;
    _innerIterations = 0;
    _T = Eigen::Isometry3f::Identity();
    _initialGuess = Eigen::Isometry3f::Identity();
    _referenceSensorOffset = Eigen::Isometry3f::Identity();
    _currentSensorOffset = Eigen::Isometry3f::Identity();
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

    if (! _projector || !_linearizer || !_correspondenceFinder){
      cerr << "I do nothing since you did not set all required algorithms" << endl;
      return;
    }
    // the current points are seen from the frame of the sensor
    _projector->setTransform(_currentSensorOffset);
    _projector->project(_correspondenceFinder->currentIndexImage(),
			_correspondenceFinder->currentDepthImage(),
			_currentFrame->points());
    _T = _initialGuess;
  
    // _correspondenceFinder->currentDepthImage().save("current.pgm", true);
    // _currentFrame->save("current.pwn", 1, true);

    for(int i = 0; i < _outerIterations; i++) {
      /************************************************************************
       *                         Correspondence Computation                   *
       ************************************************************************/

      // compute the indices of the current scene from the point of view of the sensor
      _T.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      _projector->setTransform(_T * _referenceSensorOffset);
      _projector->project(_correspondenceFinder->referenceIndexImage(),
			  _correspondenceFinder->referenceDepthImage(),
			  _referenceFrame->points());
    
      // char buf[1024];
      // sprintf(buf, "reference-%02d.pgm", i);
      // _correspondenceFinder->referenceDepthImage().save(buf, true);
    
      // sprintf(buf, "reference-%02d.pwn", i);
      // _referenceFrame->save(buf, 1, true, _T);
    
      // Correspondences computation.  
      _correspondenceFinder->compute(*_referenceFrame, *_currentFrame, _T.inverse());
      // cerr << "cf, numFound:" << _correspondenceFinder->numCorrespondences() << endl;
    
      /************************************************************************
       *                            Alignment                                 *
       ************************************************************************/
      Eigen::Isometry3f invT = _T.inverse();
      for (int k = 0; k < _innerIterations; k++) {      
	invT.matrix().block<1, 4>(3, 0) << 0.0f, 0.0f, 0.0f, 1.0f;
	Matrix6f H;
	Vector6f b;

	_linearizer->setT(invT);
	_linearizer->update();
	H = _linearizer->H() + Matrix6f::Identity() * 1000.0f;
	b = _linearizer->b();
	// add the priors
	for (size_t j=0; j<_priors.size(); j++){
	  const SE3Prior* prior = _priors[j];
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
      _T = v2t(t2v(_T));
      _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
    }

    gettimeofday(&tvEnd, 0);
    double tStart = tvStart.tv_sec*1000+tvStart.tv_usec*0.001;
    double tEnd = tvEnd.tv_sec*1000+tvEnd.tv_usec*0.001;
    _totalTime = tEnd - tStart;
    _error = _linearizer->error();
    _inliers = _linearizer->inliers();
  }


  void Aligner::serialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::serialize(data,context);
    data.setInt("outerIterations", outerIterations());
    data.setInt("innerIterations", innerIterations());
    _referenceSensorOffset.matrix().toBOSS(data,"referenceSensorOffset");
    _currentSensorOffset.matrix().toBOSS(data,"currentSensorOffset");
    data.setPointer("projector", _projector);
    data.setPointer("linearizer", _linearizer);
    data.setPointer("correspondenceFinder", _correspondenceFinder);
  }

  void Aligner::deserialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::deserialize(data,context);
    cerr << "Aligner:: Deserialize" << endl;
    setOuterIterations(data.getInt("outerIterations"));
    setInnerIterations(data.getInt("innerIterations"));
    _referenceSensorOffset.matrix().fromBOSS(data,"referenceSensorOffset");
    _currentSensorOffset.matrix().fromBOSS(data,"currentSensorOffset");
    data.bindPointer("projector", _tempProjector);
    data.bindPointer("linearizer", _tempLinearizer);
    data.bindPointer("correspondenceFinder", _tempCorrespondenceFinder);
  }

  void Aligner::deserializeComplete(){
    setLinearizer(dynamic_cast<Linearizer*>(_tempLinearizer));
    setCorrespondenceFinder(dynamic_cast<CorrespondenceFinder*>(_tempCorrespondenceFinder));
    setProjector(dynamic_cast<PointProjector*>(_tempProjector));
  }

  BOSS_REGISTER_CLASS(Aligner);

}
