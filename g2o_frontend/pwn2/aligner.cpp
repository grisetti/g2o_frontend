#include "aligner.h"

#include <iostream>
#include <sys/time.h>
#include <fstream>
#include <omp.h>

#include "g2o/stuff/unscented.h"

#include "g2o_frontend/basemath/bm_se3.h"
#include <boost/lexical_cast.hpp>

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
    _minInliers = 100;
    _debug = false;
    _rotationalMinEigenRatio = 50;
    _translationalMinEigenRatio = 50;
    _debug = false;
    _debugPrefix = "";
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

    if (_debugPrefix.length()){
      _correspondenceFinder->currentDepthImage().save((_debugPrefix+"_current.pgm").c_str(), true);
      _currentFrame->save("current.pwn", 1, true);
    }

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
      if(_debugPrefix.length())
	_correspondenceFinder->referenceDepthImage().save( (_debugPrefix+"_"+boost::lexical_cast<std::string>(i)+"_reference.pgm").c_str(), true);
 
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
	H = _linearizer->H() + Matrix6f::Identity();
	b = _linearizer->b();

	//H.setZero();
	//b.setZero();
	H += Matrix6f::Identity() * 1000.0f;
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

    _computeStatistics(_mean, _omega, _translationalEigenRatio, _rotationalEigenRatio);
    if (_rotationalEigenRatio > _rotationalMinEigenRatio || _translationalEigenRatio > _translationalMinEigenRatio) {
      if (_debug) {
	cerr << endl;
	cerr << "************** WARNING SOLUTION MIGHT BE INVALID (eigenratio failure) **************" << endl;
	cerr << "tr: " << _translationalEigenRatio << " rr: " << _rotationalEigenRatio << endl;
	cerr << "************************************************************************************" << endl;
      }
    } 
    else {
      // cout << "************** I FOUND SOLUTION VALID SOLUTION   (eigenratio ok) *******************" << endl;
      // cout << "tr: " << _translationalEigenRatio << " rr: " << _rotationalEigenRatio << endl;
      // cout << "************************************************************************************" << endl;
    }

    if (_debug) {
      cout << "Solution statistics in (t, mq): " << endl;
      cout << "mean: " << _mean.transpose() << endl;
      cout << "Omega: " << endl;
      cout << _omega << endl;
    }

  }

  void Aligner::_computeStatistics(Vector6f& mean, Matrix6f& Omega, 
				  float& translationalRatio, float& rotationalRatio) const {

    typedef g2o::SigmaPoint<Vector6f> SigmaPoint;
    typedef std::vector<SigmaPoint, Eigen::aligned_allocator<SigmaPoint> > SigmaPointVector;
  
    // Output init
    Vector6f b;
    Matrix6f H;
    b.setZero();
    H.setZero();
    translationalRatio = std::numeric_limits<float>::max();
    rotationalRatio = std::numeric_limits<float>::max();

#pragma omp parallel 
    {
#pragma omp critical 
      {
	b += _linearizer->b();
	H += _linearizer->H();
      }
    }

    JacobiSVD<Matrix6f> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Matrix6f localSigma = svd.solve(Matrix6f::Identity());
    SigmaPointVector sigmaPoints;
    Vector6f localMean = Vector6f::Zero();
    g2o::sampleUnscented(sigmaPoints, localMean, localSigma);
  
    Eigen::Isometry3f dT = _T;  // transform from current to reference
    if (_debug) {
      cerr << "##T: " << t2v(dT).transpose() << endl;
      cerr << "##hDet(): " << H.determinant() << endl;
      cerr << "##localH: " <<endl << H << endl;
      cerr << "##localSigma: " <<endl << localSigma << endl;
      cerr << "##localSigma * localH: " << endl << localSigma * H << endl;
    }
    // remap each of the sigma points to their original position
    for (size_t i = 0; i < sigmaPoints.size(); i++) {
      SigmaPoint& p = sigmaPoints[i];
      p._sample = t2v(dT*v2t(p._sample).inverse());
    }
    // reconstruct the gaussian 
    g2o::reconstructGaussian(mean,localSigma, sigmaPoints);

    // compute the information matrix from the covariance
    Omega = localSigma.inverse();
  
    // have a look at the svd of the rotational and the translational part;
    JacobiSVD<Matrix3f> partialSVD;
    partialSVD.compute(Omega.block<3,3>(0,0));
    translationalRatio = partialSVD.singularValues()(0) / partialSVD.singularValues()(2);
    if (_debug) {
      cerr << "##singular values of the Omega_t: " << partialSVD.singularValues().transpose() << endl;
      cerr << "##translational_ratio: " <<  translationalRatio << endl;
    } 
    partialSVD.compute(Omega.block<3,3>(3,3));
    rotationalRatio = partialSVD.singularValues()(0)/partialSVD.singularValues()(2);
    if (_debug) {
      cerr << "##singular values of the Omega_r: " << partialSVD.singularValues().transpose() << endl; 
      cerr << "##rotational_ratio: " << rotationalRatio << endl;
    }
  }

  void Aligner::serialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::serialize(data,context);
    data.setInt("outerIterations", outerIterations());
    data.setInt("innerIterations", innerIterations());
    t2v(_referenceSensorOffset).toBOSS(data,"referenceSensorOffset");
    t2v(_currentSensorOffset).toBOSS(data,"currentSensorOffset");
    data.setPointer("projector", _projector);
    data.setPointer("linearizer", _linearizer);
    data.setPointer("correspondenceFinder", _correspondenceFinder);
  }

  void Aligner::deserialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::deserialize(data,context);
    cerr << "Aligner:: Deserialize" << endl;
    cerr << "getting iterations" << endl;
    setOuterIterations(data.getInt("outerIterations"));
    setInnerIterations(data.getInt("innerIterations"));
    cerr << "getting matrices" << endl;
    Vector6f v;
    v.fromBOSS(data,"referenceSensorOffset");
    _referenceSensorOffset=v2t(v);
    v.fromBOSS(data,"currentSensorOffset");
    _currentSensorOffset=v2t(v);
    cerr << "getting projector" << endl;
    data.getReference("projector").bind(_projector);
    cerr << "getting linearizer" << endl;
    data.getReference("linearizer").bind(_linearizer);
    cerr << "getting correspondenceFinder" << endl;
    data.getReference("correspondenceFinder").bind(_correspondenceFinder);
  }

  void Aligner::deserializeComplete(){
  }

  BOSS_REGISTER_CLASS(Aligner);

}
