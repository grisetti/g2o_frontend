#include "alignment_horn2d.h"
#include "alignment_horn3d.h"
#include "alignment_se2.h"
#include "alignment_se3.h"
#include "alignment_line3d_linear.h"
#include "alignment_line2d_linear.h"
#include "g2o_frontend/basemath/bm_se2.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
using namespace Eigen;
using namespace g2o;
using namespace g2o_frontend;
using namespace std;
using namespace Slam3dAddons;


template <typename TypeDomain_, int dimension_>
struct EuclideanMapping{
  typedef TypeDomain_ TypeDomain;
  typedef typename Eigen::Matrix<double, dimension_, 1> VectorType;
  virtual int dimension() const {return dimension_;}
  virtual TypeDomain fromVector(const VectorType& v) const =  0;
  virtual VectorType toVector(const TypeDomain& t) const = 0;
};

template <int dimension_>
struct VectorMapping : public EuclideanMapping<Eigen::Matrix<double, dimension_, 1>, dimension_>{
  typedef typename EuclideanMapping<Eigen::Matrix<double, dimension_, 1>, dimension_>::TypeDomain TypeDomain;
  typedef typename EuclideanMapping<Eigen::Matrix<double, dimension_, 1>, dimension_>::VectorType VectorType;
  virtual TypeDomain fromVector(const VectorType& v) const {return v;}
  virtual VectorType toVector(const TypeDomain& t) const {return t;}
};

struct SE2Mapping: public EuclideanMapping<SE2, 3>{
  typedef typename EuclideanMapping<SE2, 3>::TypeDomain TypeDomain;
  typedef typename EuclideanMapping<SE2, 3>::VectorType VectorType;
  virtual TypeDomain fromVector(const VectorType& v) const {
    SE2 t;
    t.fromVector(v);
    return t;
  }
  virtual VectorType toVector(const TypeDomain& t) const {
    return t.toVector();
  }
};

struct SE3Mapping: public EuclideanMapping<Isometry3d,6>{
  typedef typename EuclideanMapping<Isometry3d, 6>::TypeDomain TypeDomain;
  typedef typename EuclideanMapping<Isometry3d, 6>::VectorType VectorType;
  virtual TypeDomain fromVector(const VectorType& v) const {
    return g2o::internal::fromVectorMQT(v);
  }
  virtual VectorType toVector(const TypeDomain& t) const {
    return g2o::internal::toVectorMQT(t);
  }
};

struct Line3DMapping: public EuclideanMapping<Line3D,6>{
  typedef typename EuclideanMapping<Line3D, 6>::TypeDomain TypeDomain;
  typedef typename EuclideanMapping<Line3D, 6>::VectorType VectorType;
  virtual TypeDomain fromVector(const VectorType& v) const {
    Line3D l(v);
    l.normalize();
    return l;
  }
  virtual VectorType toVector(const TypeDomain& t) const {
    return (VectorType)t;
  }
};

struct Line2DMapping:public EuclideanMapping<Line2D, 2>{
	typedef typename EuclideanMapping<Line2D, 2>::TypeDomain TypeDomain;
	typedef typename EuclideanMapping<Line2D, 2>::VectorType VectorType;
// 	virtual TypeDomain fromVector(const VectorType& v) const {return v;}
// 	virtual VectorType toVector(const TypeDomain& t) const {return t;}
	virtual TypeDomain fromVector(const Vector2d& v) const {
		return Line2D(v);
	}
  virtual VectorType toVector(const TypeDomain& t) const {return Vector2d(t);}
};

template <typename MappingType, typename RansacType, typename EdgeCorrespondenceType>
bool testRansac(typename RansacType::TransformType& result, 
		int nPoints, 
		const typename RansacType::TransformType& transform,
		const std::vector<double>& scales, 
		const std::vector<double>& offsets,
		const std::vector<double>& noises,
		const std::vector<double>& omegas,
		CorrespondenceValidatorPtrVector& validators,
		double outlierFraction=0.2,
		bool debug = false){
  
  typedef typename RansacType::AlignerType AlignerType;
  typedef typename RansacType::PointVertexType PointVertexType;
  typedef typename RansacType::PointEstimateType PointEstimateType;
  typedef typename RansacType::TransformType TransformType;
  typedef typename MappingType::VectorType VectorType;

  OptimizableGraph graph;

  CorrespondenceVector correspondences;
  IndexVector indices(nPoints, 0);
  MappingType mapping;
  assert(scales.size()==mapping.dimension());
  double zeroVec[100];
  std::fill(zeroVec, zeroVec+100, 0); 
  ofstream os1;
  ofstream os2;
  if (debug) {
    os1.open("L1.dat");
    os2.open("L2.dat");
  }
  for (int i=0; i<nPoints; i++){
    VectorType randPoint;
    VectorType randPoint2;
    VectorType noisePoint;
    for(size_t k=0; k<scales.size(); k++){
      randPoint[k]=scales[k]*drand48()+offsets[k];
      randPoint2[k]=scales[k]*drand48()+offsets[k];
      noisePoint[k]=noises[k]*(drand48()-.5);
    }
    PointVertexType* v1=new PointVertexType();
    v1->setEstimate(mapping.fromVector(randPoint));
    v1->setId(i);
    graph.addVertex(v1);

    PointVertexType* v2 = new PointVertexType();
    if (drand48()>outlierFraction) {
      PointEstimateType v2est=transform * v1->estimate();
      VectorType v2noise=mapping.toVector(v2est)+noisePoint;
      v2->setEstimate(mapping.fromVector(v2noise));
    } else {
      PointEstimateType v2est=mapping.fromVector(randPoint2);
      v2->setEstimate(v2est);
    }
    v2->setId(i+nPoints);
    graph.addVertex(v2);    
    v2->updateCache();

    if (debug) {
      VectorType e1=mapping.toVector(v1->estimate());
      VectorType e2=mapping.toVector(v2->estimate());
      for (int i =0; i<mapping.dimension(); i++){
	os1 << e1[i] << " " ;
	os2 << e2[i] << " " ;
      }

      os1 << endl;
      os2 << endl;
    }
    EdgeCorrespondenceType* edge = new EdgeCorrespondenceType(); 
    edge->setMeasurementData(zeroVec);
    typename EdgeCorrespondenceType::InformationType info =EdgeCorrespondenceType::InformationType::Identity();
    for (size_t k=0; i<edge->dimension() && k<omegas.size(); k++){
      info(k,k) = omegas[k];
    }
    edge->setInformation(info);
    edge->vertices()[0]=v1;
    edge->vertices()[1]=v2;
    graph.addEdge(edge);
    Correspondence c(edge,100);
    correspondences.push_back(c);
    indices[i]=i;
  }
  RansacType ransac;
  ransac.correspondenceValidators()=validators;
  ransac.setCorrespondences(correspondences);
  ransac.setMaxIterations(1000);
  ransac.setInlierErrorThreshold(1.);
  ransac.setInlierStopFraction(0.5);
  std::vector<int> tmp;
  return ransac(result,tmp, debug);
}

int main(int , char** ){
#if 0
		{// ICP 2D
    cerr << "*************** TEST ICP 2D: *************** " <<endl;

    std::vector<double> scales;
    std::vector<double> offsets;
    std::vector<double> noises;
    std::vector<double> omegas;
    for (int i=0; i<2; i++){
      scales.push_back(100);
      offsets.push_back(-50);
      noises.push_back(.5);
      omegas.push_back(.25);
    }

    SE2 t0(100, 50, M_PI/4);
    SE2 tresult;
    CorrespondenceValidatorPtrVector validators;
    DistanceCorrespondenceValidator<VertexPointXY>* val1= new DistanceCorrespondenceValidator<VertexPointXY>(3);
    val1->setIntraFrameDistanceDifference(.5);
    val1->setIntraFrameMinimalDistance(5);
    validators.push_back(val1);
    bool result = testRansac<VectorMapping<2>, RansacHorn2D, EdgePointXY>(tresult, 50, t0, 
									  scales, offsets, noises, omegas, 
									  validators,
									  0.5);
    if (result){
      cerr << "ground truth: " <<endl;
      cerr << t0.toVector() << endl;
      cerr << "transform found: " <<endl;
      cerr << tresult.toVector() << endl;
      cerr << "transform error: " << endl;
      cerr << (t0*tresult).toVector() << endl;
    } else {
      cerr << "unable to find a transform" << endl;
    }
  }
#endif
#if 0
  {  // ICP 3D
    cerr << "*************** TEST ICP 3D: *************** " <<endl;
    std::vector<double> scales;
    std::vector<double> offsets;
    std::vector<double> noises;
    std::vector<double> omegas;
    for (int i=0; i<3; i++){
      scales.push_back(100);
      offsets.push_back(-50);
      noises.push_back(.5);
      omegas.push_back(.25);
    }

    Vector6d _t;
    _t << 100, 50, 20, .3, .3, .3;
    Isometry3d t0=g2o::internal::fromVectorMQT(_t);
    Isometry3d tresult;
    CorrespondenceValidatorPtrVector validators;
    DistanceCorrespondenceValidator<VertexPointXYZ>* val1=new DistanceCorrespondenceValidator<VertexPointXYZ>(4);
    val1->setIntraFrameDistanceDifference(.5);
    val1->setIntraFrameMinimalDistance(5);
    validators.push_back(val1);
    bool result = testRansac<VectorMapping<3>, RansacHorn3D, EdgePointXYZ>(tresult, 100, t0, 
									   scales, offsets, noises, omegas, 
									   validators,
									   0.3);
    if (result){
      cerr << "ground truth: " <<endl;
      cerr << g2o::internal::toVectorMQT(t0)  << endl;
      cerr << "transform found: " <<endl;
      cerr << g2o::internal::toVectorMQT(tresult) << endl;
      cerr << "transform error: " << endl;
      cerr << g2o::internal::toVectorMQT((t0*tresult)) << endl;
    } else {
      cerr << "unable to find a transform" << endl;
    }
  }
#endif
#if 0 
  { // SE2
    cerr << "*************** TEST SE2  *************** " <<endl;
    std::vector<double> scales;
    std::vector<double> offsets;
    std::vector<double> noises;
    std::vector<double> omegas;
    // translational part;
    for (int i=0; i<2; i++){
      scales.push_back(100);
      offsets.push_back(-50);
      noises.push_back(.1);
      omegas.push_back(.25);
    }
    // rotational part
    scales.push_back((2*M_PI)*M_PI);
    offsets.push_back(-M_PI);
    noises.push_back(.1);
    omegas.push_back(1);

    SE2 t0(100, 50, M_PI/4);
    SE2 tresult;
    CorrespondenceValidatorPtrVector validators;
    bool result = testRansac<SE2Mapping, RansacSE2, EdgeSE2>(tresult, 100, t0, 
							     scales, offsets, noises, omegas, 
							     validators,
							     0.3);
    if (result){
      cerr << "ground truth: " <<endl;
      cerr << t0.toVector() << endl;
      cerr << "transform found: " <<endl;
      cerr << tresult.toVector() << endl;
      cerr << "transform error: " << endl;
      cerr << (t0*tresult).toVector() << endl;
    } else {
      cerr << "unable to find a transform" << endl;
    }
  }
#endif
#if 0
  { // SE3
    cerr << "*************** TEST SE3  *************** " <<endl;
    std::vector<double> scales;
    std::vector<double> offsets;
    std::vector<double> noises;
    std::vector<double> omegas;
    // translational part;
    for (int i=0; i<3; i++){
      scales.push_back(100);
      offsets.push_back(50);
      noises.push_back(1.);
    }
    // rotational part
    for (int i=0; i<3; i++){
      scales.push_back(.2);
      offsets.push_back(-.1);
      noises.push_back(.01);
      omegas.push_back(.1);
    }

    Vector6d _t;
    _t << 100, 50, 20, .3, .2, .1;
    Isometry3d t0=g2o::internal::fromVectorMQT(_t);

    Isometry3d tresult;
    CorrespondenceValidatorPtrVector validators;
    bool result = testRansac<SE3Mapping, RansacSE3, EdgeSE3>(tresult, 100, t0, 
							     scales, offsets, noises, omegas, 
							     validators,
							     0.4);
    if (result){
      cerr << "ground truth: " <<endl;
      cerr << g2o::internal::toVectorMQT(t0)  << endl;
      cerr << "transform found: " <<endl;
      cerr << g2o::internal::toVectorMQT(tresult) << endl;
      cerr << "transform error: " << endl;
      cerr << g2o::internal::toVectorMQT((t0*tresult)) << endl;
    } else {
      cerr << "unable to find a transform" << endl;
    }
  }
#endif
#if 0
  { // Line3D
    cerr << "*************** TEST Line3D  *************** " <<endl;
    std::vector<double> scales;
    std::vector<double> offsets;
    std::vector<double> noises;
    std::vector<double> omegas;
    // translational part;
    for (int i=0; i<3; i++){
      scales.push_back(100);
      offsets.push_back(50);
      noises.push_back(0.1);
      omegas.push_back(.001);
    }
    // rotational part
    for (int i=0; i<3; i++){
      scales.push_back(2);
      offsets.push_back(-1);
      noises.push_back(0.1);
      omegas.push_back(.001);
    }

    Vector6d _t;
    _t << 100, 50, 20, .3, .2, .1;
    Isometry3d t0=g2o::internal::fromVectorMQT(_t);

    Isometry3d tresult;
    CorrespondenceValidatorPtrVector validators;
    bool result = testRansac<Line3DMapping, RansacLine3DLinear, EdgeLine3D>(tresult, 100, t0, 
									    scales, offsets, noises, omegas, 
									    validators,
									    0.4);
    if (result){
      cerr << "ground truth: " <<endl;
      cerr << g2o::internal::toVectorMQT(t0)  << endl;
      cerr << "transform found: " <<endl;
      cerr << g2o::internal::toVectorMQT(tresult) << endl;
      cerr << "transform error: " << endl;
      cerr << g2o::internal::toVectorMQT((t0*tresult)) << endl;
    } else {
      cerr << "unable to find a transform" << endl;
    }
  }
#endif

  { // Line2d
    cerr << "*************** TEST Line2D  *************** " <<endl;
    std::vector<double> scales;
    std::vector<double> offsets;
    std::vector<double> noises;
    std::vector<double> omegas;
    // rotational part
    for (int i=0; i<1; i++){
      scales.push_back(2);
      offsets.push_back(-1);
      noises.push_back(0.);
      omegas.push_back(1000);
    }
    // translational part;
    for (int i=0; i<1; i++){
      scales.push_back(100);
      offsets.push_back(50);
      noises.push_back(0.);
      omegas.push_back(1000);
    }
    
    Vector3d _t(2, 5, .3);
    Isometry2d _t0=v2t_2d(_t);
    cerr << "ground truth vector: " <<endl;
    cerr << t2v_2d(_t0) << endl;
    cerr << "ground truth: " <<endl;
    cerr << _t0.matrix() << endl;
    SE2 tresult;
    SE2 t0(_t0);
    CorrespondenceValidatorPtrVector validators;
    bool result = testRansac<Line2DMapping, RansacLine2DLinear, EdgeLine2D>(tresult, 100, t0, 
									    scales, offsets, noises, omegas, 
									    validators,
									    0.2);
   if (result){
// 			cerr << "ground truth vector: " <<endl;
// 			cerr << t2v_2d(_t0) << endl;
// 			cerr << "ground truth: " <<endl;
// 			cerr << _t0.matrix() << endl;
			cerr << "***********FOUND!***********" << endl;
			Isometry2d res = tresult.toIsometry();
			cerr << "transform found vector: " <<endl;
			cerr << t2v_2d(res) << endl;
			cerr << "transform found: " <<endl;
			cerr << res.matrix() << endl;
			cerr << "transform error vector: " << endl;
			cerr << t2v_2d(_t0*res) << endl;
			cerr << "transform error: " << endl;
			cerr << (_t0*res).matrix() << endl;
   } else {
     cerr << "unable to find a transform" << endl;
   }
  }
  
}
