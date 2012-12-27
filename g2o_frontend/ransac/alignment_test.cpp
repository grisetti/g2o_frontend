#include "alignment_horn2d.h"
#include "alignment_horn3d.h"
#include "alignment_se2.h"
#include "alignment_se3.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#include <iostream>
#include <cstdlib>
using namespace Eigen;
using namespace g2o;
using namespace g2o_frontend;
using namespace std;

template <typename AlignerType, typename EdgeCorrespondenceType>
bool testAligner(typename AlignerType::TransformType& result, 
		 int nPoints, 
		 const typename AlignerType::TransformType& transform, 
		 const std::vector<double>& scales, 
		 const std::vector<double>& offsets,
		 const std::vector<double>& noises){

  typedef typename AlignerType::PointVertexType PointVertexType;
  typedef typename AlignerType::PointEstimateType PointEstimateType;
  typedef typename AlignerType::TransformType TransformType;

  OptimizableGraph graph;

  CorrespondenceVector correspondences;
  IndexVector indices(nPoints, 0);

  double zeroVec[100];
  std::fill(zeroVec, zeroVec+100, 0);
  for (int i=0; i<nPoints; i++){
    double randPoint[scales.size()];
    double noisePoint[scales.size()];
    for(size_t k=0; k<scales.size(); k++){
      randPoint[k]=scales[k]*drand48()+offsets[k];
      noisePoint[k]=noises[k]*(drand48()-.5);
    }
    PointVertexType* v1=new PointVertexType();
    v1->setEstimateData(randPoint);
    v1->setId(i);
    graph.addVertex(v1);

    double buf1[100];
    double buf2[100];
    PointVertexType* v2 = new PointVertexType();
    v2->setEstimate(transform * v1->estimate());
    v2->setId(i+nPoints);
    graph.addVertex(v2);    
    v2->getEstimateData(buf1);
    v2->oplus(noisePoint);
    v2->updateCache();
    v2->getEstimateData(buf2);
    // for (int q = 0; q<v2->dimension(); q++){
    //   cerr << buf1[q] - buf2[q] << " ";
    // }
    // cerr << endl;
    
    EdgeCorrespondenceType* edge = new EdgeCorrespondenceType(); 
    edge->setMeasurementData(zeroVec);
    edge->vertices()[0]=v1;
    edge->vertices()[1]=v2;
    graph.addEdge(edge);
    Correspondence c(edge,100);
    correspondences.push_back(c);
    indices[i]=i;
  }
  AlignerType aligner;
  return aligner(result,correspondences,indices);
}

int main(int , char** ){
  { // ICP 2D
    cerr << "*************** TEST ICP 2D: *************** " <<endl;

    std::vector<double> scales;
    std::vector<double> offsets;
    std::vector<double> noises;
    for (int i=0; i<2; i++){
      scales.push_back(100);
      offsets.push_back(-50);
      noises.push_back(.5);
    }

    SE2 t0(100, 50, M_PI/4);
    SE2 tresult;
    bool result = testAligner<AlignmentAlgorithmHorn2D, EdgePointXY>(tresult, 100, t0, scales, offsets, noises);
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

  {  // ICP 3D
    cerr << "*************** TEST ICP 3D: *************** " <<endl;
    std::vector<double> scales;
    std::vector<double> offsets;
    std::vector<double> noises;
    for (int i=0; i<3; i++){
      scales.push_back(100);
      offsets.push_back(-50);
      noises.push_back(.5);
    }

    Vector6d _t;
    _t << 100, 50, 20, .3, .3, .3;
    Isometry3d t0=g2o::internal::fromVectorMQT(_t);
    Isometry3d tresult;
    bool result = testAligner<AlignmentAlgorithmHorn3D, EdgePointXYZ>(tresult, 100, t0, scales, offsets, noises);
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

  { // SE2
    cerr << "*************** TEST SE2  *************** " <<endl;
    std::vector<double> scales;
    std::vector<double> offsets;
    std::vector<double> noises;
    // translational part;
    for (int i=0; i<2; i++){
      scales.push_back(100);
      offsets.push_back(-50);
      noises.push_back(.01);
    }
    // rotational part
    scales.push_back(2*M_PI);
    offsets.push_back(-M_PI);
    noises.push_back(.01);

    SE2 t0(100, 50, M_PI/4);
    SE2 tresult;
    bool result = testAligner<AlignmentAlgorithmSE2, EdgeSE2>(tresult, 100, t0, scales, offsets, noises);
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

  { // SE2
    cerr << "*************** TEST SE3  *************** " <<endl;
    std::vector<double> scales;
    std::vector<double> offsets;
    std::vector<double> noises;
    // translational part;
    for (int i=0; i<3; i++){
      scales.push_back(100);
      offsets.push_back(50);
      noises.push_back(.0);
    }
    // rotational part
    for (int i=0; i<3; i++){
      scales.push_back(.1);
      offsets.push_back(-.05);
      noises.push_back(.0);
    }

    Vector6d _t;
    _t << 100, 50, 20, .3, .2, .1;
    Isometry3d t0=g2o::internal::fromVectorMQT(_t);

    Isometry3d tresult;
    bool result = testAligner<AlignmentAlgorithmSE3, EdgeSE3>(tresult, 10000, t0, scales, offsets, noises);
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

}
