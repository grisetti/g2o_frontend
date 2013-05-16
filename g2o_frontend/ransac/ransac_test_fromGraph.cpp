#include "alignment_horn2d.h"
#include "alignment_horn3d.h"
#include "alignment_se2.h"
#include "alignment_se3.h"
#include "alignment_line3d_linear.h"
#include "alignment_line2d_linear.h"
#include "g2o_frontend/basemath/bm_se2.h"
#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdlib>
#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"
using namespace Eigen;
using namespace g2o;
using namespace g2o_frontend;
using namespace std;
using namespace Slam3dAddons;

LaserRobotData l;

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
bool testRansac_fromGraph(typename RansacType::TransformType& result,
		string filename,
		typename RansacType::TransformType& transform,
		const std::vector<double>& omegas,
		CorrespondenceValidatorPtrVector& validators,
		double outlierFraction = 0.2,
		bool debug = false){
  
    typedef typename RansacType::AlignerType AlignerType;
    typedef typename RansacType::PointVertexType PointVertexType;
    typedef typename RansacType::PointEstimateType PointEstimateType;
    typedef typename RansacType::TransformType TransformType;
    typedef typename MappingType::VectorType VectorType;

    OptimizableGraph graph;
    OptimizableGraph inputGraph;
    inputGraph.load(filename.c_str());

    //first vertex to be aligned
    OptimizableGraph::Vertex* v1_ = inputGraph.vertex(0);
    VertexSE2* v1 = dynamic_cast<VertexSE2*>(v1_);
    if(!v1) {
	cerr << " !!! Error in reading the first vertex!" << endl;
	return false;
    }
    OptimizableGraph::EdgeSet es1 = v1->edges();
  
    //second vertex to be aligned
    OptimizableGraph::Vertex* v2_ = inputGraph.vertex(1);
    VertexSE2* v2 = dynamic_cast<VertexSE2*>(v2_);
    if(!v2) {
	cerr << " !!! Error in reading the second vertex!" << endl;
	return false;
    }
    OptimizableGraph::EdgeSet es2 = v2->edges();
    
    //saving the ground truth transform
    RansacLine2DLinear::TransformType t0;
    for (OptimizableGraph::EdgeSet::iterator itv = es1.begin(); itv != es1.end(); itv++)
    {
	EdgeSE2* es = dynamic_cast<EdgeSE2*>(*itv);
	if (!es)
	    continue;
	if(v1->id() == es->vertices()[0]->id()){
	    cerr << "salvo odometria" << endl;
	    t0 = es->measurement();
	}
    }
    cerr << "ground truth " << t0.toIsometry().matrix() << endl;
    transform = t0;
    
    //computing the total number of correspondances for ransac
    int numLinee1 = 0, numLinee2 = 0;
    for (OptimizableGraph::EdgeSet::iterator itv1 = es1.begin(); itv1 != es1.end(); itv1++) {
	EdgeSE2Line2D* el1 = dynamic_cast<EdgeSE2Line2D*>(*itv1);
	if (el1)
	    numLinee1++;
    }
    for (OptimizableGraph::EdgeSet::iterator itv2 = es2.begin(); itv2 != es2.end(); itv2++) {
	EdgeSE2Line2D* el2 = dynamic_cast<EdgeSE2Line2D*>(*itv2);
	if (el2)
	    numLinee2++;
    }
    int totCorr = numLinee1*numLinee2;
    cout << "numLinee1: " << numLinee1 << ", numLinee2: " << numLinee2 << ", totCorr: " << totCorr << endl;
    
    CorrespondenceVector correspondences;
    IndexVector indices(totCorr, 0);
//   MappingType mapping;
//   assert(scales.size()==mapping.dimension());
    double zeroVec[100];
    std::fill(zeroVec, zeroVec+100, 0); 
    
    
    //TODO  FIND CORRESPONDENCES AND ADD ONLY THE VERTEXES OF THE CORRESPONDANCES TO THE GRAPH
    int i = 0, k = 0;
    int j = numLinee1;
    for (OptimizableGraph::EdgeSet::iterator itv = es1.begin(); itv != es1.end() && k < numLinee1 && i < totCorr; itv++) 
    {
	PointVertexType* vl1 = new PointVertexType();
	
	EdgeSE2Line2D* el = dynamic_cast<EdgeSE2Line2D*>(*itv);
	if (!el)
	    continue;
	const VertexLine2D* vl1_ = dynamic_cast<const VertexLine2D*>(el->vertices()[1]);

	const Line2D theLine = vl1_->estimate();
	cout << "vl1_ estimate1:\n" << theLine << endl;
	
	vl1->setEstimate(theLine);
	vl1->setId(k);
	graph.addVertex(vl1);
    
	for (OptimizableGraph::EdgeSet::iterator itv2 = es2.begin(); itv2 != es2.end() && j < totCorr; itv2++)
	{
	    cerr << "i:" << i << ", k: " << k << ", j: " << j << endl;
	    PointVertexType* vl2 = new PointVertexType();
	
	    EdgeSE2Line2D* el2 = dynamic_cast<EdgeSE2Line2D*>(*itv2);
	    if (!el2)
		continue;
	    VertexLine2D* vl2_ = dynamic_cast<VertexLine2D*>(el2->vertices()[1]);
	    const Line2D theLine2 = vl2_->estimate();
	    cout << "vl1_ estimate2:\n" << theLine2 << endl;
	
	    vl2->setEstimate(theLine2);
	    vl2->setId(j);
	    graph.addVertex(vl2);
	    EdgeCorrespondenceType* edge = new EdgeCorrespondenceType(); 
	    edge->setMeasurementData(zeroVec);
	    typename EdgeCorrespondenceType::InformationType info = EdgeCorrespondenceType::InformationType::Identity();
	    for (int h=0; i<edge->dimension() && h<omegas.size(); h++)
	    {
		info(h,h) = omegas[h];
	    }

	    edge->setInformation(info);
	    edge->vertices()[0]=vl1;
	    edge->vertices()[1]=vl2;

	    graph.addEdge(edge);
	    Correspondence c(edge,100);
	    correspondences.push_back(c);
	    indices[i]=i;
	    
	    //updating global and second local indexes
	    j++;
	    i++;
	}
	//updating local index
	k++;
    }
    cerr << "correspondances dim: " << correspondences.size() << "must be = " << i << endl;
    RansacType ransac;
    ransac.correspondenceValidators()=validators;
    ransac.setCorrespondences(correspondences);
    ransac.setMaxIterations(1000);
    ransac.setInlierErrorThreshold(1.);
    ransac.setInlierStopFraction(.5);
    std::vector<double> err;
    std::vector<int> inliers;
    bool resultRansac = ransac(result,inliers,debug);
    if(resultRansac){
	err = ransac.errors();
	cout << "Erros size: " << err.size() << endl;
	for (int h = 0; h < (int)err.size(); h++){
	    double erri = err[h];
	    cout << "error of " << h << "-th correspondance: " << erri << endl;
	}
	cout << "Inliers: (size is " << inliers.size() << ")" << endl;
	for(int g = 0; g < (int)inliers.size(); g++){
	    double index = inliers[g];
	    cout << "Inliers: " << index << "-th correspondance: " << " with error: " << err[index] << endl;
	}
    }
    
    return resultRansac;
}

int main(int argc, char** argv){
  { // Line2d
    cerr << "*************** TEST Line2D  *************** " <<endl;
//     std::vector<double> scales;
//     std::vector<double> offsets;
//     std::vector<double> noises;
    std::vector<double> omegas;
    // rotational part
    for (int i=0; i<1; i++){
//       scales.push_back(2);
//       offsets.push_back(-1);
//       noises.push_back(0.);
      omegas.push_back(100);
    }
    // translational part;
    for (int i=0; i<1; i++){
//       scales.push_back(100);
//       offsets.push_back(50);
//       noises.push_back(0.);
      omegas.push_back(1);
    }
    
    string filename;
    g2o::CommandArgs arg;
    arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
    arg.parseArgs(argc, argv);
    
//     Vector3d _t(2, 5, .3);
//     Isometry2d _t0=v2t_2d(_t);
//     cerr << "ground truth vector: " <<endl;
//     cerr << t2v_2d(_t0) << endl;
//     cerr << "ground truth: " <<endl;
//     cerr << _t0.matrix() << endl;
    SE2 tresult;
    SE2 t0;
    CorrespondenceValidatorPtrVector validators;
    bool result = testRansac_fromGraph<Line2DMapping, RansacLine2DLinear, EdgeLine2D>(tresult, filename, t0, 
									    omegas, validators,
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
			cerr << t2v_2d(t0.toIsometry()*res) << endl;
			cerr << "transform error: " << endl;
			cerr << (t0.toIsometry()*res).matrix() << endl;
   } else {
     cerr << "unable to find a transform" << endl;
   }
  }
  
}
