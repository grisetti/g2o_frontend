#include "alignment_horn2d.h"
#include "alignment_horn3d.h"
#include "alignment_se2.h"
#include "alignment_se3.h"
#include "alignment_line3d_linear.h"
#include "alignment_line2d_linear.h"
#include "line2d_correspondence_validator.h"
#include "g2o_frontend/basemath/bm_se2.h"
#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o_frontend/line_alignment/alignment_utils.h"
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

// #define FILTER_ON ;
#define FILTER_OFF ;

int v1id;
int v2id;
int maxIterations;
float inlierThreshold;
float stopFraction;
double omegaScaleFactor;

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
			  const Eigen::Matrix2d& omega,
			  CorrespondenceValidatorPtrVector& /*validators*/,
			  double /*outlierFraction = 0.2*/,
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
    OptimizableGraph::Vertex* v1_ = inputGraph.vertex(v1id);
    VertexSE2* v1 = dynamic_cast<VertexSE2*>(v1_);
    if(!v1) {
	cerr << " !!! Error in reading the first vertex!" << endl;
	return false;
    }
    OptimizableGraph::EdgeSet es1 = v1->edges();
  
    //second vertex to be aligned
    OptimizableGraph::Vertex* v2_ = inputGraph.vertex(v2id);
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

#ifdef FILTER_ON
    LinesSet s1, s2;
    for (OptimizableGraph::EdgeSet::iterator itv = es1.begin(); itv != es1.end(); itv++) 
    {
	EdgeSE2Line2D* el = dynamic_cast<EdgeSE2Line2D*>(*itv);
	if (!el)
	    continue;
	VertexLine2D* vl1 = dynamic_cast< VertexLine2D*>(el->vertices()[1]);
	const Line2D theLine = vl1->estimate();
	cout << "vl1 id: " << vl1->id() << "vl1 estimate1:\n" << theLine << endl;
	lineOfVertex lov;
	lov.line = theLine;
	lov.vertex = v1;
	lov.vline = vl1;
	s1.push_back(lov);
    }
    for (OptimizableGraph::EdgeSet::iterator itv2 = es2.begin(); itv2 != es2.end(); itv2++)
    {
	EdgeSE2Line2D* el2 = dynamic_cast<EdgeSE2Line2D*>(*itv2);
	if (!el2)
	    continue;
	VertexLine2D* vl2 = dynamic_cast<VertexLine2D*>(el2->vertices()[1]);
	const Line2D theLine2 = vl2->estimate();
	cout << "vl2 id: " << vl2->id() <<"vl2 estimate2:\n" << theLine2 << endl;
	lineOfVertex lov;
	lov.line = theLine2;
	lov.vertex = v2;
	lov.vline = vl2;
	s2.push_back(lov);
    }
    cout << "linesSets to be aligned: s1 dim = " <<  s1.size() << ", s2 dim = " << s2.size() << endl;
    LinesForMatching pairLinesSet = make_pair(s1, s2);
    LineCorrs currCorrs;
    CorrespondenceVector correspondences;
    bool resultCorrespondances = findCorrespondences(currCorrs, pairLinesSet);
    
    if(resultCorrespondances) 
    {
	IndexVector indices(currCorrs.size(), 0);
	double zeroVec[100];
	std::fill(zeroVec, zeroVec+100, 0);
	
	LinesSet s1 = pairLinesSet.first;
	LinesSet s2 = pairLinesSet.second;
	cout << "!!! Size of correspondances founded for the current pair of vertex: " << currCorrs.size() << endl;
	for (int ci = 0; ci < (int)currCorrs.size(); ci++)
	{
	    cerr << "Correspondance " << ci << "-th" << ", position in lines sets: "  <<currCorrs[ci].lid1 << ", " << currCorrs[ci].lid2 << ", with error:  " << currCorrs[ci].error << endl;

	    EdgeCorrespondenceType* edgeline = new EdgeCorrespondenceType();
	    edgeline->setMeasurementData(zeroVec);
	    typename EdgeCorrespondenceType::InformationType info = EdgeCorrespondenceType::InformationType::Identity();
	    edgeline->setInformation(omega);
	    
	    VertexLine2D* vli = dynamic_cast<VertexLine2D*>(inputGraph.vertex(s1[currCorrs[ci].lid1].vline->id()));
	    PointVertexType* v1 = new PointVertexType();
	    v1->setEstimate(vli->estimate());
	    v1->setId(vli->id());
	    if(!graph.vertex(v1.id())) graph.addVertex(v1);
	    
	    VertexLine2D* vlj = dynamic_cast<VertexLine2D*>(inputGraph.vertex(s2[currCorrs[ci].lid2].vline->id()));
	    PointVertexType* v2 = new PointVertexType();
	    v2->setEstimate(vlj->estimate());
	    v2->setId(vlj->id());
	    if(!graph.vertex(v2.id())) graph.addVertex(v2);

// 	    cerr << "id v1 " << vli->id() << " == " << v1->id() << ", id v2 " << vlj->id() << " == " << v2->id() << endl;
	    
	    edgeline->vertices()[0] = v1;
	    edgeline->vertices()[1] = v2;
	    Correspondence c(edgeline,100);
	    correspondences.push_back(c);
	    indices[ci]=ci;    
	}
	cerr << ">>>> correspondances dim: " << correspondences.size() << endl;
    }
#endif
    
//if not using the correspondences filter..
#ifdef FILTER_OFF
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
   
    
    int i = 0, k = 0;
    int j = numLinee1;
    for (OptimizableGraph::EdgeSet::iterator itv = es1.begin(); itv != es1.end() && k < numLinee1 && i < totCorr; itv++) 
    {
      
	EdgeSE2Line2D* el = dynamic_cast<EdgeSE2Line2D*>(*itv);
	if (!el)
	    continue;
	VertexLine2D* vl1_ = dynamic_cast<VertexLine2D*>(el->vertices()[1]);

	const Line2D theLine = vl1_->estimate();
	//cout << "vl1_ estimate1:\n" << theLine << endl;
	
	
	for (OptimizableGraph::EdgeSet::iterator itv2 = es2.begin(); itv2 != es2.end() && j < totCorr; itv2++)
	{
	    EdgeSE2Line2D* el2 = dynamic_cast<EdgeSE2Line2D*>(*itv2);
	    if (!el2)
		continue;
	    VertexLine2D* vl2_ = dynamic_cast<VertexLine2D*>(el2->vertices()[1]);
	    const Line2D theLine2 = vl2_->estimate();
	    //cout << "vl1_ estimate2:\n" << theLine2 << endl;
	
	    EdgeCorrespondenceType* edge = new EdgeCorrespondenceType(); 
	    edge->setMeasurementData(zeroVec);
	    typename EdgeCorrespondenceType::InformationType info = EdgeCorrespondenceType::InformationType::Identity();
	    
	    Eigen::Matrix2d myOmega=omega;
	    myOmega(0,0)+=omegaScaleFactor*fabs(vl1_->estimate()[1]*vl1_->estimate()[1]);
	    edge->setInformation(omega);
	    edge->vertices()[0]=vl1_;
	    edge->vertices()[1]=vl2_;

	    //graph.addEdge(edge);
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
#endif
    
    RansacType ransac;
    Line2DCorrespondenceValidator validator;
    ransac.correspondenceValidators().push_back(&validator);
    ransac.setCorrespondences(correspondences);
    ransac.setMaxIterations(maxIterations);
    ransac.setInlierErrorThreshold(inlierThreshold);
    ransac.setInlierStopFraction(stopFraction);
    std::vector<double> err;
    std::vector<int> inliers;
    bool resultRansac = ransac(result,inliers,debug);
    correspondences = ransac.correspondences();
    if(resultRansac){
      
	err = ransac.errors();
	cout << "Erros size: " << err.size() << endl;
	for (int h = 0; h < (int)err.size(); h++){
	    double erri = err[h];
	    //cout << "error of " << h << "-th correspondance: " << erri << endl ;
	    
	}
	cout << "Inliers: (size is " << inliers.size() << ")" << endl;
	for (size_t i = 0; i<inliers.size(); i++){
	  int idx = inliers[i];
	  float er = err[idx];
	  Correspondence& c=correspondences[idx];
	  g2o::OptimizableGraph::Edge* e=c.edge();
	  VertexLine2D* v1=static_cast<VertexLine2D*>(e->vertex(0));
	  VertexLine2D* v2=static_cast<VertexLine2D*>(e->vertex(1));
	  cerr << "(" << idx << "," << e << ","<< v1->id() << "," << v2->id() << ", " << er <<  "), ";
	  
	}
	cerr << endl;

	// AlignmentAlgorithmSE2Line2D& aligner=ransac.alignmentAlgorithm();
	// bool transformFound = aligner(result,correspondences,inliers);
    }
    
#if 1
	  ofstream os1("l1i.dat");
	  ofstream os2("l2i.dat");
	  ofstream os2R("l2ir.dat");
	  ofstream osc("cr.dat");
	  
	  for(size_t i = 0; i < inliers.size(); i++){
	    int idx = inliers[i];	
	    Correspondence& c=correspondences[idx];
	    g2o::OptimizableGraph::Edge* e=c.edge();
	    VertexLine2D* v1=static_cast<VertexLine2D*>(e->vertex(0));
	    VertexLine2D* v2=static_cast<VertexLine2D*>(e->vertex(1));
	    Eigen::Vector2d p11=dynamic_cast<VertexPointXY*>(inputGraph.vertex(v1->p1Id))->estimate();
	    Eigen::Vector2d p12=dynamic_cast<VertexPointXY*>(inputGraph.vertex(v1->p2Id))->estimate();
	    Eigen::Vector2d p21=dynamic_cast<VertexPointXY*>(inputGraph.vertex(v2->p1Id))->estimate();
	    Eigen::Vector2d p22=dynamic_cast<VertexPointXY*>(inputGraph.vertex(v2->p2Id))->estimate();

	    
	    os1 << p11.transpose() << endl;
	    os1 << p12.transpose() << endl;
	    os1 << endl;

	    os2 << p21.transpose() << endl;
	    os2 << p22.transpose() << endl;
	    os2 << endl;
	    
	    p21 = result * p21;
	    p22 = result * p22;

	    os2R << p21.transpose() << endl;
	    os2R << p22.transpose() << endl;
	    os2R << endl;
	    
	    Eigen::Vector2d pm1 = (p11+p12)*0.5;
	    Eigen::Vector2d pm2 = (p21+p22)*0.5;
	    osc << pm1.transpose() << endl;
	    osc << pm2.transpose() << endl;
	    osc << endl;
	    
	  }

	  ofstream osl1("l1.dat");
	  ofstream osl2r("l2r.dat");
	  for(size_t i = 0; i < correspondences.size(); i++){
	    Correspondence& c=correspondences[i];
	    g2o::OptimizableGraph::Edge* e=c.edge();
	    VertexLine2D* v1=static_cast<VertexLine2D*>(e->vertex(0));
	    VertexLine2D* v2=static_cast<VertexLine2D*>(e->vertex(1));
	    Eigen::Vector2d p11=dynamic_cast<VertexPointXY*>(inputGraph.vertex(v1->p1Id))->estimate();
	    Eigen::Vector2d p12=dynamic_cast<VertexPointXY*>(inputGraph.vertex(v1->p2Id))->estimate();
	    Eigen::Vector2d p21=dynamic_cast<VertexPointXY*>(inputGraph.vertex(v2->p1Id))->estimate();
	    Eigen::Vector2d p22=dynamic_cast<VertexPointXY*>(inputGraph.vertex(v2->p2Id))->estimate();

	    
	    osl1 << p11.transpose() << endl;
	    osl1 << p12.transpose() << endl;
	    osl1 << endl;

	    // osl2 << p21.transpose() << endl;
	    // osl2 << p22.transpose() << endl;
	    // osl2 << endl;
	    
	    p21 = result * p21;
	    p22 = result * p22;

	    osl2r << p21.transpose() << endl;
	    osl2r << p22.transpose() << endl;
	    osl2r << endl;
	  }

#endif
    return resultRansac;
}

int main(int argc, char** argv){
  { // Line2d
    cerr << "*************** TEST Line2D  *************** " <<endl;
//     std::vector<double> scales;
//     std::vector<double> offsets;

//     std::vector<double> noises;

    
    Eigen::Matrix2d omega;
    omega.setIdentity();
        
    string filename;
    g2o::CommandArgs arg;
    arg.param("id1", v1id, -1, "id of v1");
    arg.param("id2", v2id, -1, "id of v2");
    arg.param("i", maxIterations, 1000, "max iterations");
    arg.param("t", inlierThreshold, 0.1, "inlier threshold");
    arg.param("s", stopFraction, 0.2, "stop fraction");
    arg.param("os", omegaScaleFactor, 0, "omegaScaleFactor");
    arg.param("orr", omega(0,0), 50, "omega rr");
    arg.param("ott", omega(1,1), 20, "omega tt");
    arg.param("ort", omega(0,1), 0,  "omega rt");
    arg.param("s", stopFraction, 0.2, "stop fraction");
    omega(1,0)=omega(0,1);
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
									    omega, validators, 9,true);
   if (result){
     
// 			cerr << "ground truth vector: " <<endl;
// 			cerr << t2v_2d(_t0) << endl;
// 			cerr << "ground truth: " <<endl;
// 			cerr << _t0.matrix() << endl;
			cerr << "***********FOUND!***********" << endl;
			Isometry2d res = tresult.toIsometry();
			cerr << "transform found vector: " <<endl;
			cerr << t2v_2d(res) << endl;
			// cerr << "transform found: " <<endl;
			// cerr << res.matrix() << endl;
			cerr << "transform error vector: " << endl;
			cerr << t2v_2d(t0.toIsometry()*res) << endl;
			// cerr << "transform error: " << endl;
			// cerr << (t0.toIsometry()*res).matrix() << endl;
   } else {
     cerr << "unable to find a transform" << endl;
   }
  }
  
}
