#include <signal.h>


#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"
#include "g2o_frontend/sensor_data/imu_data.h"

#include "g2o_frontend/pwn/depthimage.h"
#include "g2o_frontend/pwn/pointwithnormal.h"
#include "g2o_frontend/pwn/pointwithnormalstatsgenerator.h"
#include "g2o_frontend/pwn/pointwithnormalaligner.h"
#include "g2o_frontend/pwn/gaussian3.h"
#include "g2o/stuff/timeutil.h"

using namespace std;
using namespace g2o;

struct Frame{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  DepthImage depthImage;
  MatrixXi indexImage;
  PointWithNormalVector points;
  Gaussian3fVector gaussians;
  PointWithNormalSVDVector svds;
  MatrixXf zBuffer;
  

  bool load(std::string filename) {
    return depthImage.load(filename.c_str());
  }
  void computeStats(PointWithNormalStatistcsGenerator * generator, const Matrix3f& cameraMatrix){
    zBuffer.resize(depthImage.rows(), depthImage.cols());
    gaussians.fromDepthImage(depthImage,cameraMatrix);
    gaussians.toPointWithNormalVector(points);
    indexImage.resize(depthImage.rows(), depthImage.cols());
    gaussians.toIndexImage(indexImage, zBuffer, cameraMatrix, Eigen::Isometry3f::Identity(), 10);
    cerr << "points: " << points.size() << endl; 
    svds.resize(points.size());
    double tNormalStart = get_time();
    generator->computeNormalsAndSVD(points, svds, indexImage, cameraMatrix);
    double tNormalEnd = get_time();
    cerr << "Normal Extraction took " << tNormalEnd - tNormalStart << " sec." << endl;
  }
  void setAligner(PointWithNormalAligner* aligner, bool isRef){
    if (isRef) {
      aligner->setReferenceCloud(&points, &svds);
    } else {
      aligner->setCurrentCloud(&points, &svds);
    }
  }
};


volatile bool hasToStop;
void sigquit_handler(int sig)
{
  if (sig == SIGINT) {
    hasToStop = 1;
    static int cnt = 0;
    if (cnt++ == 2) {
      cerr << __PRETTY_FUNCTION__ << " forcing exit" << endl;
      exit(1);
    }
  }
}

// these are to force the linking in case
// ld wants to be picky.

VertexSE3* v=new VertexSE3;
EdgeSE3* e = new EdgeSE3;
LaserRobotData* lrd = new LaserRobotData;
ParameterCamera* pc = new ParameterCamera;
ParameterSE3Offset* po = new ParameterSE3Offset;
RGBDData* rgbd=new RGBDData;
ImuData* imu = new ImuData;

int main(int argc, char**argv){
  hasToStop = false;
  string filename;
  string outfilename;

  float numThreads;

  int ng_step;
  int ng_minPoints;
  int ng_imageRadius;
  float ng_worldRadius;
  float ng_maxCurvature;
  
  float al_inlierDistance;
  float al_inlierCurvatureRatio;
  float al_inlierNormalAngle;
  float al_inlierMaxChi2;
  float al_scale;
  float al_flatCurvatureThreshold;
  float al_outerIterations;
  float al_nonLinearIterations;
  float al_linearIterations;
  float al_minInliers;
  float al_lambda;
  float al_debug;

  PointWithNormalStatistcsGenerator* normalGenerator = new PointWithNormalStatistcsGenerator;
  PointWithNormalAligner* aligner=new PointWithNormalAligner;

  g2o::CommandArgs arg;
  arg.param("ng_step",ng_step,normalGenerator->step(),"compute a normal each x pixels") ;
  arg.param("ng_minPoints",ng_minPoints,normalGenerator->minPoints(),"minimum number of points in a region to compute the normal");
  arg.param("ng_imageRadius",ng_imageRadius,normalGenerator->imageRadius(), "radius of a ball in the works where to compute the normal for a pixel");
  arg.param("ng_worldRadius",ng_worldRadius,  normalGenerator->worldRadius(), "radius of a ball in the works where to compute the normal for a pixel");
  arg.param("ng_maxCurvature",ng_maxCurvature, normalGenerator->maxCurvature(), "above this threshold the normal is not computed");
  
  arg.param("al_inlierDistance", al_inlierDistance, aligner->inlierDistanceThreshold(),  "max metric distance between two points to regard them as iniliers");
  arg.param("al_inlierCurvatureRatio", al_inlierCurvatureRatio, aligner->inlierCurvatureRatioThreshold(), "max metric distance between two points to regard them as iniliers");
  arg.param("al_inlierNormalAngle", al_inlierNormalAngle, aligner->inlierNormalAngularThreshold(), "max metric distance between two points to regard them as iniliers");
  arg.param("al_inlierMaxChi2", al_inlierMaxChi2, aligner->inlierMaxChi2(), "max metric distance between two points to regard them as iniliers");
  arg.param("al_minInliers", al_minInliers, aligner->minInliers(), "minimum numver of inliers to do the matching");
  arg.param("al_scale", al_scale, aligner->scale(), "scale of the range image for the alignment");
  arg.param("al_flatCurvatureThreshold", al_flatCurvatureThreshold, aligner->flatCurvatureThreshold(), "curvature above which the patches are not considered planar");
  arg.param("al_outerIterations", al_outerIterations, aligner->outerIterations(), "outer interations (incl. data association)");
  arg.param("al_linearIterations", al_linearIterations, aligner->linearIterations(), "linear iterations for each outer one (uses R,t)");
  arg.param("al_nonLinearIterations", al_nonLinearIterations, aligner->nonLinearIterations(), "nonlinear iterations for each outer one (uses q,t)");
  arg.param("al_lambda", al_lambda, aligner->lambda(), "damping factor for the transformation update, the higher the smaller the step");
  arg.param("al_debug", al_debug, aligner->debug(), "prints lots of stuff");
  arg.param("numThreads", numThreads, 1, "numver of threads for openmp");
  arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
  arg.paramLeftOver("graph-output", outfilename , "", "output graph file", true);
  

  if (numThreads<1)
    numThreads = 1;
  
  arg.parseArgs(argc, argv);
  
  normalGenerator->setStep(ng_step);
  normalGenerator->setMinPoints(ng_minPoints);
  normalGenerator->setImageRadius(ng_imageRadius);
  normalGenerator->setWorldRadius(ng_worldRadius);
  normalGenerator->setMaxCurvature(ng_maxCurvature);
#ifdef _PWN_USE_OPENMP_
  normalGenerator->setNumThreads(numThreads);
#endif //_PWN_USE_OPENMP_

  aligner->setInlierDistanceThreshold(al_inlierDistance);
  aligner->setInlierCurvatureRatioThreshold(al_inlierCurvatureRatio);
  aligner->setInlierNormalAngularThreshold(al_inlierNormalAngle);
  aligner->setInlierMaxChi2(al_inlierMaxChi2);
  aligner->setMinInliers(al_minInliers);
  aligner->setScale(al_scale);
  aligner->setFlatCurvatureThreshold(al_flatCurvatureThreshold);
  aligner->setOuterIterations(al_outerIterations);
  aligner->setLinearIterations(al_linearIterations);
  aligner->setNonLinearIterations(al_nonLinearIterations);
  aligner->setLambda(al_lambda);
  aligner->setDebug(al_debug);
#ifdef _PWN_USE_OPENMP_
  aligner->setNumThreads(numThreads);
#endif //_PWN_USE_OPENMP_
  
  arg.parseArgs(argc, argv);


  
  // graph construction
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);
  SparseOptimizer * graph = new SparseOptimizer();


  Frame* referenceFrame= 0;
  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;


  graph->setAlgorithm(solverGauss);
  graph->load(filename.c_str());
  
  // sort the vertices based on the id
  std::vector<int> vertexIds(graph->vertices().size());
  int k=0;
  for (OptimizableGraph::VertexIDMap::iterator it=graph->vertices().begin(); it!= graph->vertices().end(); it ++){
    vertexIds[k++] = (it->first);
  }

  std::sort(vertexIds.begin(), vertexIds.end());
  

  Factory* factory = Factory::instance();
  VertexSE3* vOld = 0;
  RGBDData*  oldData = 0;
  
  for (size_t i=0; i<vertexIds.size() && ! hasToStop; i++){
    OptimizableGraph::Vertex* v=graph->vertex(vertexIds[i]);
    cerr << "vertex: " << v->id() << " type:" << factory->tag(v) << endl;
    OptimizableGraph::Data* d = v->userData();
    k = 0;
    while(d){
      if (d) {
	RGBDData *rgbdData = dynamic_cast<RGBDData*>(d);
	VertexSE3* v3=dynamic_cast<VertexSE3*>(v);
	if (rgbdData && v3){
	  cerr << "rgbd_data found in vertex:" << v3->id() << endl;
	  if (vOld) {
	    cerr << "aligning with vertex:" << vOld->id() << endl; 
	    cerr << "robotOffset:" << t2v(vOld->estimate().inverse() * v3->estimate()).transpose() << endl; 
	  }
	  Frame* currentFrame = new Frame();
	  std::string currentFilename=rgbdData->baseFilename()+"_depth.pgm";
	  bool loadOk = currentFrame->load(currentFilename);
	  if (! loadOk) {
	    cerr << "failure in loading stuff" << endl;
	    delete currentFrame;
	    currentFrame = 0;
	    continue;
	  }
	  currentFrame->computeStats(normalGenerator,cameraMatrix);
    	  cerr << "currentFilename: [" << currentFilename << "]" << endl;
	  g2o::Parameter* _pNew = graph->parameter(rgbdData->paramIndex());
	  ParameterCamera* pNew = dynamic_cast<ParameterCamera*>(_pNew);
	  ParameterSE3Offset* pOffsetNew = dynamic_cast<ParameterSE3Offset*>(graph->parameter(rgbdData->paramIndex()+10));
	  if(! pOffsetNew) {
	    cerr << "new offset created" << endl;
	    pOffsetNew = new ParameterSE3Offset;
	    pOffsetNew->setOffset(pNew->offset());
	    pOffsetNew->setId(rgbdData->paramIndex()+10);
	    graph->addParameter(pOffsetNew);
	  } else {
	    cerr << "reusing old offset" << endl;
	  }
	  //Eigen::Isometry3d currentRobotPose = v3->estimate();
	  Eigen::Isometry3d currentCameraPose = v3->estimate()*pNew->offset();
	  if (referenceFrame && oldData && vOld ) {
	    //Eigen::Isometry3d oldRobotPose = vOld->estimate();
	    g2o::Parameter* _pOld = graph->parameter(oldData->paramIndex());
	    ParameterCamera* pOld = dynamic_cast<ParameterCamera*>(_pOld);
	    g2o::Parameter* pOffsetOld = graph->parameter(oldData->paramIndex()+10);
	    Eigen::Isometry3d oldCameraPose = vOld->estimate()*pOld->offset();
	    Eigen::Isometry3d oldCameraMovement = oldCameraPose.inverse()*currentCameraPose;

	    Matrix6f omega;
	    Vector6f mean;
	    float tratio;
	    float rratio;
	    referenceFrame->setAligner(aligner, true);
	    currentFrame->setAligner(aligner, false);

	    aligner->setImageSize(currentFrame->depthImage.rows(), currentFrame->depthImage.cols());
	    Eigen::Isometry3f X(oldCameraMovement);
	    
	    double ostart = get_time();
	    float error;
	    int result = aligner->align(error, X, mean, omega, tratio, rratio);
	    cerr << "inliers=" << result << " error/inliers: " << error/result << endl;
	    double oend = get_time();
	    cerr << "alignment took: " << oend-ostart << " sec." << endl;
	    cerr << "aligner scaled image size: " << aligner->scaledImageRows() << " " << aligner->scaledImageCols() << endl;

	    if(rratio < 100 && tratio < 100) {
	      // write the edge frame
	      Vector6f x = mean;
	      //Vector3f t = x.head<3>();
	      Vector3f mq = x.tail<3>();
	      float w = mq.squaredNorm();
	      if (w>1){
		mq.setZero();
		w = 1.0f;
	      } else {
		w = sqrt(1-w);
	      }
	      mean = t2v(X);

	      cerr << "initialTransform: " << t2v(oldCameraMovement) << endl;
	      cerr << "mlSolution: " << t2v(X) << endl;
	      cerr << "mlMean: " << mean << endl;
	      

	      EdgeSE3Offset* eNew = new EdgeSE3Offset;
	      Vector6d measNew;
	      Matrix6d omegaNew = Matrix6d::Identity();
	      for (int i=0; i<6; i++){
		measNew(i)=mean(i);
		// for (int j=0; j<6; j++){
		//   omegaNew(i,j)=omega(i,j);
		// }
	      }
	      eNew->setVertex(0,vOld);
 	      eNew->setParameterId(0,pOffsetOld->id());
	      eNew->setVertex(1,v3);
	      eNew->setParameterId(1,pOffsetNew->id());
	      eNew->setMeasurement(v2t(measNew));
	      eNew->setInformation(omegaNew);
	      graph->addEdge(eNew);
	      // os << "EDGE_SE3:QUAT " << previousIndex << " " << i << " ";
	      // os << t.transpose() << " " << mq.transpose() << " " << w <<  " ";
	      // for (int r=0; r<6; r++){
	      // 	for (int c=r; c<6; c++){
	      // 	  os << omega(r,c) << " ";
	      // 	}
	      // } 
	      // os << endl;
	    }
	  }
	  if (referenceFrame) {
	    delete referenceFrame;
	    referenceFrame = 0;
	  }
	  referenceFrame = currentFrame;
	  vOld = v3;
	  oldData = rgbdData;
	}
	// write here something for doing something with the data you just pulled
	k++;
      }
      d=d->next();
    }
  }
  if (outfilename != ""){
    cerr << "writing out [" << outfilename << "]"<< endl;
    graph->save(outfilename.c_str());
  }
}
