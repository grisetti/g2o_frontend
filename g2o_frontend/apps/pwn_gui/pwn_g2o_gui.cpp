#include "g2o_frontend/pwn2/informationmatrixfinder.h"
#include "g2o_frontend/pwn2/statsfinder.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/basemath/bm_se3.h"

#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn_viewer/pwn_qglviewer.h"
#include "g2o_frontend/pwn_viewer/pwn_imageview.h"
#include "g2o_frontend/pwn_viewer/drawable_points.h"
#include "g2o_frontend/pwn_viewer/drawable_normals.h"
#include "g2o_frontend/pwn_viewer/drawable_covariances.h"
#include "g2o_frontend/pwn_viewer/drawable_correspondences.h"
#include "g2o_frontend/pwn_viewer/gl_parameter.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_points.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_normals.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_covariances.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_correspondences.h"
#include "pwn_gui_main_window.h"

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"


#include <signal.h>

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o_frontend/sensor_data/rgbd_data.h"
#include "g2o_frontend/sensor_data/imu_data.h"

#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/basemath/bm_se3.h"

#undef _PWN_USE_CUDA_

#ifdef _PWN_USE_CUDA_
#include "g2o_frontend/pwn_cuda/cualigner.h"
#endif// PWN_CUDA

#include <qapplication.h>
#include <iostream>
#include <fstream>
#include <set>
#include <dirent.h>
#include <sys/stat.h>

using namespace Eigen;
using namespace g2o;
using namespace std;
using namespace pwn;


/** this thing keeps the state of the parameters of the viewer. 
    We create the parameters only once and we bound these parameters 
    to each drawable scene we make*/

struct DrawableFrameParameters : public GLParameter{
  DrawableFrameParameters(int step = 1){
    float r= 0.3, g = 0.3, b=0.9;
    _pPoints = new GLParameterPoints(1.0f, Vector4f(r, g, b, 1.0f));
    _pPoints->setStep(step);
    _pNormals = new GLParameterNormals(1.0f, Vector4f(0.0f, 0.0f, 1.0f, 1.0f), 0.0f);
    _pNormals->setStep(step);
    _pCovariances = new GLParameterCovariances(1.0f, 
					       Vector4f(0.0f, 1.0f, 0.0f, 1.0f), Vector4f(1.0f, 0.0f, 0.0f, 1.0f),
					       0.02f, 0.0f);
    _pCovariances->setStep(step);
    _pCorrespondences = new GLParameterCorrespondences(1.0f, Vector4f(1.0f, 0.0f, 1.0f, 1.0f), 0.0f);
    _pCorrespondences->setStep(step);
  };

  ~DrawableFrameParameters(){
    delete _pPoints;
    delete _pCorrespondences;
    delete _pNormals;
    delete _pCovariances;
  }
  void applyGLParameter() {}

  GLParameterPoints *_pPoints;
  GLParameterNormals *_pNormals;
  GLParameterCovariances *_pCovariances;
  GLParameterCorrespondences *_pCorrespondences;
};


struct DrawableFrame : public Drawable {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Eigen::Isometry3f _localTransform;
  Frame* _frame;
  DrawableFrameParameters * _parameters;
  DrawablePoints *_dPoints;
  DrawableNormals *_dNormals;
  DrawableCovariances *_dCovariances;
  DrawableCorrespondences *_dCorrespondences;
  DrawableFrame* _previousDrawableFrame;


  DrawableFrame(const Eigen::Isometry3f& transformation_ = Eigen::Isometry3f::Identity(), 
		DrawableFrameParameters* parameters_=0, 
		Frame* frame_=0){
    setTransformation(transformation_);
    _parameters = parameters_;
    _frame = frame_;
    _dPoints=0;
    _dCorrespondences = 0;
    _dCovariances = 0;
    _dCorrespondences = 0;
    _previousDrawableFrame = 0;
    constructDrawableObjects();
  }

  virtual ~DrawableFrame(){
    clearDrawableObjects();
  }


  virtual void setViewer(PWNQGLViewer *viewer_) { 
    Drawable::setViewer(viewer_);
    if (_dPoints){
      _dPoints->setViewer(_viewer);
    }
    if (_dNormals){
      _dNormals->setViewer(_viewer);
    }
    if (_dCovariances){
      _dCovariances->setViewer(_viewer);
    }
    if (_dCorrespondences){
      _dCorrespondences->setViewer(_viewer);
    }

  }


  virtual bool setParameter(GLParameter *parameter_) {
    DrawableFrameParameters* dparams = dynamic_cast<DrawableFrameParameters*>(parameter_);
    if (! dparams)
      return false;
    _parameters = dparams;
    return true;
  }

  virtual GLParameter* parameter() {return _parameters;}


  void clearDrawableObjects(){
    if (_dPoints)
      delete _dPoints;
    if (_dNormals)
      delete _dNormals;
    if (_dCovariances)
      delete _dCovariances;
    if (_dCorrespondences)
      delete _dCorrespondences;
    _dPoints=0;
    _dCorrespondences = 0;
    _dCovariances = 0;
    _dCorrespondences = 0;
  }

  void constructDrawableObjects(){
    if (_frame) {
      _dPoints = new DrawablePoints(Isometry3f::Identity(), (GLParameter*)_parameters->_pPoints, &_frame->points(), &_frame->normals());
      _dNormals = new DrawableNormals(Isometry3f::Identity(), (GLParameter*)_parameters->_pNormals, &_frame->points(), &_frame->normals());
      _dCovariances = new DrawableCovariances(Isometry3f::Identity(), (GLParameter*)_parameters->_pCovariances, &_frame->stats());
      //_dCorrespondences = new DrawableCorrespondences(Isometry3f::Identity(), (GLParameter*)_parameters->_pCorrespondences, 0,
      //&frame.points(), &frame.points(), &correspondences);
    }
  }


  void setFrame(Frame* f) {
    if (f != _frame){
      clearDrawableObjects();
      _frame = f;
      constructDrawableObjects();
    }
  }

  Frame* frame() const {
    return _frame;
  }

  const Eigen::Isometry3f& localTransformation() const {return _localTransform;}
  void setLocalTransformation(const Eigen::Isometry3f& localTransform_) {_localTransform = localTransform_;}

  // Drawing function of the class object.
  void draw() {
    if(_parameters->isShown() && _frame){
      glPushMatrix();
      glMultMatrixf(_transformation.data());
      if (_dPoints)
	_dPoints->draw();
      if (_dNormals)
	_dNormals->draw();
      if (_dCovariances)
	_dCovariances->draw();
      if (_dCorrespondences)
	_dCorrespondences->draw();
      glPopMatrix();
    }
  }
  VertexSE3* _vertex; //HACK
};



struct ViewerState{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PWNGuiMainWindow* pwnGMW;
  QGraphicsScene *refScn, *currScn;
  SparseOptimizer* graph;
  std::vector<RGBDData*> listAssociations;

  PinholePointProjector* projector;
  StatsFinder* statsFinder;
  PointInformationMatrixFinder* pointInformationMatrixFinder;
  NormalInformationMatrixFinder* normalInformationMatrixFinder;
  DepthImageConverter* converter;


  Matrix3f cameraMatrix;
  Eigen::Isometry3f sensorOffset;
  
  CorrespondenceFinder* correspondenceFinder;
  Linearizer* linearizer;

  Aligner* aligner;

  bool newCloudAdded, wasInitialGuess;
  bool *initialGuessViewer, *optimizeViewer, *addCloud, *clearLast, *clearAll;
  int *stepViewer, *stepByStepViewer;
  float *pointsViewer, *normalsViewer, *covariancesViewer, *correspondencesViewer;
  QListWidgetItem* itemList;
  QListWidget* listWidget;

  std::vector<DrawableFrame*> drawableFrameVector;
  Isometry3f initialGuess;
  Isometry3f globalT;
  DrawableFrameParameters* drawableFrameParameters;
  int imageRows, imageCols;

  float ng_scale;
  float ng_worldRadius;
  float ng_curvatureThreshold;
  int al_innerIterations;
  int al_outerIterations;
  int vz_step;


  ViewerState(PWNGuiMainWindow* mwin){
    pwnGMW = mwin;
    graph = 0;
  }
  void init(){
    imageRows = 0;
    imageCols = 0;
    
    ng_scale = 1.0f;
    ng_worldRadius = 0.1;
    ng_curvatureThreshold = 1.0f;
    al_innerIterations = 1;
    al_outerIterations = 10;
    vz_step = 5;


    cameraMatrix <<     
      525.0f, 0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f, 0.0f, 1.0f;
    
    if (0){
      sensorOffset.setIdentity();
    } else {
      sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
      Quaternionf quat = Quaternionf(0.5, -0.5, 0.5, -0.5);
      sensorOffset.linear() = quat.toRotationMatrix();
    }

    sensorOffset.matrix().row(3) << 0,0,0,1;

    projector = new PinholePointProjector();
    statsFinder = new StatsFinder();

    pointInformationMatrixFinder = new PointInformationMatrixFinder();
    normalInformationMatrixFinder = new NormalInformationMatrixFinder ;
    converter= new DepthImageConverter (projector, statsFinder, 
					pointInformationMatrixFinder, normalInformationMatrixFinder);

    projector->setTransform(Eigen::Isometry3f::Identity());
    projector->setCameraMatrix(cameraMatrix);


    // Creating and setting aligner object.
    //Aligner aligner;
    correspondenceFinder = new CorrespondenceFinder();
    linearizer = new Linearizer() ;
#ifdef _PWN_USE_CUDA_
    aligner = new CuAligner() ;
#else
    aligner = new Aligner();
#endif
    
    aligner->setProjector(projector);
    aligner->setLinearizer(linearizer);
    linearizer->setAligner(aligner);
    aligner->setCorrespondenceFinder(correspondenceFinder);

    
    statsFinder->setWorldRadius(0.2);
    aligner->setInnerIterations(al_innerIterations);
    aligner->setOuterIterations(al_outerIterations);
    
    refScn = pwnGMW->scene0();
    currScn = pwnGMW->scene1();

    listWidget = pwnGMW->listWidget;
    drawableFrameParameters = new DrawableFrameParameters();

    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solverGauss = new OptimizationAlgorithmGaussNewton(blockSolver);
    graph = new SparseOptimizer();
    graph->setAlgorithm(solverGauss);

  }

  void refreshFlags(){
    stepViewer = pwnGMW->step();
    pointsViewer = pwnGMW->points();
    normalsViewer = pwnGMW->normals();
    covariancesViewer = pwnGMW->covariances();
    correspondencesViewer = pwnGMW->correspondences();
    initialGuessViewer = pwnGMW->initialGuess();
    optimizeViewer = pwnGMW->optimize();
    stepByStepViewer = pwnGMW->stepByStep();
    addCloud = pwnGMW->addCloud();
    clearLast = pwnGMW->clearLast();    
    clearAll = pwnGMW->clearAll();
    itemList = pwnGMW->itemList();
  }

  void load(const std::string& filename){
    clear();
    listWidget->clear();
    graph->clear();
    graph->load(filename.c_str());

    vector<int> vertexIds(graph->vertices().size());
    int k=0;
    for (OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it) {
      vertexIds[k++] = (it->first);
    }

    sort(vertexIds.begin(), vertexIds.end());

    listAssociations.clear();
    size_t maxCount = 2000;
    for(size_t i = 0; i < vertexIds.size() &&  i< maxCount; ++i) {
      OptimizableGraph::Vertex* _v = graph->vertex(vertexIds[i]);
      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(_v);
      if (! v)
	continue;
      OptimizableGraph::Data* d = v->userData();
      while(d) {
	RGBDData* rgbdData = dynamic_cast<RGBDData*>(d);
	if (!rgbdData){
	  d=d->next();
	  continue;
	}
	cerr << "got data" << endl;
	// retrieve from the rgb data the index of the parameter
	int paramIndex = rgbdData->paramIndex();
	// retrieve from the graph the parameter given the index  
	g2o::Parameter* _cameraParam = graph->parameter(paramIndex);
	// attempt a cast to a parameter camera  
	ParameterCamera* cameraParam = dynamic_cast<ParameterCamera*>(_cameraParam);
	if (! cameraParam){
	  cerr << "shall thou be damned forever" << endl;
	  return;
	}
	// yayyy we got the parameter
	Eigen::Matrix3f cameraMatrix;
	Eigen::Isometry3f sensorOffset;
	cameraMatrix.setZero();
      
	int cmax = 4;
	int rmax = 3;
	for (int c=0; c<cmax; c++){
	  for (int r=0; r<rmax; r++){
	    sensorOffset.matrix()(r,c)= cameraParam->offset()(r, c);
	    if (c<3)
	      cameraMatrix(r,c) = cameraParam->Kcam()(r, c);
	  }
	}
	char buf[1024];
	sprintf(buf,"%d",v->id());
	QString listItem(buf);
	listAssociations.push_back(rgbdData);
	listWidget->addItem(listItem);
	d=d->next();
      }
    }

  }

  void clear(){
    pwnGMW->viewer_3d->clearDrawableList();
    for(size_t i = 0; i < drawableFrameVector.size(); i++){
      if (drawableFrameVector[i]->frame())
	delete drawableFrameVector[i]->frame();
      delete(drawableFrameVector[i]);
    }
    drawableFrameVector.clear();
    globalT = Isometry3f::Identity();
    refScn->clear();
    currScn->clear();
    wasInitialGuess = false;
    newCloudAdded = false;
  }
  

  void updateDrawableParameters(){
    // Check feature visualization options.   
    if (stepViewer[0]) {
      drawableFrameParameters->_pPoints->setStep(stepViewer[1]);
      drawableFrameParameters->_pNormals->setStep(stepViewer[1]);
      drawableFrameParameters->_pCovariances->setStep(stepViewer[1]);
      drawableFrameParameters->_pCorrespondences->setStep(stepViewer[1]);
    } else {
      drawableFrameParameters->_pPoints->setStep(1);
      drawableFrameParameters->_pNormals->setStep(1);
      drawableFrameParameters->_pCovariances->setStep(1);
      //drawableFrameParameters->_pCorrespondences->setStep(stepViewer1);
    }

    if(pointsViewer[0])
      drawableFrameParameters->_pPoints->setPointSize(pointsViewer[1]);
    else
      drawableFrameParameters->_pPoints->setPointSize(0.0f);

    if(normalsViewer[0])
      drawableFrameParameters->_pNormals->setNormalLength(normalsViewer[1]);
    else
      drawableFrameParameters->_pNormals->setNormalLength(0.0f);

    if(correspondencesViewer[0])
      drawableFrameParameters->_pCorrespondences->setLineWidth(correspondencesViewer[1]);
    else
      drawableFrameParameters->_pCorrespondences->setLineWidth(0.0f);

    if(covariancesViewer[0])
      drawableFrameParameters->_pCovariances->setEllipsoidScale(covariancesViewer[1]);
    else
      drawableFrameParameters->_pCovariances->setEllipsoidScale(0.0f);
    
    if(correspondencesViewer[0])
      drawableFrameParameters->_pCorrespondences->setLineWidth(correspondencesViewer[1]);
    else
      drawableFrameParameters->_pCorrespondences->setLineWidth(0.0f);
  }

  void initialGuessSelected(){
      cerr << "initial guess" << endl;
      DrawableFrame* current = drawableFrameVector[drawableFrameVector.size()-1];
      DrawableFrame* reference = drawableFrameVector[drawableFrameVector.size()-2];
      current->setTransformation(reference->transformation());
      newCloudAdded = true;
      wasInitialGuess = true;
      *initialGuessViewer = 0;
  }

  void optimizeSelected(){
      DrawableFrame* current = drawableFrameVector[drawableFrameVector.size()-1];
      DrawableFrame* reference = drawableFrameVector[drawableFrameVector.size()-2];
      cerr << "optimizing" << endl;
      cerr << "current=" << current->frame() << endl;
      cerr << "reference= " << reference->frame() << endl;

      // cerr computing initial guess based on the frame positions, just for convenience
      Eigen::Isometry3d delta = reference->_vertex->estimate().inverse()*current->_vertex->estimate();
      for(int c=0; c<4; c++)
	for(int r=0; r<3; r++)
	  initialGuess.matrix()(r,c) = delta.matrix()(r,c);
      initialGuess.matrix().row(3) << 0,0,0,1;


      if(!wasInitialGuess) {
	aligner->setOuterIterations(al_outerIterations);
	aligner->setReferenceFrame(reference->frame());
	aligner->setCurrentFrame(current->frame());
	aligner->setInitialGuess(initialGuess);
	aligner->setSensorOffset(sensorOffset);
	aligner->align();
      }
      cout << "Local transformation: " << t2v(aligner->T()).transpose() << endl;

      globalT = reference->transformation()*aligner->T();
      // Update cloud drawing position.
      current->setTransformation(globalT);
      current->setLocalTransformation(aligner->T());

      // Show zBuffers.
      refScn->clear();
      currScn->clear();
      QImage refQImage;
      QImage currQImage;
      DepthImageView div;
      div.computeColorMap(300, 2000, 128);
      div.convertToQImage(refQImage, aligner->correspondenceFinder()->referenceDepthImage());
      div.convertToQImage(currQImage, aligner->correspondenceFinder()->currentDepthImage());
      refScn->addPixmap((QPixmap::fromImage(refQImage)).scaled(QSize((int)refQImage.width()/(ng_scale*3), (int)(refQImage.height()/(ng_scale*3)))));
      currScn->addPixmap((QPixmap::fromImage(currQImage)).scaled(QSize((int)currQImage.width()/(ng_scale*3), (int)(currQImage.height()/(ng_scale*3)))));
      pwnGMW->graphicsView1_2d->show();
      pwnGMW->graphicsView2_2d->show();
      
      wasInitialGuess = false;
      newCloudAdded = false;
      *initialGuessViewer = 0;
      *optimizeViewer = 0;
  }


  void addCloudSelected(){
      if(itemList) {
	cerr << "adding a frame" << endl;
	int index = pwnGMW->listWidget->row(itemList);
	cerr << "index: " << endl;
	std::string fname = listAssociations[index]->baseFilename()+"_depth.pgm";
	DepthImage depthImage;
	if (!depthImage.load(fname.c_str(), true)){
	  cerr << " skipping " << fname << endl;
	  newCloudAdded = false;
	  *addCloud = 0;
	  return;
	}
	imageRows = depthImage.rows();
	imageCols = depthImage.cols();
	correspondenceFinder->setSize(imageRows, imageCols);
	Frame * frame=new Frame();
	converter->compute(*frame, depthImage, sensorOffset);
	
	
	
	OptimizableGraph::DataContainer* dc =listAssociations[index]->dataContainer();
	cerr << "datacontainer: " << dc << endl;
	VertexSE3* v = dynamic_cast<VertexSE3*>(dc);
	cerr << "vertex of the frame: " << v->id() << endl;
	
	// check if the element is the last in the list
	if (drawableFrameVector.size()){
	  DrawableFrame* reference = drawableFrameVector[drawableFrameVector.size()-1];
	  cerr << "reference= " << reference->frame() << endl;
	  
	  // cerr computing initial guess based on the frame positions, just for convenience
	  Eigen::Isometry3d delta = reference->_vertex->estimate().inverse()*v->estimate();
	  Eigen::Isometry3f myDelta;
	  for(int c=0; c<4; c++)
	    for(int r=0; r<3; r++)
	      myDelta.matrix()(r,c) = delta.matrix()(r,c);
	  myDelta.matrix().row(3) << 0,0,0,1;
	  globalT = globalT*myDelta;
	} else {
	  globalT.setIdentity();
	}
	DrawableFrame* drawableFrame = new DrawableFrame(globalT, drawableFrameParameters, frame);
	drawableFrame->_vertex = v;
    	drawableFrameVector.push_back(drawableFrame);
	pwnGMW->viewer_3d->addDrawable(drawableFrame);
      }
      newCloudAdded = true;
      *addCloud = 0;
  }

  void clearLastSelected(){
    if(drawableFrameVector.size() > 0) {
      pwnGMW->viewer_3d->popBack();
      DrawableFrame* lastDrawableFrame = drawableFrameVector[drawableFrameVector.size()-1];
      if (lastDrawableFrame->frame())
	delete lastDrawableFrame->frame();
      delete lastDrawableFrame;
      drawableFrameVector.pop_back();
    }
    if(drawableFrameVector.size() > 0) { 
      DrawableFrame* lastDrawableFrame = drawableFrameVector[drawableFrameVector.size()-1];
      globalT = lastDrawableFrame->transformation();;
    } 
    refScn->clear();
    currScn->clear();
    wasInitialGuess = false;
    newCloudAdded = false;
    *clearLast = 0;
  }


  void processCommands(){
    refreshFlags();
    updateDrawableParameters();
    if(!wasInitialGuess && !newCloudAdded && drawableFrameVector.size() > 1 && *initialGuessViewer) {
      initialGuessSelected();
    } else if(newCloudAdded && drawableFrameVector.size() > 1 && *optimizeViewer && !(*stepByStepViewer)) {
      optimizeSelected();
    }
    // Add cloud was pressed.
    else if(*addCloud) {
      addCloudSelected();
    }
    // clear buttons pressed.
    else if(*clearAll) {
      clear();
      *clearAll = 0;
    }
    else if(*clearLast) {
      clearLastSelected();
    }
    // To avoid memorized commands to be managed.
    *initialGuessViewer = 0;
    *optimizeViewer = 0;
    *addCloud = 0; 
    *clearAll = 0;
    *clearLast = 0;
    pwnGMW->viewer_3d->updateGL();
  }

};


int main(int argc, char** argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string g2oFilename;

  // Variables for the input parameters. Just type on the command line
  // ./pwn_normal_extraction -h to have more details about them.
  float ng_scale = 1.0f;
  float ng_curvatureThreshold = 1.0f;
  int al_innerIterations = 1;
  int al_outerIterations = 10;
  int vz_step = 5;

  // Define the camera matrix, place here the values for the particular 
  // depth camera used (Kinect, Xtion or any other type). This particular
  // matrix is the one related to the Kinect.

  
  // Input parameters handling.
  g2o::CommandArgs arg;
  
  // Optional input parameters.
  arg.param("ng_scale", ng_scale, 1.0f, "Specify the scaling factor to apply on the depth image. [float]");
  arg.param("ng_curvatureThreshold", ng_curvatureThreshold, 1.0f, "Specify the max surface curvature threshold for which normals are discarded. [float]");
  arg.param("al_innerIterations", al_innerIterations, 1, "Specify the inner iterations. [int]");
  arg.param("al_outerIterations", al_outerIterations, 10, "Specify the outer iterations. [int]");
  arg.param("vz_step", vz_step, 5, "A graphic element is drawn each vz_step elements. [int]");

  // Last parameter has to be the working directory.
  arg.paramLeftOver("", g2oFilename, "", "g2o input inputfilename", true);

  // Set parser input.
  arg.parseArgs(argc, argv);

  QApplication qApplication(argc, argv);
  PWNGuiMainWindow pwnGMW;
  ViewerState viewerState(&pwnGMW);
  viewerState.init();
  viewerState.load(g2oFilename);
  pwnGMW.show();

  while(!(*pwnGMW.closing())) {
    qApplication.processEvents();
    viewerState.processCommands();
    usleep(10000);
  }
  return 0;  
}
