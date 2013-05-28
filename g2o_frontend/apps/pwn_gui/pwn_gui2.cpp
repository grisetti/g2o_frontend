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

#include <unistd.h>

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

set<string> readDir(std::string dir) {
  DIR *dp;
  struct dirent *dirp;
  struct stat filestat;
  std::set<std::string> filenames;
  dp = opendir(dir.c_str());
  if (dp == NULL){
    return filenames;
  }
  
  while ((dirp = readdir( dp ))) {
    string filepath = dir + "/" + dirp->d_name;

    // If the file is a directory (or is in some way invalid) we'll skip it 
    if (stat(filepath.c_str(), &filestat)) 
      continue;
    if (S_ISDIR(filestat.st_mode))         
      continue;

    filenames.insert(filepath);
  }

  closedir(dp);
  return filenames;
}


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
    if(_parameters->show() && _frame){
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
};


int main(int argc, char** argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string workingDirectory = ".";

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
  arg.paramLeftOver("workingDirectory", workingDirectory, ".", "Path of the working directory. [string]", true);

  // Set parser input.
  arg.parseArgs(argc, argv);

  QApplication qApplication(argc, argv);
  PWNGuiMainWindow pwnGMW;
  QGraphicsScene *refScn, *currScn;
  
  pwnGMW.viewer_3d->setAxisIsDrawn(true);

  std::vector<string> filenames;
  std::set<string> filenamesset = readDir(workingDirectory);
  for (set<string>::const_iterator it = filenamesset.begin(); it != filenamesset.end(); it++) {
    filenames.push_back(*it);      
    QString listItem(&(*it)[0]);
    if (listItem.endsWith(".pgm", Qt::CaseInsensitive))
      pwnGMW.listWidget->addItem(listItem);
  }




  PinholePointProjector projector;
  StatsFinder statsFinder;
  statsFinder.setWorldRadius(0.2);
  PointInformationMatrixFinder pointInformationMatrixFinder;
  NormalInformationMatrixFinder normalInformationMatrixFinder;
  DepthImageConverter converter(&projector, 
                &statsFinder,
                &pointInformationMatrixFinder,
                &normalInformationMatrixFinder);

  Matrix3f cameraMatrix;
  cameraMatrix <<     
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;

  Eigen::Isometry3f sensorOffset;
  
  if (0){
    sensorOffset.setIdentity();
  } else {
    sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
    Quaternionf quat = Quaternionf(0.5, -0.5, 0.5, -0.5);
    sensorOffset.linear() = quat.toRotationMatrix();
  }

  sensorOffset.matrix().row(3) << 0,0,0,1;
  projector.setTransform(Eigen::Isometry3f::Identity());

  projector.setCameraMatrix(cameraMatrix);
 


  // Creating and setting aligner object.
  //Aligner aligner;
  CorrespondenceFinder correspondenceFinder;
  Linearizer linearizer;
#ifdef _PWN_USE_CUDA_
  CuAligner aligner;
#else
  Aligner aligner;
#endif

  aligner.setProjector(&projector);
  aligner.setLinearizer(&linearizer);
  linearizer.setAligner(&aligner);
  aligner.setCorrespondenceFinder(&correspondenceFinder);

  aligner.setInnerIterations(al_innerIterations);
  aligner.setOuterIterations(al_innerIterations);
  
    
  pwnGMW.show();
  refScn = pwnGMW.scene0();
  currScn = pwnGMW.scene1();


  bool newCloudAdded = false, wasInitialGuess = false;
  bool *initialGuessViewer = 0, *optimizeViewer = 0, *addCloud = 0, *clearLast = 0, *clearAll = 0;
  int *stepViewer = 0, *stepByStepViewer = 0;
  float *pointsViewer = 0, *normalsViewer = 0, *covariancesViewer = 0, *correspondencesViewer = 0;
  QListWidgetItem* itemList = 0;

  std::vector<DrawableFrame*> drawableFrameVector;
  Isometry3f initialGuess = Isometry3f::Identity();
  initialGuess.matrix().row(3) << 0,0,0,1;

  Isometry3f globalT = Isometry3f::Identity();
  DrawableFrameParameters* drawableFrameParameters = new DrawableFrameParameters();

  int imageRows = 0, imageCols = 0;
  while(!(*pwnGMW.closing())) {
    qApplication.processEvents();

    // Check window status changes.
    stepViewer = pwnGMW.step();
    pointsViewer = pwnGMW.points();
    normalsViewer = pwnGMW.normals();
    covariancesViewer = pwnGMW.covariances();
    correspondencesViewer = pwnGMW.correspondences();
    initialGuessViewer = pwnGMW.initialGuess();
    optimizeViewer = pwnGMW.optimize();
    stepByStepViewer = pwnGMW.stepByStep();
    addCloud = pwnGMW.addCloud();
    clearLast = pwnGMW.clearLast();    
    clearAll = pwnGMW.clearAll();
    itemList = pwnGMW.itemList();
    
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
     
    
    if(!wasInitialGuess && !newCloudAdded && drawableFrameVector.size() > 1 && *initialGuessViewer) {
      cerr << "initial guess" << endl;
      DrawableFrame* current = drawableFrameVector[drawableFrameVector.size()-1];
      DrawableFrame* reference = drawableFrameVector[drawableFrameVector.size()-2];
      current->setTransformation(reference->transformation());
      newCloudAdded = true;
      wasInitialGuess = true;
      *initialGuessViewer = 0;
    }
    // Optimize pressed with no step by step mode.
    else if(newCloudAdded && drawableFrameVector.size() > 1 && *optimizeViewer && !(*stepByStepViewer)) {
      DrawableFrame* current = drawableFrameVector[drawableFrameVector.size()-1];
      DrawableFrame* reference = drawableFrameVector[drawableFrameVector.size()-2];
      cerr << "optimizing" << endl;
      cerr << "current=" << current->frame() << endl;
      cerr << "reference= " << reference->frame() << endl;

      if(!wasInitialGuess) {
	aligner.setOuterIterations(al_outerIterations);
	aligner.setReferenceFrame(reference->frame());
	aligner.setCurrentFrame(current->frame());
	aligner.setInitialGuess(initialGuess);
	aligner.setSensorOffset(sensorOffset);
	aligner.align();
      }
      cout << "Local transformation: " << t2v(aligner.T()).transpose() << endl;

      globalT = reference->transformation()*aligner.T();
      // Update cloud drawing position.
      current->setTransformation(globalT);
      current->setLocalTransformation(aligner.T());

      // Show zBuffers.
      refScn->clear();
      currScn->clear();
      QImage refQImage;
      QImage currQImage;
      DepthImageView div;
      div.computeColorMap(300, 2000, 128);
      div.convertToQImage(refQImage, aligner.correspondenceFinder()->referenceDepthImage());
      div.convertToQImage(currQImage, aligner.correspondenceFinder()->currentDepthImage());
      refScn->addPixmap((QPixmap::fromImage(refQImage)).scaled(QSize((int)refQImage.width()/(ng_scale*3), (int)(refQImage.height()/(ng_scale*3)))));
      currScn->addPixmap((QPixmap::fromImage(currQImage)).scaled(QSize((int)currQImage.width()/(ng_scale*3), (int)(currQImage.height()/(ng_scale*3)))));
      pwnGMW.graphicsView1_2d->show();
      pwnGMW.graphicsView2_2d->show();
      
      wasInitialGuess = false;
      newCloudAdded = false;
      *initialGuessViewer = 0;
      *optimizeViewer = 0;
    }
    // Add cloud was pressed.
    else if(*addCloud) {
      if(itemList) {
	cerr << "adding a frame" << endl;
	std::string fname = itemList->text().toStdString();
	DepthImage depthImage;
	if (!depthImage.load(fname.c_str(), true)){
	  cerr << " skipping " << fname << endl;
	  continue;
	}
	imageRows = depthImage.rows();
	imageCols = depthImage.cols();
	correspondenceFinder.setSize(imageRows, imageCols);
	Frame * frame=new Frame();
	converter.compute(*frame, depthImage, sensorOffset);
	DrawableFrame* drawableFrame = new DrawableFrame(globalT, drawableFrameParameters, frame);
    	drawableFrameVector.push_back(drawableFrame);
	pwnGMW.viewer_3d->addDrawable(drawableFrame);
      }
      newCloudAdded = true;
      *addCloud = 0;
    }
    // clear buttons pressed.
    else if(*clearAll) {
      pwnGMW.viewer_3d->clearDrawableList();
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
      *clearAll = 0;
    }
    else if(*clearLast) {
      if(drawableFrameVector.size() > 0) {
        pwnGMW.viewer_3d->popBack();
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

    // To avoid memorized commands to be managed.
    *initialGuessViewer = 0;
    *optimizeViewer = 0;
    *addCloud = 0; 
    *clearAll = 0;
    *clearLast = 0;

    pwnGMW.viewer_3d->updateGL();

    usleep(10000);
  }
  return 0;  
}
