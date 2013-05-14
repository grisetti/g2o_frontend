#include "g2o_frontend/pwn2/informationmatrixfinder.h"
#include "g2o_frontend/pwn2/statsfinder.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/basemath/bm_se3.h"

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

struct DrawableFrame {
  DrawableFrame(string f, int s) {
    filename = f;
    step = s;

    float r = 0.0f + 0.75f*rand()/double(RAND_MAX);
    float g = 0.0f + 0.75f*rand()/double(RAND_MAX);
    float b = 0.0f + 0.75f*rand()/double(RAND_MAX);

    pPoints = new GLParameterPoints(1.0f, Vector4f(r, g, b, 1.0f));
    pPoints->setStep(step);
    pNormals = new GLParameterNormals(1.0f, Vector4f(0.0f, 0.0f, 1.0f, 1.0f), 0.0f);
    pNormals->setStep(step);
    pCovariances = new GLParameterCovariances(1.0f, 
					      Vector4f(0.0f, 1.0f, 0.0f, 1.0f), Vector4f(1.0f, 0.0f, 0.0f, 1.0f),
					      0.02f, 0.0f);
    pCovariances->setStep(step);
    pCorrespondences = new GLParameterCorrespondences(1.0f, Vector4f(1.0f, 0.0f, 1.0f, 1.0f), 0.0f);
    pCorrespondences->setStep(step);

    dPoints = new DrawablePoints(Isometry3f::Identity(), (GLParameter*)pPoints, &frame.points(), &frame.normals());
    dNormals = new DrawableNormals(Isometry3f::Identity(), (GLParameter*)pNormals, &frame.points(), &frame.normals());
    dCovariances = new DrawableCovariances(Isometry3f::Identity(), (GLParameter*)pCovariances, &frame.stats());
    dCorrespondences = new DrawableCorrespondences(Isometry3f::Identity(), (GLParameter*)pCorrespondences, 0,
                           &frame.points(), &frame.points(), &correspondences);
  }
  
  void computeStats() {
    Eigen::Matrix3f cameraMatrix;
    cameraMatrix << 
      525.0f, 0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f, 0.0f, 1.0f;
    
    float ng_curvatureThreshold = 1.0f;

    if(!depthImage.load(filename.c_str(), true)) {
      cout << "Failure while loading the depth image: " << filename<< " skipping image!" << endl;
      return;
    }
    cout << endl << "Loaded depth image " << filename << " of size: " << depthImage.rows() << "x" << depthImage.cols() << endl;
    
    /************************************************************************
     *                         Point Unprojection                           *
     ************************************************************************/
    cout << "Unprojecting points...";

    // Update the size of the index image.
    indexImage.resize(depthImage.rows(), depthImage.cols());
    
    // Set the camera matrix of the projector object.
    projector.setCameraMatrix(cameraMatrix);
  
    // Get the points in the 3d euclidean space.
    projector.unProject(frame.points(), indexImage, depthImage);
    
    cout << " done." << endl;

    /************************************************************************
     *                         Normal Computation                           *
     ************************************************************************/
    cout << "Computing normals...";

    PointIntegralImage integralImage;
    MatrixXi intervalImage;
  
    // Compute the integral images.
    integralImage.compute(indexImage, frame.points());
    
    // Compute the intervals.
    projector.projectIntervals(intervalImage, depthImage, 0.1f);
    
    // Resize the vector containing the stats to have the same length of the vector of points.
    frame.stats().resize(frame.points().size());
    std::fill(frame.stats().begin(), frame.stats().end(), PointStats());
    
    // Creating the stas generator object. 
    StatsFinder statsGenerator;
  
    // Stats and normals computation.
    statsGenerator.compute(frame.normals(),
               frame.stats(),
               frame.points(),
			   integralImage,
			   intervalImage,
			   indexImage,
			   ng_curvatureThreshold);
    
    cout << " done." << endl;

    /************************************************************************
     *                         Omega Computation                            *
     ************************************************************************/
    cout << "Computing omegas...";

    // Creating the omegas generators objects.
    PointInformationMatrixFinder pointInformationMatrixFinder;
    NormalInformationMatrixFinder normalInformationMatrixFinder;
  
    // Omegas computation.
    pointInformationMatrixFinder.compute(frame.pointInformationMatrix(), frame.stats(), frame.normals());
    normalInformationMatrixFinder.compute(frame.normalInformationMatrix(), frame.stats(), frame.normals());

    cout << " done." << endl;

    dPoints->setPoints(&frame.points());
    dPoints->setNormals(&frame.normals());
    dNormals->setPoints(&frame.points());
    dNormals->setNormals(&frame.normals());
    dCovariances->setCovariances(&frame.stats());
  }

  Frame frame;
  CorrespondenceVector correspondences;
  PinholePointProjector projector;
  string filename;
  int step;
  DepthImage depthImage;
  Eigen::MatrixXi indexImage;

  GLParameterPoints *pPoints;
  GLParameterNormals *pNormals;
  GLParameterCovariances *pCovariances;
  GLParameterCorrespondences *pCorrespondences;

  DrawablePoints *dPoints;
  DrawableNormals *dNormals;
  DrawableCovariances *dCovariances;
  DrawableCorrespondences *dCorrespondences;
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
  Matrix3f cameraMatrix;
  cameraMatrix <<     
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  
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
  
  std::vector<string> filenames;
  std::set<string> filenamesset = readDir(workingDirectory);
  for (set<string>::const_iterator it = filenamesset.begin(); it != filenamesset.end(); it++) {
    filenames.push_back(*it);      
    QString listItem(&(*it)[0]);
    if (listItem.endsWith(".pgm", Qt::CaseInsensitive))
      pwnGMW.listWidget->addItem(listItem);
  }

  // Creating and setting aligner object.
  //Aligner aligner;
  CorrespondenceFinder correspondenceFinder;
  Linearizer linearizer;

#ifdef _PWN_USE_CUDA_
  CudaAligner::CuAligner aligner;
#else
  Aligner aligner;
#endif

  aligner.setLinearizer(&linearizer);
  aligner.setCorrespondenceFinder(&correspondenceFinder);
  aligner.setInnerIterations(al_innerIterations);
  
  pwnGMW.show();
  refScn = pwnGMW.scene0();
  currScn = pwnGMW.scene1();

  bool newCloudAdded = false, wasInitialGuess = false;
  bool *initialGuessViewer = 0, *optimizeViewer = 0, *addCloud = 0, *clearLast = 0, *clearAll = 0;
  int *stepViewer = 0, *stepByStepViewer = 0;
  float *pointsViewer = 0, *normalsViewer = 0, *covariancesViewer = 0, *correspondencesViewer = 0;
  QListWidgetItem* itemList = 0;

  std::vector<DrawableFrame*> drawableFrameVector;
  DrawableFrame *drawableFrame = 0;
  Isometry3f initialGuess = Isometry3f::Identity();
  Isometry3f sensorOffset = Isometry3f::Identity();
  Isometry3f globalT = Isometry3f::Identity();
  Isometry3f stepByStepInit = Isometry3f::Identity();
  std::vector<Isometry3f> localT; 

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
    for(size_t i = 0; i < drawableFrameVector.size(); i++) {
      if(stepViewer[0]) {
    drawableFrameVector[i]->pPoints->setStep(stepViewer[1]);
    drawableFrameVector[i]->pNormals->setStep(stepViewer[1]);
    drawableFrameVector[i]->pCovariances->setStep(stepViewer[1]);
    drawableFrameVector[i]->pCorrespondences->setStep(stepViewer[1]);
      }
      else {
    drawableFrameVector[i]->pPoints->setStep(1.0f);
    drawableFrameVector[i]->pNormals->setStep(1.0f);
    drawableFrameVector[i]->pCovariances->setStep(1.0f);
    drawableFrameVector[i]->pCorrespondences->setStep(1.0f);
      }
      
      if(pointsViewer[0])
    drawableFrameVector[i]->pPoints->setPointSize(pointsViewer[1]);
      else
    drawableFrameVector[i]->pPoints->setPointSize(0.0f);
      
      if(normalsViewer[0])
    drawableFrameVector[i]->pNormals->setNormalLength(normalsViewer[1]);
      else
    drawableFrameVector[i]->pNormals->setNormalLength(0.0f);
      
      if(covariancesViewer[0])
    drawableFrameVector[i]->pCovariances->setEllipsoidScale(covariancesViewer[1]);
      else
    drawableFrameVector[i]->pCovariances->setEllipsoidScale(0.0f);
      
      if(correspondencesViewer[0])
    drawableFrameVector[i]->pCorrespondences->setLineWidth(correspondencesViewer[1]);
      else
    drawableFrameVector[i]->pCorrespondences->setLineWidth(0.0f);
    }
    
    if(!wasInitialGuess && !newCloudAdded && drawableFrameVector.size() > 1 && *initialGuessViewer) {
      drawableFrameVector[drawableFrameVector.size()-1]->dPoints->setTransformation(globalT * localT[localT.size()-1].inverse());
      drawableFrameVector[drawableFrameVector.size()-1]->dNormals->setTransformation(globalT * localT[localT.size()-1].inverse());
      drawableFrameVector[drawableFrameVector.size()-1]->dCovariances->setTransformation(globalT * localT[localT.size()-1].inverse());
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setReferencePointsTransformation(globalT * localT[localT.size()-1].inverse());
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setTransformation(globalT * localT[localT.size()-1].inverse());
      newCloudAdded = true;
      wasInitialGuess = true;
      *initialGuessViewer = 0;
    }
    // Optimize pressed with no step by step mode.
    else if(newCloudAdded && drawableFrameVector.size() > 1 && *optimizeViewer && !(*stepByStepViewer)) {
      if(!wasInitialGuess) {
	aligner.setOuterIterations(al_outerIterations);

    aligner.correspondenceFinder()->setSize(drawableFrameVector[drawableFrameVector.size()-2]->indexImage.rows(), drawableFrameVector[drawableFrameVector.size()-2]->indexImage.cols());
	
    aligner.setProjector(&drawableFrameVector[drawableFrameVector.size()-2]->projector);
    aligner.setReferenceFrame(&drawableFrameVector[drawableFrameVector.size()-2]->frame);
    aligner.setCurrentFrame(&drawableFrameVector[drawableFrameVector.size()-1]->frame);
	
	aligner.setInitialGuess(initialGuess);
	aligner.setSensorOffset(sensorOffset);
	
	aligner.align();
	
	localT.push_back(aligner.T());
	globalT = globalT * localT[localT.size()-1];
      }
      cout << "Local transformation: " << endl << localT[localT.size()-1].matrix() << endl;

      // Update cloud drawing position.
      drawableFrameVector[drawableFrameVector.size()-1]->dPoints->setTransformation(globalT);
      drawableFrameVector[drawableFrameVector.size()-1]->dNormals->setTransformation(globalT);
      drawableFrameVector[drawableFrameVector.size()-1]->dCovariances->setTransformation(globalT);
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setReferencePointsTransformation(globalT * localT[localT.size()-1].inverse());
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setTransformation(globalT);
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setReferencePoints(&drawableFrameVector[drawableFrameVector.size()-2]->frame.points());
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setCurrentPoints(&drawableFrameVector[drawableFrameVector.size()-1]->frame.points());
      drawableFrameVector[drawableFrameVector.size()-1]->correspondences = CorrespondenceVector(aligner.correspondenceFinder()->correspondences());
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setCorrespondences(&drawableFrameVector[drawableFrameVector.size()-1]->correspondences);
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setNumCorrespondences(aligner.correspondenceFinder()->numCorrespondences());

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
    // Step-by-step optimization.
    else if(drawableFrameVector.size() > 1 && *optimizeViewer && *stepByStepViewer) {
      if(newCloudAdded)
	stepByStepInit = globalT;
      if(!wasInitialGuess) {
	aligner.setOuterIterations(1);

    aligner.correspondenceFinder()->setSize(drawableFrameVector[drawableFrameVector.size()-2]->indexImage.rows(), drawableFrameVector[drawableFrameVector.size()-2]->indexImage.cols());
	
    aligner.setProjector(&drawableFrameVector[drawableFrameVector.size()-2]->projector);
    aligner.setReferenceFrame(&drawableFrameVector[drawableFrameVector.size()-2]->frame);
    aligner.setCurrentFrame(&drawableFrameVector[drawableFrameVector.size()-1]->frame);
	
	if(newCloudAdded)
	  aligner.setInitialGuess(initialGuess);
	else
	  aligner.setInitialGuess(localT[localT.size()-1]);
	aligner.setSensorOffset(sensorOffset);
	
	aligner.align();
	
	if(newCloudAdded)
	  localT.push_back(aligner.T());
	else
	  localT[localT.size()-1] = aligner.T();
	  
	globalT = stepByStepInit * localT[localT.size()-1];
      }
      cout << "Local transformation: " << endl << aligner.T().matrix() << endl;

      // Update cloud drawing position.
      drawableFrameVector[drawableFrameVector.size()-1]->dPoints->setTransformation(globalT);
      drawableFrameVector[drawableFrameVector.size()-1]->dNormals->setTransformation(globalT);
      drawableFrameVector[drawableFrameVector.size()-1]->dCovariances->setTransformation(globalT);
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setReferencePointsTransformation(globalT * localT[localT.size()-1].inverse());
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setTransformation(globalT);
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setReferencePoints(&drawableFrameVector[drawableFrameVector.size()-2]->frame.points());
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setCurrentPoints(&drawableFrameVector[drawableFrameVector.size()-1]->frame.points());
      drawableFrameVector[drawableFrameVector.size()-1]->correspondences = CorrespondenceVector(aligner.correspondenceFinder()->correspondences());
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setCorrespondences(&drawableFrameVector[drawableFrameVector.size()-1]->correspondences);
      drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences->setNumCorrespondences(aligner.correspondenceFinder()->numCorrespondences());

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
    drawableFrame = new DrawableFrame(itemList->text().toStdString(), vz_step);
    drawableFrame->computeStats();
    drawableFrameVector.push_back(drawableFrame);
    drawableFrame = 0;
	// Add drawable items.
    drawableFrameVector[drawableFrameVector.size()-1]->dPoints->setTransformation(globalT);;
    drawableFrameVector[drawableFrameVector.size()-1]->dNormals->setTransformation(globalT);;
    drawableFrameVector[drawableFrameVector.size()-1]->dCovariances->setTransformation(globalT);;
    pwnGMW.viewer_3d->addDrawable((Drawable*)drawableFrameVector[drawableFrameVector.size()-1]->dPoints);
    pwnGMW.viewer_3d->addDrawable((Drawable*)drawableFrameVector[drawableFrameVector.size()-1]->dNormals);
    pwnGMW.viewer_3d->addDrawable((Drawable*)drawableFrameVector[drawableFrameVector.size()-1]->dCovariances);
    pwnGMW.viewer_3d->addDrawable((Drawable*)drawableFrameVector[drawableFrameVector.size()-1]->dCorrespondences);
      }
      newCloudAdded = true;
      *addCloud = 0;
    }
    // clear buttons pressed.
    else if(*clearAll) {
      pwnGMW.viewer_3d->clearDrawableList();
      for(size_t i = 0; i < drawableFrameVector.size(); i++)
    delete(drawableFrameVector[i]);
      drawableFrameVector.clear();
      globalT = Isometry3f::Identity();
      localT.clear();
      refScn->clear();
      currScn->clear();
      wasInitialGuess = false;
      newCloudAdded = false;
      *clearAll = 0;
    }
    else if(*clearLast) {
      if(drawableFrameVector.size() > 0) {
        pwnGMW.viewer_3d->popBack();
        pwnGMW.viewer_3d->popBack();
        pwnGMW.viewer_3d->popBack();
        pwnGMW.viewer_3d->popBack();
    delete(drawableFrameVector[drawableFrameVector.size()-1]);
    drawableFrameVector.pop_back();
      }
      if(localT.size() > 0) { 
	globalT = globalT * localT[localT.size()-1].inverse();
	localT.pop_back();
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
