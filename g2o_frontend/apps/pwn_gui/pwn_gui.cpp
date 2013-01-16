#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <qapplication.h>
#include "g2o/stuff/command_args.h"
#include "g2o_frontend/pwn/depthimage.h"
#include "g2o_frontend/pwn/pointwithnormal.h"
#include "g2o_frontend/pwn/pointwithnormalstatsgenerator.h"
#include "g2o_frontend/pwn/pointwithnormalaligner.h"
#include "g2o/stuff/timeutil.h"
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

using namespace Eigen;
using namespace g2o;
using namespace std;

set<string> readDir(std::string dir){
  DIR *dp;
  struct dirent *dirp;
  struct stat filestat;
  std::set<std::string> filenames;
  dp = opendir( dir.c_str() );
  if (dp == NULL){
    return filenames;
  }
  
  while ((dirp = readdir( dp ))) {
    string filepath = dir + "/" + dirp->d_name;

    // If the file is a directory (or is in some way invalid) we'll skip it 
    if (stat( filepath.c_str(), &filestat )) continue;
    if (S_ISDIR( filestat.st_mode ))         continue;

    filenames.insert(filepath);
  }

  closedir( dp );
  return filenames;
}

struct Frame{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Frame() {
    transform.setIdentity();
  }

  DepthImage depthImage;
  MatrixXi indexImage;
  PointWithNormalVector points;
  PointWithNormalSVDVector svds;
  MatrixXf zBuffer;
  Eigen::Isometry3f transform;

  bool load(std::string filename) {
    return depthImage.load(filename.c_str());
  }
  void computeStats(PointWithNormalStatistcsGenerator & generator, const Matrix3f& cameraMatrix){
    zBuffer.resize(depthImage.rows(), depthImage.cols());
    points.fromDepthImage(depthImage,cameraMatrix,Isometry3f::Identity());
    indexImage.resize(depthImage.rows(), depthImage.cols());
    points.toIndexImage(indexImage, zBuffer, cameraMatrix, Eigen::Isometry3f::Identity(), 10);
    cerr << "points: " << points.size() << endl; 
    svds.resize(points.size());
    generator.computeNormalsAndSVD(points, svds, indexImage, cameraMatrix);
  }
  void setAligner(PointWithNormalAligner& aligner, bool isRef){
    if (isRef) {
      aligner.setReferenceCloud(&points, &svds);
    } else {
      aligner.setCurrentCloud(&points, &svds);
    }
  }
};

typedef std::vector<Frame*> FramePtrVector;

std::vector<Frame*> loadFrames(const vector<string>& filenames, PointWithNormalStatistcsGenerator& normalGenerator, const Matrix3f& cameraMatrix){
  FramePtrVector fpv;
  for (size_t i=0; i<filenames.size(); i++){
    cerr << "proecessing image" << filenames[i] << endl;
    Frame* frame= new Frame();
    if(!frame->load(filenames[i])) {
      cerr << "failure in loading image: " << filenames[i] << ", skipping" << endl;
      continue;
    }
    frame->computeStats(normalGenerator,cameraMatrix);
    fpv.push_back(frame);
  }
  return fpv;
}

int main(int argc, char** argv){
  string imageName0;
  string imageName1;
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

  PointWithNormalStatistcsGenerator* normalGenerator =  new PointWithNormalStatistcsGenerator;
  PointWithNormalAligner* aligner = new PointWithNormalAligner;

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
  arg.param("al_minInliers", al_minInliers, aligner->minInliers(), "minimum number of inliers to do the matching");
  arg.param("al_scale", al_scale, aligner->scale(), "scale of the range image for the alignment");
  arg.param("al_flatCurvatureThreshold", al_flatCurvatureThreshold, aligner->flatCurvatureThreshold(), "curvature above which the patches are not considered planar");
  arg.param("al_outerIterations", al_outerIterations, aligner->outerIterations(), "outer interations (incl. data association)");
  arg.param("al_linearIterations", al_linearIterations, aligner->linearIterations(), "linear iterations for each outer one (uses R,t)");
  arg.param("al_nonLinearIterations", al_nonLinearIterations, aligner->nonLinearIterations(), "nonlinear iterations for each outer one (uses q,t)");
  arg.param("al_lambda", al_lambda, aligner->lambda(), "damping factor for the transformation update, the higher the smaller the step");
  arg.param("al_debug", al_debug, aligner->debug(), "prints lots of stuff");

  std::vector<string> fileNames;

  arg.paramLeftOver("image0", imageName0, "", "image0", true);
  arg.paramLeftOver("image1", imageName1, "", "image1", true);
  arg.parseArgs(argc, argv);

  QApplication qApplication(argc, argv);
  PWNGuiMainWindow pwnGMW;
  QGraphicsScene *refScn, *currScn;

  std::vector<string> filenames;
  if (imageName0 == "sequence") {
    std::set<string> filenamesset = readDir(imageName1);
    for(set<string>::const_iterator it =filenamesset.begin(); it!=filenamesset.end(); it++) {
      filenames.push_back(*it);      
      QString listItem(&(*it)[0]);
      if(listItem.endsWith("depth.pgm",Qt::CaseInsensitive))
	pwnGMW.listWidget->addItem(listItem);
    }
  } else {
    filenames.push_back(imageName0);
    filenames.push_back(imageName1);
  }

  normalGenerator->setStep(ng_step);
  normalGenerator->setMinPoints(ng_minPoints);
  normalGenerator->setImageRadius(ng_imageRadius);
  normalGenerator->setWorldRadius(ng_worldRadius);
  normalGenerator->setMaxCurvature(ng_maxCurvature);
  
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

  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  
  Isometry3f trajectory;
  trajectory.setIdentity();

  pwnGMW.show();
  refScn = pwnGMW.scene0();
  currScn = pwnGMW.scene1();
  bool *addCloud = 0, *initialGuessViewer = 0, *optimizeViewer = 0, *clearLast = 0, *clearAll = 0;
  int *stepViewer = 0, *stepByStepViewer = 0;
  float *pointsViewer = 0, *normalsViewer = 0, *covariancesViewer = 0, *correspondencesViewer = 0;
  QListWidgetItem* itemList = 0;
  GLParameterPoints *pParam = new GLParameterPoints();
  GLParameterNormals *nParam = new GLParameterNormals(0.1f, Vector4f(0.0f, 0.0f, 1.0f, 0.5f), 0.0f);
  GLParameterCovariances *cParam = new GLParameterCovariances(0.5f, 
							      Vector4f(0.0f, 1.0f, 0.0f, 0.5f), Vector4f(1.0f, 0.0f, 0.0f, 0.5f),
							      0.02f, 0.0f);
  GLParameterCorrespondences *corrParam = new GLParameterCorrespondences(0.1f, Vector4f(1.0f, 0.0f, 1.0f, 0.5f), 0.0f);
  Isometry3f tmp = Isometry3f::Identity(), startingTraj = Isometry3f::Identity();
  Frame *referenceFrame = 0, *currentFrame = 0;
  bool firstCloudAdded = true, wasInitialGuess = true;
  while (!(*pwnGMW.closing())) {
    qApplication.processEvents();

    // Update state variables value.
    initialGuessViewer = pwnGMW.initialGuess();
    optimizeViewer = pwnGMW.optimize();
    clearLast = pwnGMW.clearLast();    
    clearAll = pwnGMW.clearAll();
    stepByStepViewer = pwnGMW.stepByStep();
    stepViewer = pwnGMW.step();
    pointsViewer = pwnGMW.points();
    normalsViewer = pwnGMW.normals();
    covariancesViewer = pwnGMW.covariances();
    correspondencesViewer = pwnGMW.correspondences();
    addCloud = pwnGMW.addCloud();
    itemList = pwnGMW.itemList();
    vector<Drawable*> drawableList = pwnGMW.viewer_3d->drawableList();  

    // Handle cloud insert.
    if(*addCloud) {
      if(itemList) {
	if(!firstCloudAdded)
	  referenceFrame = currentFrame;
	else
	  firstCloudAdded = false;
	// Load cloud.
	currentFrame = new Frame();
	if(!currentFrame->load(itemList->text().toStdString())) {
	  cerr << "failure in loading image: " << itemList->text().toStdString() << ", skipping" << endl;
	  *addCloud = 0;
	  continue;
	}
	currentFrame->computeStats(*normalGenerator, cameraMatrix);
	DrawablePoints* dp = new DrawablePoints(Isometry3f::Identity(), (GLParameter*)pParam, 1, &(*currentFrame).points);
	DrawableNormals *dn = new DrawableNormals(Isometry3f::Identity(), (GLParameter*)nParam, 1, &(*currentFrame).points);
	DrawableCovariances *dc = new DrawableCovariances(Isometry3f::Identity(), (GLParameter*)cParam, 1, &(*currentFrame).svds);
  	DrawableCorrespondences* dcorr = new DrawableCorrespondences(Isometry3f::Identity(), (GLParameter*)corrParam, 1, 0, 0);
	if(drawableList.size() != 0) {
	  DrawableCorrespondences* dcorr = (DrawableCorrespondences*)drawableList[drawableList.size()-1];
	  dcorr->setPoints1(0);
	  dcorr->setPoints2(0);
	  dcorr->setCorrespondences(0);
	  dcorr->setNumCorrespondences(0);
	}
	pwnGMW.viewer_3d->addDrawable((Drawable*)dp);
	pwnGMW.viewer_3d->addDrawable((Drawable*)dn);
	pwnGMW.viewer_3d->addDrawable((Drawable*)dc);
	pwnGMW.viewer_3d->addDrawable((Drawable*)dcorr);	
	startingTraj = trajectory;
      }
      *addCloud = 0;
    }

    // Check feature visualization options.
    for(size_t i = 0; i < drawableList.size(); i++) {
      if(stepViewer[0])
	drawableList[i]->setStep(stepViewer[1]);
    }
  
    if(pointsViewer[0])
      pParam->setPointSize(pointsViewer[1]);
    else
      pParam->setPointSize(0.0f);
    if(normalsViewer[0])
         nParam->setNormalLength(normalsViewer[1]);
    else
      nParam->setNormalLength(0.0f);
    if(covariancesViewer[0])
      cParam->setEllipsoidScale(covariancesViewer[1]);
    else
      cParam->setEllipsoidScale(0.0f);
    if (correspondencesViewer[0])
      corrParam->setLineWidth(correspondencesViewer[1]);
    else
      corrParam->setLineWidth(0.0f);

    // Initial guess pressed.
    if(*initialGuessViewer) {
      *initialGuessViewer = 0;
      if(referenceFrame) {
	drawableList[drawableList.size()-1]->setTransformation(Isometry3f::Identity());
	drawableList[drawableList.size()-2]->setTransformation(Isometry3f::Identity());
	drawableList[drawableList.size()-3]->setTransformation(Isometry3f::Identity());
	drawableList[drawableList.size()-4]->setTransformation(Isometry3f::Identity());
	wasInitialGuess = true;
      }
    }
    // Optimize pressed no step by step mode.
    else if(*optimizeViewer && !(*stepByStepViewer)) {
      *optimizeViewer = 0;
      if(referenceFrame) {
	aligner->setOuterIterations(al_outerIterations);
	referenceFrame->setAligner(*aligner, true);
	currentFrame->setAligner(*aligner, false);
	aligner->setImageSize(currentFrame->depthImage.rows(), currentFrame->depthImage.cols());
	Eigen::Isometry3f X;
	X.setIdentity();
	double ostart = get_time();
	float error;
	int result = aligner->align(error, X);
	
	cerr << "inliers=" << result << " error/inliers: " << error/result << endl;
	cerr << "localTransform : " << endl;
	cerr << X.inverse().matrix() << endl;
	if(wasInitialGuess) {
	  trajectory = startingTraj;
	  wasInitialGuess = false;
	}
	trajectory=startingTraj*X;

	
	cerr << "globaltransform: " << endl;
	cerr << trajectory.matrix() << endl;
	double oend = get_time();
	cerr << "alignment took: " << oend-ostart << " sec." << endl;
	cerr << "aligner scaled image size: " << aligner->scaledImageRows() << " " << aligner->scaledImageCols() << endl;
	DrawableCorrespondences* dcorr = (DrawableCorrespondences*)drawableList[drawableList.size()-1];
	dcorr->setPoints1(aligner->refPoints());
	dcorr->setPoints2(aligner->currPoints());
	dcorr->setCorrespondences(&aligner->correspondences());
	dcorr->setNumCorrespondences(aligner->numCorrespondences());
	drawableList[drawableList.size()-1]->setTransformation(trajectory);
	drawableList[drawableList.size()-2]->setTransformation(trajectory);
	drawableList[drawableList.size()-3]->setTransformation(trajectory);
	drawableList[drawableList.size()-4]->setTransformation(trajectory);	
	// Show zBuffers.
	int _r = aligner->scaledImageRows();
	int _c = aligner->scaledImageCols();
	refScn->clear();
	currScn->clear();
	QImage refQImage;
	QImage currQImage;
	DepthImageView div;
	div.convertToQImage(refQImage, aligner->refZBuffer()); 
	div.convertToQImage(currQImage, aligner->currZBuffer());
	refScn->addPixmap((QPixmap::fromImage(refQImage)).scaled(QSize((int)_c/(aligner->scale()*3), (int)(_r/(aligner->scale()*3)))));
	currScn->addPixmap((QPixmap::fromImage(currQImage)).scaled(QSize((int)_c/(aligner->scale()*3), (int)(_r/(aligner->scale()*3)))));
	pwnGMW.graphicsView1_2d->show();
	pwnGMW.graphicsView2_2d->show();
      }
    }
    // Optimize pressed with step by step mode.
    else if(*optimizeViewer && *stepByStepViewer) {
      *optimizeViewer = 0;
      if(referenceFrame) {
	aligner->setOuterIterations(1);
	referenceFrame->setAligner(*aligner, true);
	currentFrame->setAligner(*aligner, false);
	aligner->setImageSize(currentFrame->depthImage.rows(), currentFrame->depthImage.cols());
	Eigen::Isometry3f X;
	if(wasInitialGuess) {
	  trajectory = startingTraj;
	  X.setIdentity();
	  wasInitialGuess = false;
	}
	else
	  X = tmp;
	double ostart = get_time();
	float error;
	int result = aligner->align(error, X);
	
	tmp = X;
	cerr << "inliers=" << result << " error/inliers: " << error/result << endl;
	cerr << "localTransform : " << endl;
	cerr << X.inverse().matrix() << endl;
	trajectory=startingTraj*X;
	
	cerr << "globaltransform: " << endl;
	cerr << trajectory.matrix() << endl;
	double oend = get_time();
	cerr << "alignment took: " << oend-ostart << " sec." << endl;
	cerr << "aligner scaled image size: " << aligner->scaledImageRows() << " " << aligner->scaledImageCols() << endl;
	DrawableCorrespondences* dcorr = (DrawableCorrespondences*)drawableList[drawableList.size()-1];
	dcorr->setPoints1(aligner->refPoints());
	dcorr->setPoints2(aligner->currPoints());
	dcorr->setCorrespondences(&aligner->correspondences());
	dcorr->setNumCorrespondences(aligner->numCorrespondences());
	drawableList[drawableList.size()-1]->setTransformation(trajectory);
	drawableList[drawableList.size()-2]->setTransformation(trajectory);
	drawableList[drawableList.size()-3]->setTransformation(trajectory);
	drawableList[drawableList.size()-4]->setTransformation(trajectory);
	// Show zBuffers.
	int _r = aligner->scaledImageRows();
	int _c = aligner->scaledImageCols();
	refScn->clear();
	currScn->clear();
	QImage refQImage;
	QImage currQImage;
	DepthImageView div;
	div.convertToQImage(refQImage, aligner->refZBuffer()); 
	div.convertToQImage(currQImage, aligner->currZBuffer());
	refScn->addPixmap((QPixmap::fromImage(refQImage)).scaled(QSize((int)_c/(aligner->scale()*3), (int)(_r/(aligner->scale()*3)))));
	currScn->addPixmap((QPixmap::fromImage(currQImage)).scaled(QSize((int)_c/(aligner->scale()*3), (int)(_r/(aligner->scale()*3)))));
	pwnGMW.graphicsView1_2d->show();
	pwnGMW.graphicsView2_2d->show();
      }
    }
    // clear buttons pressed.
    else if(*clearAll) {
      pwnGMW.viewer_3d->clearDrawableList();
      trajectory = Isometry3f::Identity();
      startingTraj = trajectory;
      *clearAll = 0;
    }
    else if(*clearLast) {
      if(drawableList.size() >= 4) {	
        pwnGMW.viewer_3d->popBack();
        pwnGMW.viewer_3d->popBack();
        pwnGMW.viewer_3d->popBack();
        pwnGMW.viewer_3d->popBack();
      }
      *clearLast = 0;
    }
    pwnGMW.viewer_3d->updateGL();

    usleep(10000);
  }
  return 0;  
}
