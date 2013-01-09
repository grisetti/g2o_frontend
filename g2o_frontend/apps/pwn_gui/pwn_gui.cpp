#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <string>
#include <set>

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
#include "g2o_frontend/pwn_viewer/pwn_gui_main_window.h"


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
  Frame(){
    transform.setIdentity();
  }

  DepthImage depthImage;
  MatrixXi indexImage;
  PointWithNormalVector points;
  PointWithNormalSVDVector svds;
  MatrixXf zBuffer;
  Eigen::Isometry3f& transform;

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

int
 main(int argc, char** argv){
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

  PointWithNormalStatistcsGenerator normalGenerator;
  PointWithNormalAligner aligner;

  g2o::CommandArgs arg;
  arg.param("ng_step",ng_step,normalGenerator.step(),"compute a normal each x pixels") ;
  arg.param("ng_minPoints",ng_minPoints,normalGenerator.minPoints(),"minimum number of points in a region to compute the normal");
  arg.param("ng_imageRadius",ng_imageRadius,normalGenerator.imageRadius(), "radius of a ball in the works where to compute the normal for a pixel");
  arg.param("ng_worldRadius",ng_worldRadius,  normalGenerator.worldRadius(), "radius of a ball in the works where to compute the normal for a pixel");
  arg.param("ng_maxCurvature",ng_maxCurvature, normalGenerator.maxCurvature(), "above this threshold the normal is not computed");
  
  arg.param("al_inlierDistance", al_inlierDistance, aligner.inlierDistanceThreshold(),  "max metric distance between two points to regard them as iniliers");
  arg.param("al_inlierCurvatureRatio", al_inlierCurvatureRatio, aligner.inlierCurvatureRatioThreshold(), "max metric distance between two points to regard them as iniliers");
  arg.param("al_inlierNormalAngle", al_inlierNormalAngle, aligner.inlierNormalAngularThreshold(), "max metric distance between two points to regard them as iniliers");
  arg.param("al_inlierMaxChi2", al_inlierMaxChi2, aligner.inlierMaxChi2(), "max metric distance between two points to regard them as iniliers");
  arg.param("al_minInliers", al_minInliers, aligner.minInliers(), "minimum numver of inliers to do the matching");
  arg.param("al_scale", al_scale, aligner.scale(), "scale of the range image for the alignment");
  arg.param("al_flatCurvatureThreshold", al_flatCurvatureThreshold, aligner.flatCurvatureThreshold(), "curvature above which the patches are not considered planar");
  arg.param("al_outerIterations", al_outerIterations, aligner.outerIterations(), "outer interations (incl. data association)");
  arg.param("al_linearIterations", al_linearIterations, aligner.linearIterations(), "linear iterations for each outer one (uses R,t)");
  arg.param("al_nonLinearIterations", al_nonLinearIterations, aligner.nonLinearIterations(), "nonlinear iterations for each outer one (uses q,t)");
  arg.param("al_lambda", al_lambda, aligner.lambda(), "damping factor for the transformation update, the higher the smaller the step");
  arg.param("al_debug", al_debug, aligner.debug(), "prints lots of stuff");


  std::vector<string> fileNames;

  arg.paramLeftOver("image0", imageName0, "", "image0", true);
  arg.paramLeftOver("image1", imageName1, "", "image1", true);
  arg.parseArgs(argc, argv);

  std::vector<string> filenames;
  if (imageName0 == "sequence") {
    std::set<string> filenamesset = readDir(imageName1);
    for(set<string>::const_iterator it =filenamesset.begin(); it!=filenamesset.end(); it++) {
      filenames.push_back(*it);
   }
  } else {
    filenames.push_back(imageName0);
    filenames.push_back(imageName1);
  }

  QApplication qApplication(argc, argv);
  QGraphicsScene *scn0, *scn1;

  
  normalGenerator.setStep(ng_step);
  normalGenerator.setMinPoints(ng_minPoints);
  normalGenerator.setImageRadius(ng_imageRadius);
  normalGenerator.setWorldRadius(ng_worldRadius);
  normalGenerator.setMaxCurvature(ng_maxCurvature);
  
  aligner.setInlierDistanceThreshold(al_inlierDistance);
  aligner.setInlierCurvatureRatioThreshold(al_inlierCurvatureRatio);
  aligner.setInlierNormalAngularThreshold(al_inlierNormalAngle);
  aligner.setInlierMaxChi2(al_inlierMaxChi2);
  aligner.setMinInliers(al_minInliers);
  aligner.setScale(al_scale);
  aligner.setFlatCurvatureThreshold(al_flatCurvatureThreshold);
  aligner.setOuterIterations(al_outerIterations);
  aligner.setLinearIterations(al_linearIterations);
  aligner.setNonLinearIterations(al_nonLinearIterations);
  aligner.setLambda(al_lambda);
  aligner.setDebug(al_debug);



  std::vector<Frame*> frames;
  Frame* referenceFrame= 0;

  for (size_t i=0; i<filenames.size(); i++){
    cerr << "proecessing image" << filenames[i] << endl;
    Frame* currentFrame= new Frame();
    if(!currentFrame->load(filenames[i])) {
      cerr << "failure in loading image: " << filenames[i] << ", skipping" << endl;
      continue;
    }

  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;

  cerr << "there are " << filenames.size() << " files  in the pool" << endl; 
  FramePtrVector frames=loadFrames(filenames,normalGenerator,cameraMatrix);

  Isometry3f trajectory;
  trajectory.setIdentity();
  for (size_t i=0; i<filenames.size(); i++){
    cerr << "proecessing image" << filenames[i] << endl;
    Frame* currentFrame= new Frame();
    if(!currentFrame->load(filenames[i])) {
      cerr << "failure in loading image: " << filenames[i] << ", skipping" << endl;
      continue;
    }
    currentFrame->computeStats(normalGenerator,cameraMatrix);
    if (referenceFrame) {
      referenceFrame->setAligner(aligner, true);
      currentFrame->setAligner(aligner, false);

      aligner.setImageSize(currentFrame->depthImage.rows(), currentFrame->depthImage.cols());
      Eigen::Isometry3f X;
      X.setIdentity();
      double ostart = get_time();
      float error;
      int result = aligner.align(error, X);
      cerr << "inliers=" << result << " error/inliers: " << error/result << endl;
      cerr << "localTransform : " << endl;
      cerr << X.inverse().matrix() << endl;
      trajectory=trajectory*X;
      cerr << "globaltransform: " << endl;
      cerr << trajectory.matrix() << endl;
      double oend = get_time();
      cerr << "alignment took: " << oend-ostart << " sec." << endl;
      cerr << "aligner scaled image size: " << aligner.scaledImageRows() << " " << aligner.scaledImageCols() << endl;

    }
    Vector6f t=t2v(trajectory);
    ts << t.transpose() << endl;
    if (referenceFrame)
      delete referenceFrame;

    char buf [1024];
    int fnum = (int) i;
    sprintf(buf, "in-%05d.dat", fnum);
    currentFrame->points.save(buf, 50);
    cerr << "saving " << currentFrame->points.size() << " in file " <<  buf << endl;
    sprintf(buf, "out-%05d.dat", fnum);
    PointWithNormalVector pv2 = trajectory*currentFrame->points;
    pv2.save(buf, 50);
    cerr << "saving " << pv2.size() << " in file " <<  buf << endl;
    
    referenceFrame = currentFrame;
  }

  return 0;

  
}


#if 0

  /**** REGISTRATION VISUALIZATION****/
  QApplication qApplication(argc, argv);
  QGraphicsScene *scn0, *scn1;
  ColorMap cmap;
  cmap.compute(0, 7000, 0xff);
  QString filename0 = "cloud0.pgm";
  QString filename1 = "cloud1.pgm";  

  DMMainWindow dmMW;
  dmMW.show();
  scn0 = dmMW.scene0();
  scn1 = dmMW.scene1();
  bool *initialGuessViewer = 0, *optimizeViewer = 0;
  int *stepViewer = 0, *stepByStepViewer = 0;
  float *pointsViewer = 0, *normalsViewer = 0, *covariancesViewer = 0, *correspondencesViewer = 0;
  GLParameterPoints *p0Param = new GLParameterPoints();
  GLParameterPoints *p1Param = new GLParameterPoints();
  GLParameterNormals *n0Param = new GLParameterNormals();
  GLParameterNormals *n1Param = new GLParameterNormals();
  GLParameterCovariances *c0Param = new GLParameterCovariances();
  GLParameterCovariances *c1Param = new GLParameterCovariances();
  GLParameterCorrespondences *corrParam = new GLParameterCorrespondences();
  DrawablePoints* dp0 = new DrawablePoints(Isometry3f::Identity(), (GLParameter*)p0Param, 1, &cloud0);
  DrawablePoints* dp1 = new DrawablePoints(T1_0.inverse(), (GLParameter*)p1Param, 1, &cloud1);
  DrawableNormals *dn0 = new DrawableNormals(Isometry3f::Identity(), (GLParameter*)n0Param, 1, &cloud0);
  DrawableNormals *dn1 = new DrawableNormals(T1_0.inverse(), (GLParameter*)n1Param, 1, &cloud1);
  DrawableCovariances *dc0 = new DrawableCovariances(Isometry3f::Identity(), (GLParameter*)c0Param, 1, &svd0);
  DrawableCovariances *dc1 = new DrawableCovariances(T1_0.inverse(), (GLParameter*)c1Param, 1, &svd1);
  DrawableCorrespondences* dcorr = new DrawableCorrespondences(T1_0.inverse(), (GLParameter*)corrParam, 1, &correspondences);
  while (!(*dmMW.closing())) {
    qApplication.processEvents();
    
    // Update state variables value.
    initialGuessViewer = dmMW.initialGuess();
    optimizeViewer = dmMW.optimize();
    stepByStepViewer = dmMW.stepByStep();
    stepViewer = dmMW.step();
    pointsViewer = dmMW.points();
    normalsViewer = dmMW.normals();
    covariancesViewer = dmMW.covariances();
    correspondencesViewer = dmMW.correspondences();
    
    // Checking state variable value.
    // Initial guess.
    if (*initialGuessViewer) {
      *initialGuessViewer = 0;
      T1_0 = Isometry3f::Identity();
    }
    // Registration.
    else if(*optimizeViewer && !(*stepByStepViewer)) {
      *optimizeViewer = 0;
      computeRegistration(T1_0, img1, 
			  cloud0, svd0, correspondences, 
			  cloud0PtrScaled, cloud1PtrScaled,
			  omega0PtrScaled,
			  svd0PtrScaled, svd1PtrScaled,
			  corrOmegas1, corrP0, corrP1,
			  omega0, zBuffer, cameraMatrixScaled,
			  _r, _c,
			  outerIterations, innerIterations, doLinear);
      scn0->clear();
      scn1->clear();
      QImage qImage0(filename0);
      QImage qImage1(filename1);
      toQImage(qImage0, img0, cmap);
      toQImage(qImage1, img1, cmap);
      scn0->addPixmap((QPixmap::fromImage(qImage0)).scaled(QSize((int)_c/(scale*3), (int)(_r/(scale*3)))));
      scn1->addPixmap((QPixmap::fromImage(qImage1)).scaled(QSize((int)_c/(scale*3), (int)(_r/(scale*3)))));
      dmMW.graphicsView1_2d->show();
      dmMW.graphicsView2_2d->show();
    }
    // Step by step registration.
    else if(*optimizeViewer && *stepByStepViewer) {
      *optimizeViewer = 0;
      computeRegistration(T1_0, img1,
			  cloud0, svd0, correspondences, 
			  cloud0PtrScaled, cloud1PtrScaled,
			  omega0PtrScaled,
			  svd0PtrScaled, svd1PtrScaled,
			  corrOmegas1, corrP0, corrP1,
			  omega0, zBuffer, cameraMatrixScaled,
			  _r, _c,
			  1, innerIterations, doLinear);
      scn0->clear();
      scn1->clear();
      QImage qImage0(filename0);
      QImage qImage1(filename1);
      toQImage(qImage0, img0, cmap);
      toQImage(qImage1, img1, cmap);
      scn0->addPixmap((QPixmap::fromImage(qImage0)).scaled(QSize((int)_c/(scale*3), (int)(_r/(scale*3)))));
      scn1->addPixmap((QPixmap::fromImage(qImage1)).scaled(QSize((int)_c/(scale*3), (int)(_r/(scale*3)))));
      dmMW.graphicsView1_2d->show();
      dmMW.graphicsView2_2d->show();
    }
    
    dp1->setTransformation(T1_0.inverse());
    dn1->setTransformation(T1_0.inverse());
    dc1->setTransformation(T1_0.inverse());
    dcorr->setTransformation(T1_0.inverse());

    dmMW.viewer_3d->clearDrawableList();
    if (pointsViewer[0]) {
      p0Param->setColor(Vector4f(0.0f, 1.0f, 0.0f, 0.5f));
      p0Param->setPointSize(pointsViewer[1]);
      p1Param->setPointSize(pointsViewer[1]);
      if (stepViewer[0]) {
	dp0->setStep(stepViewer[1]);
	dp1->setStep(stepViewer[1]);
      }
      dmMW.viewer_3d->addDrawable((Drawable*)dp0);
      dmMW.viewer_3d->addDrawable((Drawable*)dp1);
    }
    if (normalsViewer[0]) {
      n0Param->setNormalLength(normalsViewer[1]);
      n1Param->setNormalLength(normalsViewer[1]);
      if (stepViewer[0]) {
	dn0->setStep(stepViewer[1]);
	dn1->setStep(stepViewer[1]);
      }
      dmMW.viewer_3d->addDrawable((Drawable*)dn0);
      dmMW.viewer_3d->addDrawable((Drawable*)dn1);
    }
    if (covariancesViewer[0]) {
      c0Param->setEllipsoidScale(covariancesViewer[1]);
      c1Param->setEllipsoidScale(covariancesViewer[1]);
      if (stepViewer[0]) {
	dc0->setStep(stepViewer[1]);
	dc1->setStep(stepViewer[1]);
      }
      dmMW.viewer_3d->addDrawable((Drawable*)dc0);
      dmMW.viewer_3d->addDrawable((Drawable*)dc1);
    }
    if (correspondencesViewer[0]) {
      corrParam->setLineWidth(correspondencesViewer[1]);
      if (stepViewer[0])
	dcorr->setStep(stepViewer[1]);	
      dmMW.viewer_3d->addDrawable((Drawable*)dcorr);
    }
    dmMW.viewer_3d->updateGL();
    usleep(10000);
  }
  dmMW.viewer_3d->clearDrawableList();
 
  delete(p0Param);
  delete(p1Param);
  delete(n0Param);
  delete(n1Param);
  delete(c0Param);
  delete(c1Param);
  delete(corrParam);
  delete(dp0);
  delete(dp1);
  delete(dn0);
  delete(dn1);
  delete(dc0);
  delete(dc1);
  delete(dcorr);

#endif
