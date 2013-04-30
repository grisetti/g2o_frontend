#include "g2o_frontend/pwn/normalgenerator.h"
#include "g2o_frontend/pwn/omegagenerator.h"
#include "g2o_frontend/pwn/homogeneouspoint3fscene.h"
#include "g2o_frontend/pwn/pinholepointprojector.h"
#include "g2o_frontend/pwn/aligner.h"
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

#include <qapplication.h>
#include <iostream>
#include <fstream>
#include <set>
#include <dirent.h>
#include <sys/stat.h>

using namespace Eigen;
using namespace g2o;
using namespace std;

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

struct Frame {
  Frame(string f, int s) {
    filename = f;
    step = s;

    pPoints = new GLParameterPoints(1.0f, Vector4f(1.0f, 0.5f, 0.0f, 1.0f));
    pPoints->setStep(step);
    pNormals = new GLParameterNormals(1.0f, Vector4f(0.0f, 0.0f, 1.0f, 1.0f), 0.0f);
    pNormals->setStep(step);
    pCovariances = new GLParameterCovariances(1.0f, 
					      Vector4f(0.0f, 1.0f, 0.0f, 1.0f), Vector4f(1.0f, 0.0f, 0.0f, 1.0f),
					      0.02f, 0.0f);
    pCovariances->setStep(step);
    pCorrespondences = new GLParameterCorrespondences(1.0f, Vector4f(1.0f, 0.0f, 1.0f, 1.0f), 0.0f);
    pCorrespondences->setStep(step);

    dPoints = new DrawablePoints(Isometry3f::Identity(), (GLParameter*)pPoints, scene.points(), scene.normals());
    dNormals = new DrawableNormals(Isometry3f::Identity(), (GLParameter*)pNormals, scene.points(), scene.normals());
    dCovariances = new DrawableCovariances(Isometry3f::Identity(), (GLParameter*)pCovariances, scene.stats());
    dCorrespondences = new DrawableCorrespondences(Isometry3f::Identity(), (GLParameter*)pCorrespondences, 0,
						   scene.points(), scene.points(), correspondences);
  }
  
  void computeStats() {
    Eigen::Matrix3f cameraMatrix;
    cameraMatrix << 
      525.0f, 0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f, 0.0f, 1.0f;
    
    float ng_curvatureThreshold = 1.0f;

    if(!scene.depthImage().load(filename.c_str(), true)) {
      cout << "Failure while loading the depth image: " << filename<< " skipping image!" << endl;
      return;
    }
    cout << endl << "Loaded depth image " << filename << " of size: " << scene.depthImage().rows() << "x" << scene.depthImage().cols() << endl;
    
    /************************************************************************
     *                         Point Unprojection                           *
     ************************************************************************/
    cout << "Unprojecting points...";

    // Projector object.
    PinholePointProjector projector;

    // Update the size of the index image.
    scene.indexImage().resize(scene.depthImage().rows(), scene.depthImage().cols());
    
    // Set the camera matrix of the projector object.
    projector.setCameraMatrix(cameraMatrix);
  
    // Get the points in the 3d euclidean space.
    projector.unProject(scene.points(), scene.indexImage(), scene.depthImage());
    
    cout << " done." << endl;

    /************************************************************************
     *                         Normal Computation                           *
     ************************************************************************/
    cout << "Computing normals...";

    HomogeneousPoint3fIntegralImage integralImage;
    MatrixXi intervalImage;
  
    // Compute the integral images.
    integralImage.compute(scene.indexImage(), scene.points());
    
    // Compute the intervals.
    projector.projectIntervals(intervalImage, scene.depthImage(), 0.1f);
    
    // Resize the vector containing the stats to have the same length of the vector of points.
    scene.stats().resize(scene.points().size());
    std::fill(scene.stats().begin(), scene.stats().end(), HomogeneousPoint3fStats());
    
    // Creating the stas generator object. 
    HomogeneousPoint3fStatsGenerator statsGenerator;
  
    // Stats and normals computation.
    statsGenerator.compute(scene.normals(),
			   scene.stats(),
			   scene.points(),
			   integralImage,
			   intervalImage,
			   scene.indexImage(),
			   ng_curvatureThreshold);
    
    cout << " done." << endl;

    /************************************************************************
     *                         Omega Computation                            *
     ************************************************************************/
    cout << "Computing omegas...";

    // Creating the omegas generators objects.
    PointOmegaGenerator pointOmegaGenerator;
    NormalOmegaGenerator normalOmegaGenerator;
  
    // Omegas computation.
    pointOmegaGenerator.compute(scene.pointOmegas(), scene.stats(), scene.normals());
    normalOmegaGenerator.compute(scene.normalOmegas(), scene.stats(), scene.normals());

    cout << " done." << endl;

    dPoints->setPoints(scene.points());
    dPoints->setNormals(scene.normals());
    dNormals->setPoints(scene.points());
    dNormals->setNormals(scene.normals());
    dCovariances->setCovariances(scene.stats());
  }

  HomogeneousPoint3fScene scene;
  CorrespondenceVector correspondences;
  string filename;
  int step;

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
  string workingDirectory = "./";

  // Variables for the input parameters. Just type on the command line
  // ./pwn_normal_extraction -h to have more details about them.
  float ng_scale = 1.0f;
  float ng_curvatureThreshold = 1.0f;
  int al_innerIterations = 1;
  int al_outerIterations = 10;
  int vz_step = 1;

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
  arg.param("al_innerIterations", al_innerIterations, 5, "Specify the inner iterations. [int]");
  arg.param("al_outerIterations", al_outerIterations, 10, "Specify the outer iterations. [int]");
  arg.param("vz_step", vz_step, 1, "A graphic element is drawn each vz_step elements. [int]");

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

  // Creating the normal generator object and setting some parameters. 
  NormalGenerator normalGenerator;
  // Set the image scale.
  normalGenerator.setScale(ng_scale);
  // Set the camera matrix.
  normalGenerator.setCameraMatrix(cameraMatrix);

  // Creating the omegas generators objects.
  PointOmegaGenerator pointOmegaGenerator;
  NormalOmegaGenerator normalOmegaGenerator;

  // Creating and setting aligner object.
  Aligner aligner;
  aligner.setOuterIterations(al_outerIterations);
  aligner.setInnerIterations(al_innerIterations);
    
  pwnGMW.show();
  refScn = pwnGMW.scene0();
  currScn = pwnGMW.scene1();

  bool *addCloud = 0, *clearLast = 0, *clearAll = 0;;
  QListWidgetItem* itemList = 0;
  
  std::vector<Frame> frameVector;
  
  while (!(*pwnGMW.closing())) {
    qApplication.processEvents();

    // Check window status changes.
    addCloud = pwnGMW.addCloud();
    clearLast = pwnGMW.clearLast();    
    clearAll = pwnGMW.clearAll();
    itemList = pwnGMW.itemList();
    
    // Add cloud was pressed.
    if(*addCloud) {
      if(itemList) {
	Frame frame(itemList->text().toStdString(), vz_step);
	frame.computeStats();
	frameVector.push_back(frame);
	
	// Add drawable items.
	pwnGMW.viewer_3d->addDrawable((Drawable*)frameVector[frameVector.size()-1].dPoints);
	pwnGMW.viewer_3d->addDrawable((Drawable*)frameVector[frameVector.size()-1].dNormals);
	pwnGMW.viewer_3d->addDrawable((Drawable*)frameVector[frameVector.size()-1].dCovariances);
	pwnGMW.viewer_3d->addDrawable((Drawable*)frameVector[frameVector.size()-1].dCorrespondences);	
      }
      *addCloud = 0;
    }
    // clear buttons pressed.
    else if(*clearAll) {
      pwnGMW.viewer_3d->clearDrawableList();
      *clearAll = 0;
    }
    else if(*clearLast) {
      if(pwnGMW.viewer_3d->drawableList().size() >= 3) {	
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
