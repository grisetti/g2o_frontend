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

int main(int argc, char** argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string workingDirectory = "./";

  // Variables for the input parameters. Just type on the command line
  // ./pwn_normal_extraction -h to have more details about them.
  float ng_scale = 1.0f;
  float ng_curvatureThreshold = 1.0f;
  int al_innerIterations = 5;
  int al_outerIterations = 5;
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
  arg.param("al_innerIterations", al_innerIterations, 5, "Specify the inner iterations. [int]");
  arg.param("al_outerIterations", al_outerIterations, 5, "Specify the outer iterations. [int]");
  arg.param("vz_step", vz_step, 5, "A graphic element is drawn each vz_step elements. [int]");

  // Last parameter has to be the working directory.
  arg.paramLeftOver("workingDirectory", workingDirectory, "./", "Path of the working directory. [string]", true);

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
  Isometry3f initialGuess = Isometry3f::Identity();
  Isometry3f sensorOffset = Isometry3f::Identity();
  Aligner aligner;
  aligner.setOuterIterations(al_outerIterations);
  aligner.setInnerIterations(al_innerIterations);
  aligner.setInitialGuess(initialGuess);
  aligner.setSensorOffset(sensorOffset);
    
  pwnGMW.show();
  refScn = pwnGMW.scene0();
  currScn = pwnGMW.scene1();

  while (!(*pwnGMW.closing())) {
    qApplication.processEvents();

    pwnGMW.viewer_3d->updateGL();

    usleep(10000);
  }
  return 0;  
}
