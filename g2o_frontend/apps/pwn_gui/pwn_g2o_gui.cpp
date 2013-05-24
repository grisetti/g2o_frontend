#include "pwn_gui_main_window.h"
#include "viewer_state.h"
#include <signal.h>
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include <qapplication.h>
#include <iostream>
#include <fstream>
#include <set>
#include <sys/stat.h>

#include <unistd.h>

using namespace Eigen;
using namespace g2o;
using namespace std;
using namespace pwn;


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
