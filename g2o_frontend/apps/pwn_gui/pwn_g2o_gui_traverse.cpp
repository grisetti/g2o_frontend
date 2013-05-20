#include "pwn_gui_main_window.h"
#include "viewer_state.h"
#include <signal.h>
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"
#include "g2o_frontend/traversability/grid3d.h"
#include "g2o_frontend/traversability/grid2d.h"
#include "g2o_frontend/traversability/utils.h"


#include <qapplication.h>
#include <iostream>
#include <fstream>
#include <set>
#include <sys/stat.h>

using namespace Eigen;
using namespace g2o;
using namespace std;
using namespace pwn;

Grid2D globalMap;

int gMaxAngle = 30;
double gCellWidth = 0.04;
double gCellHeight = 0.04;
double gMaxTravHeight = 0.15;
double gRadiusNormals = 0.2;
double gRobotHeight = 1;
bool gDepthIsRaw = true;
bool gZPointsUp = true;
bool gMyCatacombs = true;
bool gOutputImage = true;
// string g2oFile = "/home/igor/Downloads/good_2013-02-14-14-45-01/s_priscilla_2013-02-14_imu2_rgbd_opt.g2o";
string g2oFile ;
// string imgDir = "/home/igor/Downloads/good_2013-02-14-14-45-01/"; // [Igor]
string imgDir; // [Olga] I think it makes here more sense

int counter = 0;
void processCloud(const PointVector& imagePoints,
                  const NormalVector& imageNormals) 
{ 
    Grid3D grid3d=Grid3D();
    Grid2D currentLocalMap2D = Grid2D();

    // cout<<"Point cloud adding..."<<endl;
    grid3d.addPointCloud(imagePoints, imageNormals);
    // cout<<"Point cloud was added"<<endl;

    grid3d.checkTraversability(currentLocalMap2D);
    // cout<<"Map was analysed."<<endl;

    globalMap.add(currentLocalMap2D);
    cerr << " :-) " << endl;
    cv::Mat map_image;
    if (++counter%20==0)
    {
      globalMap.showMap(map_image);

      /* save the image */
      cv::cvtColor(map_image, map_image, CV_RGB2BGR);
      cv::imwrite("boh.jpg", map_image);
      //cout<<"image written to "<<outputImage<<endl;
      //cv::imshow( "Display window", map_image );       // Show our image inside it.
    }
}



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
    
    if (viewerState._meHasNewFrame){
      DrawableFrame* vFrame=viewerState.drawableFrameVector.back();
      Frame* frame=vFrame->_frame;
      PointVector points=frame->points();
      NormalVector normals=frame->normals();

      points.transformInPlace(viewerState.globalT);
      normals.transformInPlace(viewerState.globalT);
      processCloud(points,normals);
      viewerState._meHasNewFrame = false;
    }


    usleep(10000);
  }
  return 0;  
}
