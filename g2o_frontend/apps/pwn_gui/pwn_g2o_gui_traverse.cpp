#include "g2o_frontend/traversability/traversability_analyzer.h"
#include <qapplication.h>
#include "pwn_gui_main_window.h"
#include "viewer_state.h"


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


    //Define the parametesr needed for the traversability
    int maxAngle = 30;
    double cellWidth = 0.04;
    double maxTravHeight = 0.15;
    double robotHeight = 1;

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

    //TODO: add new parameters that will set angle etc. 
    arg.param("maxAngle", maxAngle, 30, "Specify maximal traversible angle for the robot in degrees. [int]");
    arg.param("cellWidth", cellWidth, 0.04, "Specify the width of the cell in meters. [double]");
    arg.param("maxTraversableHeight", maxTravHeight, 0.15, "Specify maximal traversable height in meters. [double]");
    arg.param("robotHeight", robotHeight, 1, "Specify the robot height in meters. [double]");

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

    //create an object of traversability analyzer
    TraversabilityAnalyzer traversabilityAnalyzer(maxAngle, cellWidth, maxTravHeight, robotHeight);
    
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

        
        //TODO: write down the vector<int> traversability
        //each element corresponds to point
        // traversabilityAnalyzer.createTraversabilityVector(points, normals, vFrame->_traversabilityVector);
        // std::cerr<<"traversability from main "<< traversabilityAnalyzer.getTraversabilityVector()->size()<<std::endl;
        // std::cerr<<"traversability vFrame from main "<< vFrame->_traversabilityVector->size()<<std::endl;
        vFrame->constructDrawableObjects();

        viewerState._meHasNewFrame = false;
      }


      usleep(10000);
    }
    return 0;  
  }



