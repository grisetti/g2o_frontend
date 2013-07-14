#include <iostream>
#include <fstream>

#include "g2o/stuff/command_args.h"

#include "g2o_frontend/pwn2/depthimage.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/cylindricalpointprojector.h"

using namespace std;
using namespace Eigen;
using namespace g2o;
using namespace pwn;

int main(int argc, char **argv) {
  int numImages = 0;
  string cloudFilename;
  string outputFilename = "totalFrame.pwn";

  // Input parameters handling.
  g2o::CommandArgs arg;
  arg.paramLeftOver("clouddFilename", cloudFilename, "", "input cloud", true);
 
  // Set parser input.
  arg.parseArgs(argc, argv);

  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;

  
  PinholePointProjector projector;
  projector.setCameraMatrix(cameraMatrix);
  StatsCalculator statsCalculator;
  PointInformationMatrixCalculator pointInformationMatrixCalculator;
  NormalInformationMatrixCalculator normalInformationMatrixCalculator;
  DepthImageConverter depthImageConverter(&projector, &statsCalculator,
  					  &pointInformationMatrixCalculator,
  					  &normalInformationMatrixCalculator);
  
  cerr << "loading" << cloudFilename.c_str() << endl;
  DepthImage inputImage;
  inputImage.load(cloudFilename.c_str(),true);
  Frame frame;

  cerr << "computing stats... ";
  depthImageConverter.compute(frame, inputImage, Eigen::Isometry3f::Identity());
  cerr << " done" << endl;
  
  // Frame frame;
  // Isometry3f tmp;
  // frame.load(tmp, cloudFilename.c_str());

  CylindricalPointProjector cylindricalProjector;
  float angularFov = M_PI;
  float angularResolution = 2 * 360 / M_PI;
  cylindricalProjector.setAngularFov(angularFov);
  cylindricalProjector.setAngularResolution(angularResolution);
  DepthImage outputDepthImage;
  Eigen::MatrixXi outputIndexImage;
  outputIndexImage.resize(angularFov * 2.0f * angularResolution, 480);
  
  
  cylindricalProjector.project(outputIndexImage,outputDepthImage,frame.points());
  outputDepthImage.save("cyl.pgm", true);
  
  Frame reconstructedFrame;
  DepthImageConverter cylindricalImageConverter(&cylindricalProjector, &statsCalculator,
						&pointInformationMatrixCalculator,
						&normalInformationMatrixCalculator);

  cylindricalImageConverter.compute(reconstructedFrame, outputDepthImage, Eigen::Isometry3f::Identity());
  
  reconstructedFrame.save("test.pwn");
  
  return 0;
}
