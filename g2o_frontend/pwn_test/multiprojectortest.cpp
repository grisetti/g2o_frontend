#include <iostream>
#include <fstream>

#include "g2o/stuff/command_args.h"

#include "g2o_frontend/pwn2/depthimage.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/multipointprojector.h"

using namespace std;
using namespace Eigen;
using namespace g2o;
using namespace pwn;

int main(int argc, char **argv) {
  int numImages = 0;
  string cloudFilename[4];
  string outputFilename = "totalFrame.pwn";

  // Input parameters handling.
  g2o::CommandArgs arg;
  
  arg.param("numImages", numImages, 4, "The number of images to read, can be a value between 0 and 4");
 
  // Last parameter has to be the working directory.
  arg.paramLeftOver("cloud1", cloudFilename[0], "", "first input cloud", true);
  arg.paramLeftOver("cloud2", cloudFilename[1], "", "second input cloud", true);
  arg.paramLeftOver("cloud3", cloudFilename[2], "", "third input cloud", true);
  arg.paramLeftOver("cloud4", cloudFilename[3], "", "fourth input cloud", true);

  // Set parser input.
  arg.parseArgs(argc, argv);

  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;

  
  Isometry3f sensorOffset[4];
  Frame frame[4];
  Frame totalFrame;
  pwn::DepthImage depthImage[4];
  PinholePointProjector projector;
  projector.setCameraMatrix(cameraMatrix);
  MultiPointProjector multiProjector;
  StatsCalculator statsCalculator;
  PointInformationMatrixCalculator pointInformationMatrixCalculator;
  NormalInformationMatrixCalculator normalInformationMatrixCalculator;
  DepthImageConverter depthImageConverter(&projector, &statsCalculator,
					  &pointInformationMatrixCalculator,
					  &normalInformationMatrixCalculator);

  // Testing project
  for(int i = 0; i < numImages; i++) {
    depthImage[i].load(cloudFilename[i].c_str(), true);

    sensorOffset[i] = Isometry3f::Identity();
    sensorOffset[i].translation() = Vector3f(0.15f, 0.0f, 0.05f);
    Quaternionf quat = Quaternionf(0.5, -0.5, 0.5, -0.5);
    if(i > 0)
      sensorOffset[i].linear() =  AngleAxisf(i * M_PI / 2.0f, Vector3f::UnitZ()) * quat.toRotationMatrix();
    else
      sensorOffset[i].linear() =  quat.toRotationMatrix();
    sensorOffset[i].matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
    multiProjector.addPointProjector(&projector, sensorOffset[i], depthImage[i].rows(), depthImage[i].cols());
    
    depthImageConverter.compute(frame[i], depthImage[i], sensorOffset[i]);

    totalFrame.add(frame[i]);
  }

  MatrixXi indexImageRullino;
  DepthImage rullino;

  multiProjector.project(indexImageRullino, rullino, totalFrame.points());

  // Testing unproject
  totalFrame.clear();
  multiProjector.unProject(totalFrame.points(), totalFrame.gaussians(), indexImageRullino, rullino);
  
  // Testing projectIntervals
  totalFrame.clear();

  depthImageConverter.setProjector(&multiProjector);
  
  Isometry3f tmp = Isometry3f::Identity();
  tmp.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  depthImageConverter.compute(totalFrame, rullino, tmp);

  totalFrame.save(outputFilename.c_str(), 1, true);

  return 0;
}
