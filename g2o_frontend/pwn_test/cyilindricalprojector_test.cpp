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


  CylindricalPointProjector cylindricalProjector;
  cylindricalProjector.setAngularFov(M_PI/4);
  cylindricalProjector.setAngularResolution(2*720/M_PI);
  DepthImage outputDepthImage;
  Eigen::MatrixXi outputIndexImage;
  outputIndexImage.resize(720,480);
  
  
  cylindricalProjector.project(outputIndexImage,outputDepthImage,frame.points());
  outputDepthImage.save("cyl.pgm", true);
  
  Frame reconstructedFrame;
  DepthImageConverter cylindricalImageConverter(&cylindricalProjector, &statsCalculator,
						&pointInformationMatrixCalculator,
						&normalInformationMatrixCalculator);

  cylindricalImageConverter.compute(reconstructedFrame, outputDepthImage, Eigen::Isometry3f::Identity());
  
  reconstructedFrame.save("test.pwn");
  
  // // Testing project
  // for(int i = 0; i < numImages; i++) {
  //   depthImage[i].load(cloudFilename[i].c_str(), true);

  //   sensorOffset[i] = Isometry3f::Identity();
  //   sensorOffset[i].translation() = Vector3f(0.15f, 0.0f, 0.05f);
  //   Quaternionf quat = Quaternionf(0.5, -0.5, 0.5, -0.5);
  //   if(i > 0)
  //     sensorOffset[i].linear() =  AngleAxisf(i * M_PI / 2.0f, Vector3f::UnitZ()) * quat.toRotationMatrix();
  //   else
  //     sensorOffset[i].linear() =  quat.toRotationMatrix();
  //   sensorOffset[i].matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
  //   multiProjector.addPointProjector(&projector, sensorOffset[i], depthImage[i].rows(), depthImage[i].cols());
    
  //   depthImageConverter.compute(frame[i], depthImage[i], sensorOffset[i]);

  //   totalFrame.add(frame[i]);
  // }

  // MatrixXi indexImageRullino;
  // DepthImage rullino;

  // multiProjector.project(indexImageRullino, rullino, totalFrame.points());

  // // Testing unproject
  // totalFrame.clear();
  // multiProjector.unProject(totalFrame.points(), totalFrame.gaussians(), indexImageRullino, rullino);
  
  // // Testing projectIntervals
  // totalFrame.clear();

  // depthImageConverter = DepthImageConverter(&multiProjector, &statsCalculator,
  // 					    &pointInformationMatrixCalculator,
  // 					    &normalInformationMatrixCalculator);
  
  // Isometry3f tmp = Isometry3f::Identity();
  // tmp.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  // depthImageConverter.compute(totalFrame, rullino, tmp, true);

  // totalFrame.save(outputFilename.c_str(), 1, true);

  // return 0;
}
