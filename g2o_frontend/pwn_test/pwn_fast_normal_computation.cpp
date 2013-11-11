#include <iostream>

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/pointprojector.h"
#include "g2o_frontend/pwn2/statscalculator.h"
#include "g2o_frontend/pwn2/informationmatrixcalculator.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"

using namespace std;
using namespace Eigen;
using namespace pwn;

int main(int argc, char **argv) {
  int ng_imageRadius, vz_step, numComputations, ng_minimumPoints;
  float di_scaleFactor, di_scale, ng_maxDist;
  string inputFilename, outputFilename;
  
  // Input parameters handling.
  g2o::CommandArgs arg;

  arg.param("ng_imageRadius", ng_imageRadius, 1, "Image radius of pixels where to take points for the computation of a normal");
  arg.param("ng_minimumPoints", ng_minimumPoints, 6, "Minimum number of points to use to compute a normal");
  arg.param("di_scale", di_scale, 1.0f, "Scaling factor to apply on the size of the depth image");
  arg.param("ng_maxDist", ng_maxDist, 0.000001f, "Max distance to consider when detect the invariant component");
  arg.param("di_scaleFactor", di_scaleFactor, 0.001f, "Depth image values scaling factor");
  arg.param("vz_step", vz_step, 1, "Save in the output file one point each vz_step points");
  arg.param("numComputations", numComputations, 1, "Numbers of computations to do");

  // Last parameter has to be the working directory.
  arg.paramLeftOver("inputFilename", inputFilename, "test.pgm", "Input .pgm depth image filename", true);
  arg.paramLeftOver("outputFilename", outputFilename, "test.pwn", "output .pwn filename", true);

  // Set parser input.
  arg.parseArgs(argc, argv);

  // Load depth image
  DepthImage depth, scaledDepth;
  depth.load(inputFilename.c_str(), true, di_scaleFactor);
  DepthImage::scale(scaledDepth, depth, di_scale);

  // Initialize pwn objects
  Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f,   0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f,   0.0f,   1.0f;
  Isometry3f sensorOffset = Isometry3f::Identity();
  float minDistance = 0.5f;
  float maxDistance = 5.0f;
  PinholePointProjector projector;
  projector.setCameraMatrix(cameraMatrix);
  projector.setTransform(Isometry3f::Identity());
  projector.setMaxDistance(maxDistance);
  projector.setMinDistance(minDistance);
  projector.scale(1.0f/(float)di_scale);
  
  StatsCalculator statsCalculator;
  int minImageRadius = 10;
  int maxImageRadius = 30;
  int minPoints = 50;
  float worldRadius = 0.1f;
  float curvatureThreshold = 1.0f;
  statsCalculator.setWorldRadius(worldRadius);
  statsCalculator.setMinImageRadius(minImageRadius);
  statsCalculator.setMaxImageRadius(maxImageRadius);
  statsCalculator.setMinPoints(minPoints);
  statsCalculator.setCurvatureThreshold(curvatureThreshold);
  
  PointInformationMatrixCalculator pointInformationMatrixCalculator;
  pointInformationMatrixCalculator.setCurvatureThreshold(curvatureThreshold);
  
  NormalInformationMatrixCalculator normalInformationMatrixCalculator;
  normalInformationMatrixCalculator.setCurvatureThreshold(curvatureThreshold);
  
  IntImage index;
  if (scaledDepth.rows() != index.rows() || scaledDepth.cols() != index.cols()) {
    index.resize(scaledDepth.rows(), scaledDepth.cols());
  }

  double ts, tf;
  double totalTime = 0.0;  
  Frame frame;
  DepthImageConverter *converter = new DepthImageConverter(&projector, &statsCalculator,
							   &pointInformationMatrixCalculator, 
							   &normalInformationMatrixCalculator);
  for(int i = 0; i < numComputations; i++) {
    ts = g2o::get_time();
    converter->compute(frame, scaledDepth);
    tf = g2o::get_time();
    totalTime += tf - ts;
  }
  cout << "Mean time needed for " << numComputations << " normal computation with integral images: " << totalTime / (float)numComputations << " seconds" << endl;

  frame.clear();
  projector.unProject(frame.points(), frame.gaussians(), index, scaledDepth);
  frame.normals().resize(frame.points().size());
  std::fill(frame.normals().begin(), frame.normals().end(), Normal());
  frame.pointInformationMatrix().resize(frame.points().size());
  std::fill(frame.pointInformationMatrix().begin(), frame.pointInformationMatrix().end(), Matrix4f::Identity());
  frame.normalInformationMatrix().resize(frame.points().size());
  std::fill(frame.normalInformationMatrix().begin(), frame.normalInformationMatrix().end(), Matrix4f::Identity());
  frame.stats().resize(frame.points().size());
  std::fill(frame.stats().begin(), frame.stats().end(), Stats());

  totalTime = 0.0;
  for(int i = 0; i < numComputations; i++) {
    ts = g2o::get_time();
    //statsCalculator.fastCompute(frame.normals(), frame.points(), index, ng_imageRadius, ng_minimumPoints, ng_maxDist);
    statsCalculator.fastCompute(frame.normals(), frame.points(), index, ng_imageRadius);
    tf = g2o::get_time();
    totalTime += tf - ts;
  }
  cout << "Mean time needed for " << numComputations << " normal computation with cross product: " << totalTime / (float)numComputations << " seconds" << endl;

  // Apply sensorOffset
  frame.transformInPlace(sensorOffset);

  // Save frame
  frame.save(outputFilename.c_str(), vz_step, true);

  return 0;
}
