#include <iostream>

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
  // Load depth image
  string depthFilename = "test.pgm";
  float scaleFactor = 0.001f;
  
  DepthImage depth;
  depth.load(depthFilename.c_str(), true, scaleFactor);
  
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
  if (depth.rows() != index.rows() || depth.cols() != index.cols()) {
    index.resize(depth.rows(), depth.cols());
  }

  Frame frame;
  DepthImageConverter *converter = new DepthImageConverter(&projector, &statsCalculator,
							   &pointInformationMatrixCalculator, &normalInformationMatrixCalculator);
  double ts, tf;
  double totalTime = 0.0;
  // for(int i = 0; i < 100; i++) {
  //   ts = g2o::get_time();
  //   converter->compute(frame, depth);
  //   tf = g2o::get_time();
  //   totalTime += tf - ts;
  // }
  // cout << "Mean time needed for 100 normal computation with integral images: " << totalTime/100.0f << " seconds" << endl;

  frame.clear();
  projector.unProject(frame.points(), frame.gaussians(), index, depth);
  frame.normals().resize(frame.points().size());
  std::fill(frame.normals().begin(), frame.normals().end(), Normal());
  frame.pointInformationMatrix().resize(frame.points().size());
  std::fill(frame.pointInformationMatrix().begin(), frame.pointInformationMatrix().end(), Matrix4f::Identity());
  frame.normalInformationMatrix().resize(frame.points().size());
  std::fill(frame.normalInformationMatrix().begin(), frame.normalInformationMatrix().end(), Matrix4f::Identity());
  frame.stats().resize(frame.points().size());
  std::fill(frame.stats().begin(), frame.stats().end(), Stats());

  int imageRadius = 2;
  totalTime = 0.0;
  for(int i = 0; i < 100; i++) {
    std::fill(frame.normals().begin(), frame.normals().end(), Normal());
    ts = g2o::get_time();
    statsCalculator.fastCompute(frame.normals(), frame.points(), index, imageRadius);
    tf = g2o::get_time();
    totalTime += tf - ts;
  }
  cout << "Mean time needed for 100 normal computation with cross product: " << totalTime/100.0f << " seconds" << endl;

  // Apply sensorOffset
  frame.transformInPlace(sensorOffset);

  // Save frame
  frame.save("test.pwn", 1, true);

  return 0;
}
