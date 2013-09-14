#include <iostream>
#include <fstream>
#include <sstream>

#include <pcl/registration/icp.h>

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"

#include "g2o_frontend/pwn_utils/pwn_file_format_converter.h"

using namespace std;
using namespace Eigen;
using namespace g2o;
using namespace pwn;

#undef _PWN_USE_CUDA_

int main(int argc, char **argv) {
  string inputFilename, outputFilename;
  int ng_minImageRadius;
  int ng_maxImageRadius;
  int ng_minPoints;
  float ng_worldRadius;
  float ng_scale;
  float ng_curvatureThreshold;

  float cf_inlierDistanceThreshold;
  float al_inlierMaxChi2;
  float pj_maxDistance;
  string sensorType;

  // Input parameters handling
  CommandArgs arg;
  
  // Optional input parameters
  arg.param("ng_minImageRadius", ng_minImageRadius, 10, "Specify the minimum number of pixels composing the square where to take points for a normal computation");
  arg.param("pj_maxDistance", pj_maxDistance, 6, "maximum distance of the points");
  arg.param("ng_maxImageRadius", ng_maxImageRadius, 30, "Specify the maximum number of pixels composing the square where to take points for a normal computation");
  arg.param("ng_minPoints", ng_minPoints, 50, "Specify the minimum number of points to be used to compute a normal");
  arg.param("ng_worldRadius", ng_worldRadius, 0.05f, "Specify the max distance for a point to be used to compute a normal");
  arg.param("ng_scale", ng_scale, 1.0f, "Specify the scaling factor to apply on the depth image");
  arg.param("ng_curvatureThreshold", ng_curvatureThreshold, 1.0f, "Specify the max surface curvature threshold for which normals are discarded");

  arg.param("cf_inlierDistanceThreshold", cf_inlierDistanceThreshold, 0.5f, "Maximum metric distance between two points to regard them as iniliers");

  arg.param("al_inlierMaxChi2", al_inlierMaxChi2, 1e3, "Max chi2 error value for the alignment step");
  arg.param("sensorType", sensorType, "kinect", "Sensor type: xtion640/xtion480/kinect");

  // Last parameter has to be the filename containing the name of pairs of images to match
  // The input file has to contain four columns of images paths saparated by tabs, each column represent the path for 
  // respectively reference depth image, reference color image, current depth image and current color image. If you don't want to
  // use color images you can just put a "-" as name for color images
  arg.paramLeftOver("inputFilename", inputFilename, "", "File containing pairs of depth images paths to register", true);
  arg.paramLeftOver("outputFilename", outputFilename, "", "File containing the values of the benchmark", true);

  // Set parser input
  arg.parseArgs(argc, argv);

  // Open file conatining the pairs of images to match
  ifstream is(inputFilename.c_str());
  if (!is) {
    cerr << "Impossible to open the file containing the names of the images to register... quitting." << endl;
    return 0;
  }
  
  // Create output file
  ofstream os(outputFilename.c_str());
  if (!os) {
    cerr << "Impossible to create the file containing the values of the benchmark... quitting." << endl;
    return 0;
  }

  os << "inliers\tchi2\ttime" << endl;
 
  // Objects Initialization
  // Projector
  PinholePointProjector projector;
  int imageRows, imageCols, scaledImageRows = 0, scaledImageCols = 0;
  Matrix3f cameraMatrix, scaledCameraMatrix;
  Eigen::Isometry3f sensorOffset = Isometry3f::Identity();

  // Set sensor offset
  sensorOffset.translation() = Vector3f(0.0f, 0.0f, 0.0f);
  Quaternionf quaternion = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
  sensorOffset.linear() = quaternion.toRotationMatrix();
  sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  
  cameraMatrix.setIdentity();
  if (sensorType == "xtion640") {
    cameraMatrix << 
      570.342f,   0.0f,   320.0f,
        0,      570.342f, 240.0f,
        0.0f,     0.0f,     1.0f;  
  }  
  else if (sensorType == "xtion320") {
    cameraMatrix << 
      570.342f,  0.0f,   320.0f,
        0.0f,  570.342f, 240.0f,
        0.0f,    0.0f,     1.0f;  
    cameraMatrix.block<2,3>(0,0) *= 0.5;
  } 
  else if (sensorType == "kinect") {
    cameraMatrix << 
      525.0f,   0.0f, 319.5f,
        0.0f, 525.0f, 239.5f,
        0.0f,   0.0f,   1.0f;  
  } 
  else {
    cerr << "Unknown sensor type: [" << sensorType << "]... aborting (you need to specify either xtion or kinect)" << endl;
    return 0;
  }

  // cameraMatrix << 
  float scale = 1.0f / ng_scale;
  scaledCameraMatrix = cameraMatrix * scale;
  scaledCameraMatrix(2, 2) = 1.0f;
  
  // Set the camera matrix to the pinhole point projector
  projector.setCameraMatrix(scaledCameraMatrix);
  projector.setMaxDistance(pj_maxDistance);

  // Stats calculator
  StatsCalculator statsCalculator;
  statsCalculator.setWorldRadius(ng_minImageRadius);
  statsCalculator.setMinImageRadius(ng_maxImageRadius);
  statsCalculator.setMinImageRadius(ng_minPoints);
  statsCalculator.setWorldRadius(ng_worldRadius);
  
  // Information matrix calculators
  PointInformationMatrixCalculator pointInformationMatrixCalculator;
  NormalInformationMatrixCalculator normalInformationMatrixCalculator;
  pointInformationMatrixCalculator.setCurvatureThreshold(ng_curvatureThreshold);
  normalInformationMatrixCalculator.setCurvatureThreshold(ng_curvatureThreshold);

  // Depth image converter
  DepthImageConverter converter(&projector, &statsCalculator, 
				&pointInformationMatrixCalculator, &normalInformationMatrixCalculator);
  statsCalculator.setCurvatureThreshold(ng_curvatureThreshold);
  DepthImage refDepthImage, currDepthImage, refScaledDepthImage, currScaledDepthImage;
  Eigen::MatrixXi indexImage, scaledIndexImage;

// Correspondece finder and linearizer
  CorrespondenceFinder dummyCorrespondenceFinder;
  dummyCorrespondenceFinder.setInlierDistanceThreshold(cf_inlierDistanceThreshold);
  dummyCorrespondenceFinder.setFlatCurvatureThreshold(1.0f);  
  dummyCorrespondenceFinder.setInlierCurvatureRatioThreshold(std::numeric_limits<float>::max());
  dummyCorrespondenceFinder.setInlierNormalAngularThreshold(cosf(M_PI));
  Linearizer dummyLinearizer;
  dummyLinearizer.setInlierMaxChi2(al_inlierMaxChi2);

#ifdef _PWN_USE_CUDA_
  CuAligner dummyAligner;
#else
  Aligner dummyAligner;
#endif

  dummyAligner.setProjector(&projector);
  dummyAligner.setLinearizer(&dummyLinearizer);
  dummyLinearizer.setAligner(&dummyAligner);
  dummyAligner.setCorrespondenceFinder(&dummyCorrespondenceFinder);
  dummyAligner.setInnerIterations(0);
  dummyAligner.setOuterIterations(1);


  // Read one pair of images each time and do the registration
  char line[4096];
  string refDepthImageFilename, refColorImageFilename, currDepthImageFilename, currColorImageFilename;
  while (is.getline(line, 4096)) {
    // Get current pair
    istringstream iss(line);
    iss >> refDepthImageFilename >> refColorImageFilename >> currDepthImageFilename >> currColorImageFilename;

    cout << "-------------------------------------------" << endl;
    cout << "Aligning: " <<  currDepthImageFilename << " ---> " << refDepthImageFilename << endl;

    // Load depth and color images   
    refDepthImage.load(refDepthImageFilename.c_str());
    currDepthImage.load(currDepthImageFilename.c_str());
    imageRows = refDepthImage.rows();
    imageCols = refDepthImage.cols();
    scaledImageRows = imageRows / ng_scale;
    scaledImageCols = imageCols / ng_scale;
    dummyCorrespondenceFinder.setSize(scaledImageRows, scaledImageCols);
    
    // Compute stats
    Frame refFrame, currFrame, refFrameNoNormals, currFrameNoNormals;
    DepthImage::scale(refScaledDepthImage, refDepthImage, ng_scale);
    DepthImage::scale(currScaledDepthImage, currDepthImage, ng_scale);
    converter.compute(refFrame, refScaledDepthImage, sensorOffset, false);
    converter.compute(currFrame, currScaledDepthImage, sensorOffset, false);

    // Convert pwn clouds to pcl type
    pcl::PointCloud<pcl::PointXYZ>::Ptr refPclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr currPclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    pwnToPclPointCloud(refPclPointCloud, &refFrame);
    pwnToPclPointCloud(currPclPointCloud, &currFrame);   
    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // Set the maximum correspondence distance
    icp.setMaxCorrespondenceDistance(0.5);
    // Set the maximum number of iterations
    //icp.setMaximumIterations(50);
    // Set the transformation epsilon
    //icp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon
    //icp.setEuclideanFitnessEpsilon(1);
    icp.setInputSource(currPclPointCloud);
    icp.setInputTarget(refPclPointCloud);
    double ostart = get_time();
    icp.align(aligned);
    double oend = get_time();
    cout << "ICP time: " << oend - ostart << " seconds" << endl;
    Matrix4f finalTransformation = icp.getFinalTransformation();
    Isometry3f T = Isometry3f::Identity();
    T.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    T.translation() = finalTransformation.block<3, 1>(0, 3);
    T.linear() = finalTransformation.block<3, 3>(0, 0);
    
    dummyAligner.setReferenceFrame(&refFrame);
    dummyAligner.setCurrentFrame(&currFrame);
    dummyAligner.setInitialGuess(T);
    dummyAligner.setSensorOffset(sensorOffset);
    dummyAligner.align();

    dummyLinearizer.setT(T.inverse());
    dummyLinearizer.computeChi2WithoutNormalsInfo();
    float chi2 = std::numeric_limits<float>::max();
    if (dummyLinearizer.inliers() != 0) {
      chi2 = dummyLinearizer.computeChi2WithoutNormalsInfo() / dummyLinearizer.inliers();
    }
    cout << "Chi2: " << chi2 << endl;
    cout << "Inliers: " << dummyLinearizer.inliers() << endl;
    os << dummyLinearizer.inliers() << "\t";
    os << chi2 << "\t";
    os << oend - ostart << endl;
  
    // Save clouds
    // refFrame.save("reference.pwn", 1, true, Eigen::Isometry3f::Identity());
    // currFrame.save("current.pwn", 1, true, Eigen::Isometry3f::Identity());   
    // currFrame.save("alignedICP.pwn", 1, true, T);
  }

  return 0;
}
