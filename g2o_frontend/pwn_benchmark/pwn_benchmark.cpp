#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/registration/icp.h>

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"

#include "g2o_frontend/pwn_utils/pwn_file_format_converter.h"

#undef _PWN_USE_CUDA_

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace g2o;
using namespace pwn;

Eigen::Isometry3f fromMat(const cv::Mat m);

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

int main(int argc, char **argv) {
  int usePwn, usePwnNoNormals, useIcp, useDvo;
  string inputFilename, outputChi2Filename, outputTimeFilename;
  int ng_minImageRadius;
  int ng_maxImageRadius;
  int ng_minPoints;
  float ng_worldRadius;
  float ng_scale;
  float ng_curvatureThreshold;

  float cf_inlierNormalAngularThreshold;
  float cf_flatCurvatureThreshold;
  float cf_inlierCurvatureRatioThreshold;
  float cf_inlierDistanceThreshold;

  int al_innerIterations;
  int al_outerIterations;
  int al_minNumInliers; 
  float al_minError;
  float al_inlierMaxChi2;
  float pj_maxDistance;
  string sensorType;

  // Input parameters handling
  CommandArgs arg;
  
  // Optional input parameters
  arg.param("usePwn", usePwn, 1, "Say if you want to use PWN alignment for the bechmark");
  arg.param("usePwnNoNormals", usePwnNoNormals, 1, "Say if you want to use PWN alignment without normals information for the bechmark");
  arg.param("useIcp", useIcp, 0, "Say if you want to use ICP alignment for the bechmark");
  arg.param("useDvo", useDvo, 0, "Say if you want to use DVO alignment for the bechmark");
  arg.param("ng_minImageRadius", ng_minImageRadius, 10, "Specify the minimum number of pixels composing the square where to take points for a normal computation");
  arg.param("pj_maxDistance", pj_maxDistance, 6, "maximum distance of the points");
  arg.param("ng_maxImageRadius", ng_maxImageRadius, 30, "Specify the maximum number of pixels composing the square where to take points for a normal computation");
  arg.param("ng_minPoints", ng_minPoints, 50, "Specify the minimum number of points to be used to compute a normal");
  arg.param("ng_worldRadius", ng_worldRadius, 0.05f, "Specify the max distance for a point to be used to compute a normal");
  arg.param("ng_scale", ng_scale, 1.0f, "Specify the scaling factor to apply on the depth image");
  arg.param("ng_curvatureThreshold", ng_curvatureThreshold, 1.0f, "Specify the max surface curvature threshold for which normals are discarded");
  
  arg.param("cf_inlierNormalAngularThreshold", cf_inlierNormalAngularThreshold, M_PI / 6.0f, "Maximum angle between the normals of two points to regard them as iniliers");
  arg.param("cf_flatCurvatureThreshold", cf_flatCurvatureThreshold, 0.02f, "Maximum curvature value for a point to be used for data association");
  arg.param("cf_inlierCurvatureRatioThreshold", cf_inlierCurvatureRatioThreshold, 1.3f, "Maximum curvature ratio value between two points to regard them as iniliers");
  arg.param("cf_inlierDistanceThreshold", cf_inlierDistanceThreshold, 0.5f, "Maximum metric distance between two points to regard them as iniliers");

  arg.param("al_innerIterations", al_innerIterations, 1, "Specify the inner iterations");
  arg.param("al_outerIterations", al_outerIterations, 10, "Specify the outer iterations");
  arg.param("al_minNumInliers", al_minNumInliers, 10000, "Specify the minimum number of inliers to consider an alignment good");
  arg.param("al_minError", al_minError, 10.0f, "Specify the minimum error to consider an alignment good");
  arg.param("al_inlierMaxChi2", al_inlierMaxChi2, 1e3, "Max chi2 error value for the alignment step");
  arg.param("sensorType", sensorType, "kinect", "Sensor type: xtion640/xtion480/kinect");

  // Last parameter has to be the filename containing the name of pairs of images to match
  // The input file has to contain four columns of images paths saparated by tabs, each column represent the path for 
  // respectively reference depth image, reference color image, current depth image and current color image. If you don't want to
  // use color images you can just put a "-" as name for color images
  arg.paramLeftOver("inputFilename", inputFilename, "", "File containing pairs of depth images paths to register", true);
  arg.paramLeftOver("outputChi2Filename", outputChi2Filename, "", "File containing the chi2 values of the benchmark", true);
  arg.paramLeftOver("outputTimeFilename", outputTimeFilename, "", "File containing the time values of the benchmark", true);

  // Set parser input
  arg.parseArgs(argc, argv);

  // Open file conatining the pairs of images to match
  ifstream is(inputFilename.c_str());
  if (!is) {
    cerr << "Impossible to open the file containing the names of the images to register... quitting." << endl;
    return 0;
  }
  
  // Create output file
  ofstream osChi2(outputChi2Filename.c_str());
  if (!osChi2) {
    cerr << "Impossible to create the file containing the chi2 values of the benchmark... quitting." << endl;
    return 0;
  }
  ofstream osTime(outputTimeFilename.c_str());
  if (!osTime) {
    cerr << "Impossible to create the file containing the time values of the benchmark... quitting." << endl;
    return 0;
  }
  if (usePwn) {
    osChi2 << "pwn\t";
    osTime << "pwn\t";
  }
  if (usePwnNoNormals) {
    osChi2 << "pwnnn\t";
    osTime << "pwnnn\t";
  }
  if (useIcp) {
    osChi2 << "icp\t";
    osTime << "icp\t";
  }
  if (useDvo) {
    osChi2 << "dvo\t";
    osTime << "dvo\t";
  }
  osChi2 << endl;
  osTime << endl;

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
  Mat cvCameraMatrix = Mat::eye(3, 3, CV_32FC1);
  if (sensorType == "xtion640") {
    cameraMatrix << 
      570.342f,   0.0f,   320.0f,
        0,      570.342f, 240.0f,
        0.0f,     0.0f,     1.0f;  
    float vals[] = {570.342f, 0.0f, 320.0f,
		    0.0f, 570.342f, 240.0f,
		    0.0f, 0.0f, 1.0f};
    cvCameraMatrix = Mat(3, 3, CV_32FC1, vals);
  }  
  else if (sensorType == "xtion320") {
    cameraMatrix << 
      570.342f,  0.0f,   320.0f,
        0.0f,  570.342f, 240.0f,
        0.0f,    0.0f,     1.0f;  
    cameraMatrix.block<2,3>(0,0) *= 0.5;
    float vals[] = {570.342f * 0.5f, 0.0f, 320.0f * 0.5f,
		    0.0f, 570.342f * 0.5f, 240.0f * 0.5f,
		    0.0f, 0.0f, 1.0f};
    cvCameraMatrix = Mat(3, 3, CV_32FC1, vals);
  } 
  else if (sensorType == "kinect") {
    cameraMatrix << 
      525.0f,   0.0f, 319.5f,
        0.0f, 525.0f, 239.5f,
        0.0f,   0.0f,   1.0f;  
    float vals[] = {525.0f, 0.0f, 319.5f,
		    0.0f, 525.0f, 239.5f,
		    0.0f, 0.0f, 1.0f};
    cvCameraMatrix = Mat(3, 3, CV_32FC1, vals);
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
  CorrespondenceFinder correspondenceFinder;
  correspondenceFinder.setInlierDistanceThreshold(cf_inlierDistanceThreshold);
  correspondenceFinder.setFlatCurvatureThreshold(cf_flatCurvatureThreshold);  
  correspondenceFinder.setInlierCurvatureRatioThreshold(cf_inlierCurvatureRatioThreshold);
  correspondenceFinder.setInlierNormalAngularThreshold(cosf(cf_inlierNormalAngularThreshold));
  Linearizer linearizer;
  linearizer.setInlierMaxChi2(al_inlierMaxChi2);

#ifdef _PWN_USE_CUDA_
  CuAligner aligner;
#else
  Aligner aligner;
#endif

  aligner.setProjector(&projector);
  aligner.setLinearizer(&linearizer);
  linearizer.setAligner(&aligner);
  aligner.setCorrespondenceFinder(&correspondenceFinder);
  aligner.setInnerIterations(al_innerIterations);
  aligner.setOuterIterations(al_outerIterations);

  // Read one pair of images each time and do the registration
  char line[4096];
  string refDepthImageFilename, refColorImageFilename, currDepthImageFilename, currColorImageFilename;
  Mat referenceImage, currentImage, currentDepth, referenceDepth, referenceDepthFlt, currentDepthFlt, Rt;
  while (is.getline(line, 4096)) {
    // Get current pair
    istringstream iss(line);
    iss >> refDepthImageFilename >> refColorImageFilename >> currDepthImageFilename >> currColorImageFilename;

    if (useDvo && (refColorImageFilename == "-" || currColorImageFilename == "-")) {
      cerr << "Impossible to use DVO alignment on this pair beacuse one or both color images are not defined... skipping." << endl;
      continue;
    }

    cout << "######################################################################################" << endl;
    cout << "Aligning: " << line << endl;
    cout << "######################################################################################" << endl;

    // Load depth and color images   
    refDepthImage.load(refDepthImageFilename.c_str());
    currDepthImage.load(currDepthImageFilename.c_str());
    imageRows = refDepthImage.rows();
    imageCols = refDepthImage.cols();
    scaledImageRows = imageRows / ng_scale;
    scaledImageCols = imageCols / ng_scale;
    correspondenceFinder.setSize(scaledImageRows, scaledImageCols);
    referenceDepth = imread(refDepthImageFilename.c_str(), -1);
    referenceDepth.convertTo(referenceDepthFlt, CV_32FC1, 1.0f/1000.0f);
    referenceImage = imread(refColorImageFilename, -1);
    currentDepth = imread(currDepthImageFilename.c_str(), -1);
    currentDepth.convertTo(currentDepthFlt, CV_32FC1, 1.0f/1000.0f);
    currentImage = imread(currColorImageFilename.c_str(), -1);
    
    // Compute stats
    Frame refFrame, currFrame, refFrameNoNormals, currFrameNoNormals;
    DepthImage::scale(refScaledDepthImage, refDepthImage, ng_scale);
    DepthImage::scale(currScaledDepthImage, currDepthImage, ng_scale);
    converter.compute(refFrame, refScaledDepthImage, sensorOffset, false);
    converter.compute(currFrame, currScaledDepthImage, sensorOffset, false);
    refFrameNoNormals = refFrame;
    // InformationMatrixVector &refNormalOmegas = refFrameNoNormals.normalInformationMatrix();
    // StatsVector &refStats = refFrameNoNormals.stats();
    // NormalVector &refNormals = refFrameNoNormals.normals();
    // for (size_t i = 0; i < refNormals.size(); i++) {
    //   //refNormals[i] = Vector4f(1.0f, 1.0f, 1.0f, 0.0f);
    //   refStats[i](4, 4) = 0.0f;
    //   //refNormalOmegas[i].setZero();
    // }
    currFrameNoNormals = currFrame;
    // InformationMatrixVector &currNormalOmegas = currFrameNoNormals.normalInformationMatrix();
    // StatsVector &currStats = currFrameNoNormals.stats();
    // NormalVector &currNormals = currFrameNoNormals.normals();
    // for (size_t i = 0; i < currNormals.size(); i++) {
    //   //currNormals[i] = Vector4f(1.0f, 1.0f, 1.0f, 0.0f);
    //   currStats[i](4, 4) = 0.0f;
    //   //currNormalOmegas[i].setZero();
    // }

    // Convert pwn clouds to pcl type
    pcl::PointCloud<pcl::PointXYZ>::Ptr refPclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr currPclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    pwnToPclPointCloud(refPclPointCloud, &refFrame);
    pwnToPclPointCloud(currPclPointCloud, &currFrame);

    // Save clouds
    refFrame.save("reference.pwn", 1, true, Eigen::Isometry3f::Identity());
    currFrame.save("current.pwn", 1, true, Eigen::Isometry3f::Identity());      

    // PWN
    if (usePwn) {
      cout << "PWN...";
      Isometry3f initialGuess = Isometry3f::Identity();
      initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      aligner.setReferenceFrame(&refFrame);
      aligner.setCurrentFrame(&currFrame);
      aligner.setInitialGuess(initialGuess);
      aligner.setSensorOffset(sensorOffset);
      aligner.setInnerIterations(al_innerIterations);
      aligner.setOuterIterations(al_outerIterations);
      double ostart = get_time();
      aligner.align();
      double oend = get_time();
      cout << " done." << endl;
      cout << "PWN time: " << oend - ostart << " seconds" << endl;
      cout << "PWN Chi2: " << aligner.error() << endl;
      cout << "---------------------------------------------------" << endl;
      osChi2 << aligner.error() << "\t";
      osTime << oend - ostart << "\t";
      currFrame.save("alignedPWN.pwn", 1, true, aligner.T());
    }

    // PWN without normals
    if (usePwnNoNormals) {
      cout << "PWN without normals...";    
      Isometry3f initialGuess = Isometry3f::Identity();
      initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      aligner.setReferenceFrame(&refFrameNoNormals);
      aligner.setCurrentFrame(&currFrameNoNormals);
      aligner.setInitialGuess(initialGuess);
      aligner.setSensorOffset(sensorOffset);
      aligner.setInnerIterations(al_innerIterations);
      aligner.setOuterIterations(al_outerIterations);
      double ostart = get_time();
      aligner.align();
      double oend = get_time();
      cout << " done." << endl;
      cout << "PWN without normals time: " << oend - ostart << " seconds" << endl;
      cout << "PWN without normals Chi2: " << aligner.error() << endl;
      cout << "---------------------------------------------------" << endl;
      osChi2 << aligner.error() << "\t";
      osTime << oend - ostart << "\t";
      currFrame.save("alignedPWNNN.pwn", 1, true, aligner.T());
    }

    // ICP
    if (useIcp) {
      cout << "ICP...";
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      // Set the maximum correspondence distance
      //icp.setMaxCorrespondenceDistance(0.05);
      // Set the maximum number of iterations
      //icp.setMaximumIterations(50);
      // Set the transformation epsilon
      //icp.setTransformationEpsilon(1e-8);
      // Set the euclidean distance difference epsilon
      //icp.setEuclideanFitnessEpsilon(1);
      icp.setInputSource(refPclPointCloud);
      icp.setInputTarget(currPclPointCloud);
      double ostart = get_time();
      icp.align(aligned);
      double oend = get_time();
      cout << " done." << endl;
      cout << "ICP time: " << oend - ostart << " seconds" << endl;
      Matrix4f finalTransformation = icp.getFinalTransformation();
      Isometry3f initialGuess = Isometry3f::Identity();
      initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      initialGuess.translation() = finalTransformation.block<3, 1>(0, 3);
      initialGuess.linear() = finalTransformation.block<3, 3>(0, 0);
      aligner.setReferenceFrame(&refFrame);
      aligner.setCurrentFrame(&currFrame);
      aligner.setInitialGuess(initialGuess.inverse());
      aligner.setSensorOffset(sensorOffset);
      aligner.setInnerIterations(1);
      aligner.setOuterIterations(1);
      aligner.align();
      cout << "ICP Chi2: " << aligner.error() << endl;
      cout << "---------------------------------------------------" << endl;
      osChi2 << aligner.error() << "\t";
      osTime << oend - ostart << "\t";
      pcl::io::savePCDFileASCII("reference.pcd", *refPclPointCloud);
      pcl::io::savePCDFileASCII("current.pcd", *currPclPointCloud);
      pcl::io::savePCDFileASCII("alignedICP.pcd", aligned);
      currFrame.save("alignedICP.pwn", 1, true, initialGuess.inverse());
    }

    // DVO
    if (useDvo) {
      cout << "DVO...";
      Rt = Mat::eye(4, 4, CV_32FC1);
      double ostart = get_time();
      cv::RGBDOdometry(Rt, Mat(),
		       referenceImage, referenceDepthFlt, Mat(),
		       currentImage, currentDepthFlt, Mat(),
		       cvCameraMatrix);
      double oend = get_time();
      cout << " done." << endl;
      cout << "DVO time: " << oend - ostart << " seconds" << endl;
      Eigen::Isometry3f initialGuess = fromMat(Rt);
      initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      aligner.setReferenceFrame(&refFrame);
      aligner.setCurrentFrame(&currFrame);
      aligner.setInitialGuess(initialGuess);
      aligner.setSensorOffset(sensorOffset);
      aligner.setInnerIterations(1);
      aligner.setOuterIterations(1);
      aligner.align();
      cout << "DVO Chi2: " << aligner.error() << endl;
      cout << "---------------------------------------------------" << endl;
      osChi2 << aligner.error() << "\t";
      osTime << oend - ostart << "\t";
      currFrame.save("alignedDVO.pwn", 1, true, initialGuess);
    }
    
    osChi2 << endl;
    osTime << endl;
  }

  return 0;
}

Eigen::Isometry3f fromMat(const cv::Mat m) {
  Eigen::Matrix3f R;
  Eigen::Vector3f t;
  Eigen::Isometry3f T;
  T.setIdentity();
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      T.matrix()(i, j) = m.at<float>(i, j);
    }
  }
  return T;
}
