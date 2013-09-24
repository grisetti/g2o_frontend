#include <iostream>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <set>
#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"

#include "g2o_frontend/basemath/bm_se3.h"

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace g2o;
using namespace pwn;

void parseGroundTruth(Isometry3f &initialTransformation, ifstream &isGroundtruth, double targetTimestamp);
Eigen::Isometry3f fromMat(const cv::Mat m);
string type2str(int type);

int main(int argc, char **argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string directory, benchmarkFilename, absoluteTrajectoryFilename, relativeTrajectoryFilename;
  int ng_minImageRadius;
  int ng_maxImageRadius;
  int ng_minPoints;
  float ng_worldRadius;
  float ng_scale;
  float ng_curvatureThreshold;

  float cf_inlierDistanceThreshold;
  float al_inlierMaxChi2;
  float pj_maxDistance;
  float di_scaleFactor;
  string sensorType;

  // Input parameters handling
  CommandArgs arg;
  
  // Optional input parameters
  arg.param("pj_maxDistance", pj_maxDistance, 6, "maximum distance of the points");
  arg.param("di_scaleFactor", di_scaleFactor, 1000.0f, "Scale factor to apply to convert depth images in meters");
  arg.param("ng_minImageRadius", ng_minImageRadius, 10, "Specify the minimum number of pixels composing the square where to take points for a normal computation");
  arg.param("ng_maxImageRadius", ng_maxImageRadius, 30, "Specify the maximum number of pixels composing the square where to take points for a normal computation");
  arg.param("ng_minPoints", ng_minPoints, 50, "Specify the minimum number of points to be used to compute a normal");
  arg.param("ng_worldRadius", ng_worldRadius, 0.05f, "Specify the max distance for a point to be used to compute a normal");
  arg.param("ng_scale", ng_scale, 1.0f, "Specify the scaling factor to apply on the depth image");
  arg.param("ng_curvatureThreshold", ng_curvatureThreshold, 1.0f, "Specify the max surface curvature threshold for which normals are discarded");
  
  arg.param("cf_inlierDistanceThreshold", cf_inlierDistanceThreshold, 0.5f, "Maximum metric distance between two points to regard them as iniliers");
  arg.param("al_inlierMaxChi2", al_inlierMaxChi2, 1e3, "Max chi2 error value for the alignment step");
  arg.param("sensorType", sensorType, "kinect", "Sensor type: xtion640/xtion320/kinect/kinectFreiburg1/kinectFreiburg2/kinectFreiburg3");

  arg.param("benchmarkFilename", benchmarkFilename, "dvo_benchmark.txt", "Specify the name of the file that will contain the results of the benchmark");
  arg.param("absoluteTrajectoryFilename", absoluteTrajectoryFilename, "dvo_absolute_trajectory.txt", "Specify the name of the file that will contain the absolute trajectory computed");
  arg.param("relativeTrajectoryFilename", relativeTrajectoryFilename, "dvo_relative_trajectory.txt", "Specify the name of the file that will contain the relative trajectory computed");

  arg.paramLeftOver("directory", directory, ".", "Directory where the program will find the ground truth file and the subfolders depth and rgb containing the images", true);
  
  // Set parser input
  arg.parseArgs(argc, argv);

  /************************************************************************
   *                             Managing Files                           *
   ************************************************************************/
  // Create benchmark file
  ofstream osBenchmark(benchmarkFilename.c_str());
  if (!osBenchmark) {
    cerr << "Impossible to create the file containing the values of the benchmark... quitting program." << endl;
    return 0;
  }
  
  osBenchmark << "inliers\tchi2\ttime" << endl;
  
  // Create absolute trajectory file
  ofstream osAbsoluteTrajectory(absoluteTrajectoryFilename.c_str());
  if (!osAbsoluteTrajectory) {
    cerr << "Impossible to create the file containing the absolute computed trajectory poses... quitting program." << endl;
    return 0;
  }
  
  // Create relative trajectory file
  ofstream osRelativeTrajectory(relativeTrajectoryFilename.c_str());
  if (!osRelativeTrajectory) {
    cerr << "Impossible to create the file containing the relative computed trajectory poses... quitting program." << endl;
    return 0;
  }

  // Open associations file
  cout << "Opening associations file " << "dvo_associations.txt" << "... ";
  ifstream isAssociations("dvo_associations.txt");
  if (!isAssociations) {
    cerr << "Impossible to open associations file... quitting program." << endl;
    return 0;
  }
  char line[4096];
  if (!isAssociations.getline(line, 4096)) {
    cerr << "Associations file associations.txt is empty... quitting program." << endl;
    return 0;
  }    
  istringstream issAssociations(line);
  string refRgbTimestamp, currRgbTimestamp, refDepthTimestamp, currDepthTimestamp; 
  string refRgbFilename, currRgbFilename, refDepthFilename, currDepthFilename;
  issAssociations >> refRgbTimestamp >> refRgbFilename >> refDepthTimestamp >> refDepthFilename;
  cout << "done." << endl;
  
  // Parsing ground truth file for the initial transformation
  cout << "Parsing ground truth file " << "groundtruth.txt" << "... ";
  ifstream isGroundtruth("groundtruth.txt");
  if (!isGroundtruth) {
    cerr << "Impossible to open ground truth file... quitting program." << endl;
    return 0;
  }
  Isometry3f initialTransformation = Isometry3f::Identity();
  parseGroundTruth(initialTransformation, isGroundtruth, atof(refDepthTimestamp.c_str()));
  initialTransformation.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  cout << "done." << endl;

  /************************************************************************
   *                       Objects Initialization                         *
   ************************************************************************/
  // Projector
  PinholePointProjector projector;
  int imageRows, imageCols, scaledImageRows = 0, scaledImageCols = 0;
  Matrix3f cameraMatrix, scaledCameraMatrix;

  // Set sensor offset 
  Isometry3f sensorOffset = Isometry3f::Identity();
  // sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
  // Quaternionf quaternion = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
  // sensorOffset.linear() = quaternion.toRotationMatrix();
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
  else if (sensorType == "kinectFreiburg1") {
    cameraMatrix << 
      517.3f,   0.0f, 318.6f,
        0.0f, 516.5f, 255.3f,
        0.0f,   0.0f,   1.0f;
    float vals[] = {517.3f, 0.0f, 318.6f,
		    0.0f, 516.5f, 255.3f,
		    0.0f, 0.0f, 1.0f};
    cvCameraMatrix = Mat(3, 3, CV_32FC1, vals);
  }
  else if (sensorType == "kinectFreiburg2") {
    cameraMatrix << 
      520.9f,   0.0f, 325.1f,
      0.0f,   521.0f, 249.7f,
      0.0f,     0.0f,   1.0f;  
    float vals[] = {520.9f, 0.0f, 325.1f,
		    0.0f, 521.0f, 249.7f,
		    0.0f, 0.0f, 1.0f};
    cvCameraMatrix = Mat(3, 3, CV_32FC1, vals);
  }
  else if (sensorType == "kinectFreiburg3") {
    cameraMatrix << 
      535.4f,   0.0f, 320.1f,
      0.0f,   539.2f, 247.6f,
      0.0f,     0.0f,   1.0f;  
    float vals[] = {535.4f, 0.0f, 320.1f,
		    0.0f, 539.2f, 247.6f,
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

  int transformationType = RIGID_BODY_MOTION;

  vector<int> iterCounts(4);
  iterCounts[0] = 7;
  iterCounts[1] = 7;
  iterCounts[2] = 7;
  iterCounts[3] = 10;
  
  vector<float> minGradMagnitudes(4);
  minGradMagnitudes[0] = 12;
  minGradMagnitudes[1] = 5;
  minGradMagnitudes[2] = 3;
  minGradMagnitudes[3] = 1;

  const float minDepth = 0.5f; //in meters
  const float maxDepth = 5.0f; //in meters
  const float maxDepthDiff = 0.07f; //in meters

  /************************************************************************
   *                       Sequential Alignment                          *
   ************************************************************************/
  Mat referenceImage, currentImage, currentDepth, referenceDepth, referenceDepthFlt, currentDepthFlt, Rt;
  Isometry3f currentAbsolutePose = initialTransformation;
  currentAbsolutePose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  Vector6f currentAbsolutePoseVector = t2v(currentAbsolutePose);
  Isometry3f currentRelativePose = Isometry3f::Identity();
  currentRelativePose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  Vector6f currentRelativePoseVector = t2v(currentRelativePose);
  cout << "Initial Absolute Pose: " << currentAbsolutePoseVector.transpose() << endl;
  cout << "Initial Relative Pose: " << currentRelativePoseVector.transpose() << endl;
  float qw = sqrtf(1.0f - (currentAbsolutePoseVector[3]*currentAbsolutePoseVector[3] + 
			   currentAbsolutePoseVector[4]*currentAbsolutePoseVector[4] +
			   currentAbsolutePoseVector[5]*currentAbsolutePoseVector[5]));
  osAbsoluteTrajectory << refDepthTimestamp << " "
		       << currentAbsolutePoseVector[0] << " " << currentAbsolutePoseVector[1] << " " << currentAbsolutePoseVector[2] << " " 
		       << currentAbsolutePoseVector[3] << " " << currentAbsolutePoseVector[4] << " " << currentAbsolutePoseVector[5] << " " << qw
		       << endl;
  while (isAssociations.getline(line, 4096)) {
    istringstream issAssociations(line);
    issAssociations >> currRgbTimestamp >> currRgbFilename >> currDepthTimestamp >> currDepthFilename;
    cout << "-------------------------------------------" << endl;
    cout << "Aligning: " << currDepthFilename << " ---> " << refDepthFilename << endl;

    // Load depth images   
    refDepthImage.load((refDepthFilename.substr(0, refDepthFilename.size() - 3) + "pgm").c_str(), true, di_scaleFactor);
    currDepthImage.load((currDepthFilename.substr(0, currDepthFilename.size() - 3) + "pgm").c_str(), true, di_scaleFactor);
    imageRows = refDepthImage.rows();
    imageCols = refDepthImage.cols();
    scaledImageRows = imageRows / ng_scale;
    scaledImageCols = imageCols / ng_scale;
    dummyCorrespondenceFinder.setImageSize(scaledImageRows, scaledImageCols);
    projector.setImageSize(scaledImageRows, scaledImageCols);
    referenceDepth = imread(refDepthFilename.c_str(), -1);
    referenceDepth.convertTo(referenceDepthFlt, CV_32FC1, di_scaleFactor);
    referenceImage = imread(refRgbFilename, 1);
    currentDepth = imread(currDepthFilename.c_str(), -1);
    currentDepth.convertTo(currentDepthFlt, CV_32FC1, di_scaleFactor);
    currentImage = imread(currRgbFilename.c_str(), 1);
    Mat grayReferenceImage, grayCurrentImage;
    cvtColor(referenceImage, grayReferenceImage, CV_BGR2GRAY);
    cvtColor(currentImage, grayCurrentImage, CV_BGR2GRAY);

    // Compute stats
    Frame refFrame, currFrame, refFrameNoNormals, currFrameNoNormals;
    DepthImage::scale(refScaledDepthImage, refDepthImage, ng_scale);
    DepthImage::scale(currScaledDepthImage, currDepthImage, ng_scale);
    converter.compute(refFrame, refScaledDepthImage, sensorOffset, false);
    converter.compute(currFrame, currScaledDepthImage, sensorOffset, false);
    refFrameNoNormals = refFrame;

    // Align clouds    
    Rt = Mat::eye(4, 4, CV_32FC1);
    double ostart = get_time();
    cv::RGBDOdometry(Rt, Mat(),
    		     grayCurrentImage, currentDepthFlt, Mat(),
		     grayReferenceImage, referenceDepthFlt, Mat(),
    		     cvCameraMatrix, minDepth, maxDepth, maxDepthDiff,
		     iterCounts, minGradMagnitudes, transformationType); 
    double oend = get_time();
    cout << "time: " << oend - ostart << " seconds" << endl;
    Eigen::Isometry3f T = fromMat(Rt);
    cout << "RT: " << endl << Rt << endl;
    T.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
    // Compute and save avaluation parameters
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
    osBenchmark << dummyLinearizer.inliers() << "\t";
    osBenchmark << chi2 << "\t";
    osBenchmark << oend - ostart << endl;

    // float maxChi2Error = 0.1f;
    // if(chi2 >= maxChi2Error) {
    //   cout << "Got a chi2 error equal to " << chi2 << " which is greater than " << maxChi2Error << "!" << endl;
    //   cout << "Alignmnet " << currDepthFilename << " ---> " << refDepthFilename << " could be wrong!" << endl;
    // }

    currentRelativePose = T;
    currentRelativePose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    currentAbsolutePose = currentAbsolutePose * currentRelativePose;
    currentAbsolutePose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    
    currentAbsolutePoseVector = t2v(currentAbsolutePose);    
    qw = sqrtf(1.0f - (currentAbsolutePoseVector[3]*currentAbsolutePoseVector[3] + 
		       currentAbsolutePoseVector[4]*currentAbsolutePoseVector[4] + 
		       currentAbsolutePoseVector[5]*currentAbsolutePoseVector[5]));
    osAbsoluteTrajectory << currDepthTimestamp << " "
			 << currentAbsolutePoseVector[0] << " " << currentAbsolutePoseVector[1] << " " << currentAbsolutePoseVector[2] << " " 
			 << currentAbsolutePoseVector[3] << " " << currentAbsolutePoseVector[4] << " " << currentAbsolutePoseVector[5] << " " << qw
			 << endl;
    currentRelativePoseVector = t2v(currentRelativePose);
    qw = sqrtf(1.0f - (currentRelativePoseVector[3]*currentRelativePoseVector[3] + 
		       currentRelativePoseVector[4]*currentRelativePoseVector[4] + 
		       currentRelativePoseVector[5]*currentRelativePoseVector[5]));
    osRelativeTrajectory << currDepthTimestamp << " "
			 << currentRelativePoseVector[0] << " " << currentRelativePoseVector[1] << " " << currentRelativePoseVector[2] << " " 
			 << currentRelativePoseVector[3] << " " << currentRelativePoseVector[4] << " " << currentRelativePoseVector[5] << " " << qw
			 << endl;
    
    cout << "Absolute Pose: " << currentAbsolutePoseVector.transpose() << endl;
    cout << "Relative Pose: " << currentRelativePoseVector.transpose() << endl;
    
    refRgbTimestamp = currRgbTimestamp;
    refRgbFilename = currRgbFilename;
    refDepthTimestamp = currDepthTimestamp;
    refDepthFilename = currDepthFilename;

    currFrame.save((currDepthFilename.substr(0, currDepthFilename.size() - 3) + "pwn").c_str(), 5, true, currentAbsolutePose);   

    // Save clouds
    // refFrame.save("reference.pwn", 1, true, Eigen::Isometry3f::Identity());
    // currFrame.save("current.pwn", 1, true, Eigen::Isometry3f::Identity());      
    // currFrame.save("alignedPWN.pwn", 1, true, currentRelativePose);
  }

  return 0;
}

void parseGroundTruth(Isometry3f &initialTransformation, ifstream &isGroundtruth, double targetTimestamp) {
  char line[4096];
  double currentTimestamp;
  initialTransformation = Isometry3f::Identity();
  double minDifference = numeric_limits<float>::max();
  Vector6f initialTransformationVector;
  while (isGroundtruth.getline(line, 4096)) {
    istringstream issGroundtruth(line);
    issGroundtruth >> currentTimestamp;
    if(fabs(currentTimestamp - targetTimestamp) < minDifference) {
      issGroundtruth >> initialTransformationVector[0] >> initialTransformationVector[1] >> initialTransformationVector[2]
		     >> initialTransformationVector[3] >> initialTransformationVector[4] >> initialTransformationVector[5];
      initialTransformation = v2t(initialTransformationVector);
      minDifference = fabs(currentTimestamp - targetTimestamp);
    }
  }
  initialTransformation.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
}

Eigen::Isometry3f fromMat(const cv::Mat m) {
  Eigen::Matrix3f R;
  Eigen::Vector3f t;
  Eigen::Isometry3f T;
  T.setIdentity();
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      T.matrix()(i, j) = m.at<double>(i, j);
    }
  }
  return T;
}

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
