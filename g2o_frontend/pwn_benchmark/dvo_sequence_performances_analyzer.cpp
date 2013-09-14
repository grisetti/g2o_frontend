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

#undef _PWN_USE_CUDA_

set<string> readDirectory(string dir = ".");
void parseGroundTruth(vector<Vector6f> &groundTruthPoses, ifstream &is, string groundTruthTimeStamp = "");
Eigen::Isometry3f fromMat(const cv::Mat m);
string type2str(int type);

int main(int argc, char **argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string directory, groundTruthTimeStamp, benchmarkFilename, trajectoryFilename;
  string groundTruthTrajectoryFilename, depthImageFilename, rgbImageFilename, groundTruthFilename;
  
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
  arg.param("pj_maxDistance", pj_maxDistance, 6, "maximum distance of the points");

  arg.param("ng_minImageRadius", ng_minImageRadius, 10, "Specify the minimum number of pixels composing the square where to take points for a normal computation");
  arg.param("ng_maxImageRadius", ng_maxImageRadius, 30, "Specify the maximum number of pixels composing the square where to take points for a normal computation");
  arg.param("ng_minPoints", ng_minPoints, 50, "Specify the minimum number of points to be used to compute a normal");
  arg.param("ng_worldRadius", ng_worldRadius, 0.05f, "Specify the max distance for a point to be used to compute a normal");
  arg.param("ng_scale", ng_scale, 1.0f, "Specify the scaling factor to apply on the depth image");
  arg.param("ng_curvatureThreshold", ng_curvatureThreshold, 1.0f, "Specify the max surface curvature threshold for which normals are discarded");
  
  arg.param("cf_inlierDistanceThreshold", cf_inlierDistanceThreshold, 0.5f, "Maximum metric distance between two points to regard them as iniliers");
  arg.param("al_inlierMaxChi2", al_inlierMaxChi2, 1e3, "Max chi2 error value for the alignment step");
  arg.param("sensorType", sensorType, "kinect", "Sensor type: xtion640/xtion320/kinect");

  arg.param("benchmarkFilename", benchmarkFilename, "pwn_benchmark.txt", "Specify the name of the file that will contain the results of the benchmark");
  arg.param("trajectoryFilename", trajectoryFilename, "trajectory.txt", "Specify the name of the file that will contain the trajectory computed");
  arg.param("groundTruthTrajectoryFilename", groundTruthTrajectoryFilename, "ground_trajectory.txt", "Specify the name of the file that will contain the ground truth trajectory");
  arg.param("depthImageFilename", depthImageFilename, "", "Specify the depth image from which the sequence has to start");
  arg.param("rgbImageFilename", rgbImageFilename, "", "Specify the rgb image from which the sequence has to start");
  arg.param("groundTruthFilename", groundTruthFilename, "groundtruth.txt", "Specify the name of the file containing the ground truth values");
  arg.param("groundTruthTimeStamp", groundTruthTimeStamp, "groundtruth.txt", "Specify the time stamp at which the ground truth will be tracked");

  arg.paramLeftOver("directory", directory, ".", "Directory where the program will find the ground truth file and the subfolders depth and rgb containing the images", true);
  
  // Set parser input
  arg.parseArgs(argc, argv);

  // Create benchmark file
  ofstream osBenchmark(benchmarkFilename.c_str());
  if (!osBenchmark) {
    cerr << "Impossible to create the file containing the values of the benchmark... quitting." << endl;
    return 0;
  }
  
  osBenchmark << "inliers\tchi2\ttime" << endl;
  
  // Create trajectory file
  ofstream osTrajectory(trajectoryFilename.c_str());
  if (!osTrajectory) {
    cerr << "Impossible to create the file containing the computed trajectory poses... quitting." << endl;
    return 0;
  }
  
  // Create trajectory file
  ofstream osGroundTruthTrajectory(groundTruthTrajectoryFilename.c_str());
  if (!osGroundTruthTrajectory) {
    cerr << "Impossible to create the file containing the ground truth trajectory poses... quitting." << endl;
    return 0;
  }

  /************************************************************************
   *                       Ground Truth File Parsing                      *
   ************************************************************************/
  cout << "Parsing ground truth file " << groundTruthFilename << "... ";
  ifstream is(groundTruthFilename.c_str());
  if (!is) {
    cerr << "impossible to open ground truth file, quitting program." << endl;
    return 0;
  }
  vector<Vector6f> groundTruthPoses;
  parseGroundTruth(groundTruthPoses, is, groundTruthTimeStamp);
  for (size_t i = 0; i < groundTruthPoses.size(); i++) {
    osGroundTruthTrajectory << groundTruthPoses[i][0] << " " << groundTruthPoses[i][1] << " " << groundTruthPoses[i][2] << " " 
			    << groundTruthPoses[i][3] << " " << groundTruthPoses[i][4] << " " << groundTruthPoses[i][5]
			    << endl;
  }
  cout << "done." << endl;

  /************************************************************************
   *                     Depth Images Directory Parsing                   *
   ************************************************************************/
  cerr << "Parsing directory " << directory + "/depth/" << "... ";
  int startingDepthImageIndex = 0;
  vector<string> depthImagesFilenames;
  set<string> depthImagesFilenamesSet = readDirectory(directory + "/depth");
  for (set<string>::const_iterator it = depthImagesFilenamesSet.begin(); it != depthImagesFilenamesSet.end(); it++) {
    depthImagesFilenames.push_back(*it);
  }
  if (depthImageFilename != "") {
    for (size_t i = 0; i < depthImagesFilenames.size(); i++) {
      if (depthImagesFilenames[i] == directory + "/depth/" + depthImageFilename) {
	startingDepthImageIndex = i;
	break;
      }	
    }
  }
  cout << "done." << endl;

  /************************************************************************
   *                       RGB Images Directory Parsing                   *
   ************************************************************************/
  cerr << "Parsing directory " << directory + "/rgb/" << "... ";
  int startingRgbImageIndex = 0;
  vector<string> rgbImagesFilenames;
  set<string> rgbImagesFilenamesSet = readDirectory(directory + "/rgb");
  for (set<string>::const_iterator it = rgbImagesFilenamesSet.begin(); it != rgbImagesFilenamesSet.end(); it++) {
    rgbImagesFilenames.push_back(*it);
  }
  if (rgbImageFilename != "") {
    for (size_t i = 0; i < rgbImagesFilenames.size(); i++) {
      if (rgbImagesFilenames[i] == directory + "/rgb/" + rgbImageFilename) {
	startingRgbImageIndex = i;
	break;
      }	
    }
  }
  cout << "done." << endl;

  /************************************************************************
   *                       Objects Initialization                         *
   ************************************************************************/
  // Projector
  PinholePointProjector projector;
  int imageRows, imageCols, scaledImageRows = 0, scaledImageCols = 0;
  Matrix3f cameraMatrix, scaledCameraMatrix;
  Isometry3f sensorOffset = Isometry3f::Identity();

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
  const float maxDepth = 10.0f; //in meters
  const float maxDepthDiff = 0.07f; //in meters

  /************************************************************************
   *                       Sequential Alignment                          *
   ************************************************************************/
  size_t i = startingDepthImageIndex;
  size_t j = startingRgbImageIndex;
  size_t k = 0;
  vector<Isometry3f> trajectory;
  trajectory.clear();
  Isometry3f currentPose = Isometry3f::Identity();
  currentPose = v2t(groundTruthPoses[k]);
  trajectory.push_back(currentPose);
  cout << "Initial Pose: " << t2v(trajectory[k]).transpose() << endl;
  osTrajectory << groundTruthPoses[k][0] << " " << groundTruthPoses[k][1] << " " << groundTruthPoses[k][2] << " " 
	       << groundTruthPoses[k][3] << " " << groundTruthPoses[k][4] << " " << groundTruthPoses[k][5]
	       << endl;
  Isometry3f previousT = Isometry3f::Identity();
  previousT.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  Mat referenceImage, currentImage, currentDepth, referenceDepth, referenceDepthFlt, currentDepthFlt, Rt;
  // Set sensor offset 
  sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
  Quaternionf quaternion = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
  sensorOffset.linear() = quaternion.toRotationMatrix();
  sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  while (i < depthImagesFilenames.size() - 1 && j < depthImagesFilenames.size() - 1) {
    const string &refDepthImageFilename = depthImagesFilenames[i];
    const string &currDepthImageFilename = depthImagesFilenames[i + 1];
    const string &refRgbImageFilename = rgbImagesFilenames[i];
    const string &currRgbImageFilename = rgbImagesFilenames[i + 1];
    cout << "-------------------------------------------" << endl;
    cout << "Aligning: " << currDepthImageFilename << " ---> " << refDepthImageFilename << endl;

    // Load depth images   
    refDepthImage.load((refDepthImageFilename.substr(0, refDepthImageFilename.size() - 3) + "pgm").c_str(), true);
    currDepthImage.load((currDepthImageFilename.substr(0, currDepthImageFilename.size() - 3) + "pgm").c_str(), true);
    imageRows = refDepthImage.rows();
    imageCols = refDepthImage.cols();
    scaledImageRows = imageRows / ng_scale;
    scaledImageCols = imageCols / ng_scale;
    dummyCorrespondenceFinder.setSize(scaledImageRows, scaledImageCols);
    referenceDepth = imread(refDepthImageFilename.c_str(), -1);
    referenceDepth.convertTo(referenceDepthFlt, CV_32FC1, 1.0f/1000.0f);
    referenceImage = imread(refRgbImageFilename, 1);
    currentDepth = imread(currDepthImageFilename.c_str(), -1);
    currentDepth.convertTo(currentDepthFlt, CV_32FC1, 1.0f/1000.0f);
    currentImage = imread(currRgbImageFilename.c_str(), 1);
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

    currentPose = trajectory[k] * T;
    currentPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    trajectory.push_back(currentPose);
    
    Vector6f currentVector = Vector6f::Zero();
    currentVector = t2v(currentPose);    
    osTrajectory << currentVector[0] << " " << currentVector[1] << " " << currentVector[2] << " " 
		 << currentVector[3] << " " << currentVector[4] << " " << currentVector[5] 
		 << endl;

    cout << "Relative Pose: " << t2v(T).transpose() << endl;
    cout << "Absolute Pose: " << t2v(currentPose).transpose() << endl;
    
    // if(previousT.matrix() == Matrix4f::Identity())
    //   refFrame.save((refDepthImageFilename.substr(0, refDepthImageFilename.size() - 3) + "pwn").c_str(), 5, true, previousT);
    
    // previousT = previousT * T;
    // currFrame.save((currDepthImageFilename.substr(0, currDepthImageFilename.size() - 3)  + "pwn").c_str(), 5, true, previousT);
    
    // Save clouds
    refFrame.save("reference.pwn", 1, true, Eigen::Isometry3f::Identity());
    currFrame.save("current.pwn", 1, true, Eigen::Isometry3f::Identity());      
    currFrame.save("alignedPWN.pwn", 1, true, T);
    break;

    i++;
    j++;
    k++;
  }

  return 0;
}

set<string> readDirectory(string dir) {
  DIR *dp;
  struct dirent *dirp;
  struct stat filestat;
  set<string> filenames;
  dp = opendir(dir.c_str());
  if (dp == NULL)
    return filenames;
  
  while ((dirp = readdir(dp))) {
    string filepath = dir + "/" + dirp->d_name;

    // If the file is a directory (or is in some way invalid) we'll skip it 
    if (stat(filepath.c_str(), &filestat))
      continue;
    if (S_ISDIR(filestat.st_mode))
      continue;
    if (filepath.substr(filepath.size() - 3) != "png")
      continue;

    filenames.insert(filepath);
  }

  closedir(dp);

  return filenames;
}

void parseGroundTruth(vector<Vector6f> &groundTruthPoses, ifstream &is, string groundTruthTimeStamp) {
  char line[4096];
  string timeStamp;
  Vector6f pose;
  bool firstFound = false;
  if (groundTruthTimeStamp == "")
    firstFound = true;
  groundTruthPoses.clear();
  while (is.getline(line, 4096)) {
    // If the line is a comment skip it
    if (line[0] == '#')
      continue;
    
    // If the line is not a comment read the measure and save it
    pose = Vector6f::Zero();
    istringstream iss(line);
    Quaternionf q;
    iss >> timeStamp
	>> pose[0] >> pose[1] >> pose[2]
	>> q.x() >> q.y() >> q.z() >> q.w();
    q.normalize();
    pose[3] = q.x();
    pose[4] = q.y();
    pose[5] = q.z();

    // Check if the current measure is the one the user choosed to be the first
    if (!firstFound && timeStamp == groundTruthTimeStamp)
      firstFound = true;

    // If we found starting measure we can begin to save groun truth values
    if (firstFound) {
      groundTruthPoses.push_back(pose);
    }
  }
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
