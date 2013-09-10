#include <iostream>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <set>
#include <sys/stat.h>
#include <sys/types.h>

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"

using namespace std;
using namespace Eigen;
using namespace g2o;
using namespace pwn;

struct Pose{
  float tx;
  float ty;
  float tz;
  float qx;
  float qy;
  float qz;
  float qw;

  Pose() {
    tx = ty = tz = qx = qy = qz = qw = 1.0f;
  }
};

set<string> readDirectory(string dir = ".");
void parseGroundTruth(vector<Pose> &groundTruthPoses, ifstream &is, string groundTruthTimeStamp = "");
void poseToIsometry3f(Isometry3f &isometry, Pose &pose);

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
  arg.param("pj_maxDistance", pj_maxDistance, 6, "maximum distance of the points");

  arg.param("ng_minImageRadius", ng_minImageRadius, 10, "Specify the minimum number of pixels composing the square where to take points for a normal computation");
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
  vector<Pose> groundTruthPoses;
  parseGroundTruth(groundTruthPoses, is, groundTruthTimeStamp);
  for (size_t i = 0; i < groundTruthPoses.size(); i++) {
    osGroundTruthTrajectory << groundTruthPoses[i].tx << " " << groundTruthPoses[i].ty << " " << groundTruthPoses[i].tz << " " 
			    << groundTruthPoses[i].qx << " " << groundTruthPoses[i].qy << " " << groundTruthPoses[i].qz << " "  << groundTruthPoses[i].qw
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
  CorrespondenceFinder correspondenceFinder;
  correspondenceFinder.setInlierDistanceThreshold(cf_inlierDistanceThreshold);
  correspondenceFinder.setFlatCurvatureThreshold(cf_flatCurvatureThreshold);  
  correspondenceFinder.setInlierCurvatureRatioThreshold(cf_inlierCurvatureRatioThreshold);
  correspondenceFinder.setInlierNormalAngularThreshold(cosf(cf_inlierNormalAngularThreshold));
  Linearizer linearizer;
  linearizer.setInlierMaxChi2(al_inlierMaxChi2);

  CorrespondenceFinder dummyCorrespondenceFinder;
  dummyCorrespondenceFinder.setInlierDistanceThreshold(cf_inlierDistanceThreshold);
  dummyCorrespondenceFinder.setFlatCurvatureThreshold(1.0f);  
  dummyCorrespondenceFinder.setInlierCurvatureRatioThreshold(std::numeric_limits<float>::max());
  dummyCorrespondenceFinder.setInlierNormalAngularThreshold(cosf(M_PI));
  Linearizer dummyLinearizer;
  dummyLinearizer.setInlierMaxChi2(al_inlierMaxChi2);

#ifdef _PWN_USE_CUDA_
  CuAligner aligner;
  CuAligner dummyAligner;
#else
  Aligner aligner;
  Aligner dummyAligner;
#endif

  aligner.setProjector(&projector);
  aligner.setLinearizer(&linearizer);
  linearizer.setAligner(&aligner);
  aligner.setCorrespondenceFinder(&correspondenceFinder);
  aligner.setInnerIterations(al_innerIterations);
  aligner.setOuterIterations(al_outerIterations);

  dummyAligner.setProjector(&projector);
  dummyAligner.setLinearizer(&dummyLinearizer);
  dummyLinearizer.setAligner(&dummyAligner);
  dummyAligner.setCorrespondenceFinder(&dummyCorrespondenceFinder);
  dummyAligner.setInnerIterations(0);
  dummyAligner.setOuterIterations(1);

  /************************************************************************
   *                       Sequential Alignment                          *
   ************************************************************************/
  size_t i = startingDepthImageIndex;
  size_t j = startingRgbImageIndex;
  size_t k = 0;
  vector<Isometry3f> trajectory;
  trajectory.clear();
  Isometry3f currentPose = Isometry3f::Identity();
  poseToIsometry3f(currentPose, groundTruthPoses[k]);
  trajectory.push_back(currentPose);
  cout << "Initial Pose: " << endl << trajectory[k].matrix() << endl;
  osTrajectory << groundTruthPoses[k].tx << " " << groundTruthPoses[k].ty << " " << groundTruthPoses[k].tz << " " 
	       << groundTruthPoses[k].qx << " " << groundTruthPoses[k].qy << " " << groundTruthPoses[k].qz << " "  << groundTruthPoses[k].qw
	       << endl;
  while (i < depthImagesFilenames.size() - 1 && j < depthImagesFilenames.size() - 1) {
    const string &refDepthImageFilename = depthImagesFilenames[i];
    const string &currDepthImageFilename = depthImagesFilenames[i + 1];
    cout << "-------------------------------------------" << endl;
    cout << "Aligning: " <<  refDepthImageFilename << " ---> " << currDepthImageFilename << endl;

    // Load depth images   
    refDepthImage.load(refDepthImageFilename.c_str());
    currDepthImage.load(currDepthImageFilename.c_str());
    imageRows = refDepthImage.rows();
    imageCols = refDepthImage.cols();
    scaledImageRows = imageRows / ng_scale;
    scaledImageCols = imageCols / ng_scale;
    correspondenceFinder.setSize(scaledImageRows, scaledImageCols);
    dummyCorrespondenceFinder.setSize(scaledImageRows, scaledImageCols);

    // Compute stats
    Frame refFrame, currFrame, refFrameNoNormals, currFrameNoNormals;
    DepthImage::scale(refScaledDepthImage, refDepthImage, ng_scale);
    DepthImage::scale(currScaledDepthImage, currDepthImage, ng_scale);
    converter.compute(refFrame, refScaledDepthImage, sensorOffset, false);
    converter.compute(currFrame, currScaledDepthImage, sensorOffset, false);
    refFrameNoNormals = refFrame;

    // Align clouds
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
    cout << "Time: " << oend - ostart << " seconds" << endl;
    
    // Compute and save avaluation parameters
    dummyAligner.setReferenceFrame(&refFrame);
    dummyAligner.setCurrentFrame(&currFrame);
    dummyAligner.setInitialGuess(aligner.T());
    dummyAligner.setSensorOffset(sensorOffset);
    dummyAligner.align();
    
    dummyLinearizer.setT(aligner.T().inverse());
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

    currentPose = trajectory[k] * aligner.T();
    currentPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    trajectory.push_back(currentPose);

    Vector3f translation = Vector3f(currentPose(0, 3), currentPose(1, 3), currentPose(2, 3));
    Quaternionf quaternion = Quaternionf(currentPose.linear());
    osTrajectory << translation.x() << " " << translation.y() << " " << translation.z() << " " 
		 << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w() 
		 << endl;
    
    // Save clouds
    // refFrame.save("reference.pwn", 1, true, Eigen::Isometry3f::Identity());
    // currFrame.save("current.pwn", 1, true, Eigen::Isometry3f::Identity());      
    // currFrame.save("alignedPWN.pwn", 1, true, aligner.T());

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
    if (filepath.substr(filepath.size() - 3) != "pgm")
      continue;

    filenames.insert(filepath);
  }

  closedir(dp);

  return filenames;
}

void parseGroundTruth(vector<Pose> &groundTruthPoses, ifstream &is, string groundTruthTimeStamp) {
  char line[4096];
  string timeStamp;
  Pose pose;
  bool firstFound = false;
  if (groundTruthTimeStamp == "")
    firstFound = true;
  groundTruthPoses.clear();
  while (is.getline(line, 4096)) {
    // If the line is a comment skip it
    if (line[0] == '#')
      continue;
    
    // If the line is not a comment read the measure and save it
    pose = Pose();
    istringstream iss(line);
    iss >> timeStamp
	>> pose.tx >> pose.ty >> pose.tz
	>> pose.qx >> pose.qy >> pose.qz >> pose.qw;

    // Check if the current measure is the one the user choosed to be the first
    if (!firstFound && timeStamp == groundTruthTimeStamp)
      firstFound = true;

    // If we found starting measure we can begin to save groun truth values
    if (firstFound) {
      groundTruthPoses.push_back(pose);
    }
  }
}

void poseToIsometry3f(Isometry3f &isometry, Pose &pose) {
  Vector3f translation = Vector3f(pose.tx, pose.ty, pose.tz);
  Quaternionf quaternion = Quaternionf(pose.qw, pose.qx, pose.qy, pose.qz);
  quaternion.normalize();
  isometry = Isometry3f::Identity();
  isometry.translation() = translation;
  isometry.linear() = quaternion.toRotationMatrix();
  isometry.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
}
