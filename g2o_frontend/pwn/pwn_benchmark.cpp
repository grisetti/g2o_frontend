#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <sstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include "g2o/stuff/command_args.h"
#include "depthimage.h"
#include "pointwithnormal.h"
#include "pointwithnormalstatsgenerator.h"
#include "pointwithnormalaligner.h"
#include "g2o/stuff/timeutil.h"
#include "scene.h"
#include "pixelmapper.h"
#include "scene_merger.h"

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace g2o;

// Function to compute the fitness score of two registered point clouds
double getFitnessScore(pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr input_transformed) {
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  double fitness_score = 0.0;
  
  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);
  // For each point in the source dataset
  // Initialize voxel grid filter object with the leaf size given by the user.
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setLeafSize(0.01, 0.01, 0.01);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredTarget(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredAligned(new pcl::PointCloud<pcl::PointXYZI>);
  sor.setInputCloud(target_cloud);
  sor.filter(*filteredTarget);
  sor.setInputCloud(input_transformed);
  sor.filter(*filteredAligned);
  tree->setInputCloud(filteredTarget);
  int nr = 0;
  for(size_t i = 0; i < filteredAligned->points.size(); ++i) {
    Eigen::Vector4f p1 = Eigen::Vector4f(filteredAligned->points[i].x,
					 filteredAligned->points[i].y,
					 filteredAligned->points[i].z, 0);
    if(isnan(p1(0)) || isnan(p1(1)) || isnan(p1(2)) || isinf(p1(0)) || isinf(p1(1)) || isinf(p1(2)))
      continue;			    					 
					    					 
    // Find its nearest neighbor in the target
    tree->nearestKSearch(filteredAligned->points[i], 1, nn_indices, nn_dists);
		
    // Deal with occlusions (incomplete targets)   
    if(nn_dists[0] > 0.025)//std::numeric_limits<double>::max ())
      continue;
      
    Eigen::Vector4f p2 = Eigen::Vector4f(filteredTarget->points[nn_indices[0]].x,
					 filteredTarget->points[nn_indices[0]].y,
					 filteredTarget->points[nn_indices[0]].z, 0);
    // Calculate the fitness score
    fitness_score += fabs ((p1-p2).squaredNorm());
    nr++;  
  }
  
  if(nr > 0)
    return(fitness_score/nr);
  else
    return(std::numeric_limits<double>::max());
}

// Function to create pcl pont cloud given a depth iamge
void makePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, Mat image, Mat depth, float imageFocalLength) {
  unsigned char* iptr=reinterpret_cast<unsigned char*>(image.data);
  unsigned short* dptr=reinterpret_cast<unsigned short*>(depth.data);
  assert(image.rows == depth.rows && image.cols == depth.cols);
  int w = image.cols;
  int h = image.rows;
  
  float f = imageFocalLength; 
  cloud->height = h;
  cloud->width = w;
  cloud->is_dense = false;
  cloud->points.resize(h*w);
  register float constant = 1.0f/f;
  int v = -h/2;
  int k = 0;
  float bad_point = std::numeric_limits<float>::quiet_NaN();
  
  for(int i = 0; i < image.rows; i++, v++) {
    int u = -w/2;
    for(int j = 0; j < image.cols; j++, u++) {
      pcl::PointXYZI& pt = cloud->points[k];
      unsigned short d = *dptr;
      if(d == 0) {
	pt.x = pt.y = pt.z = bad_point;
	pt.intensity = 0;
      } 
      else {
	pt.z = d*1e-3f;
	pt.x = u * pt.z * constant;
	pt.y = v * pt.z * constant;
	pt.intensity = (float)(*iptr);
      }
      iptr++;
      dptr++;
      k++;
    }
  }
}

Eigen::Isometry3f fromMat(const cv::Mat m) {
  Eigen::Matrix3f R;
  Eigen::Vector3f t;
  Eigen::Isometry3f T;
  T.setIdentity();
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      T.matrix()(i, j) = m.at<double>(i, j);
    }
  }
  return T;
}

// Function to extract all the file names inside a given directory
set<string> readDir(std::string dir){
  DIR *dp;
  struct dirent *dirp;
  struct stat filestat;
  std::set<std::string> filenames;
  dp = opendir(dir.c_str());
  if(dp == NULL) {
    return filenames;
  }
  
  while((dirp = readdir(dp))) {
    string filepath = dir + "/" + dirp->d_name;

    // If the file is a directory (or is in some way invalid) we'll skip it 
    if(stat(filepath.c_str(), &filestat)) 
      continue;
    if(S_ISDIR(filestat.st_mode))         
      continue;

    filenames.insert(filepath);
  }

  closedir(dp);
  return filenames;
}

int main(int argc, char** argv) {
  // Input variables
  string directory, benchmarkFilename;

  float numThreads;

  int ng_step, ng_minPoints, ng_imageRadius;
  float ng_worldRadius, ng_maxCurvature;
  
  float al_inlierDistance, al_inlierCurvatureRatio, al_inlierNormalAngle, al_inlierMaxChi2, al_scale, al_flatCurvatureThreshold, al_outerIterations;
  float al_nonLinearIterations, al_linearIterations, al_minInliers, al_lambda, al_debug, me_distanceThreshold, me_normalThreshold;

  int processEachN;
  
  // Registration objects
  PointWithNormalStatistcsGenerator normalGenerator;
  PointWithNormalAligner aligner, dummyAligner;

  // Merging objects
  SceneMerger merger;
  merger.setNormalGenerator(&normalGenerator);

  // Input parameters handling
  g2o::CommandArgs arg;
  arg.param("ng_step", ng_step, normalGenerator.step(), "compute a normal each x pixels") ;
  arg.param("ng_minPoints", ng_minPoints, normalGenerator.minPoints(), "minimum number of points in a region to compute the normal");
  arg.param("ng_imageRadius", ng_imageRadius, normalGenerator.imageRadius(), "radius of the sphere where to compute the normal for a pixel defined");
  arg.param("ng_worldRadius", ng_worldRadius,  normalGenerator.worldRadius(), "radius of the sphere where to compute the normal for a pixel");
  arg.param("ng_maxCurvature", ng_maxCurvature, normalGenerator.maxCurvature(), "above this threshold the normal is not computed");
  
  arg.param("al_inlierDistance", al_inlierDistance, aligner.inlierDistanceThreshold(), "max metric distance between two points to regard them as iniliers");
  arg.param("al_inlierCurvatureRatio", al_inlierCurvatureRatio, aligner.inlierCurvatureRatioThreshold(), "max curvature difference between two points to regard them as iniliers");
  arg.param("al_inlierNormalAngle", al_inlierNormalAngle, aligner.inlierNormalAngularThreshold(), "max normals angle difference between two points to regard them as iniliers");
  arg.param("al_inlierMaxChi2", al_inlierMaxChi2, aligner.inlierMaxChi2(), "max metric distance between two points to regard them as iniliers");
  arg.param("al_minInliers", al_minInliers, aligner.minInliers(), "minimum numver of inliers to do the matching");
  arg.param("al_scale", al_scale, aligner.scale(), "scale of the range image for the alignment");
  arg.param("al_flatCurvatureThreshold", al_flatCurvatureThreshold, aligner.flatCurvatureThreshold(), "curvature above which the patches are not considered planar");
  arg.param("al_outerIterations", al_outerIterations, aligner.outerIterations(), "outer interations (incl. data association)");
  arg.param("al_linearIterations", al_linearIterations, aligner.linearIterations(), "linear iterations for each outer one (uses R,t)");
  arg.param("al_nonLinearIterations", al_nonLinearIterations, aligner.nonLinearIterations(), "nonlinear iterations for each outer one (uses q,t)");
  arg.param("al_lambda", al_lambda, aligner.lambda(), "damping factor for the transformation update, the higher the smaller the step");
  arg.param("al_debug", al_debug, aligner.debug(), "prints lots of stuff");
  arg.param("me_distanceThreshold", me_distanceThreshold, merger.distanceThreshold(), "distance along the z when to merge points");
  arg.param("me_normalThreshold", me_normalThreshold, acos(merger.normalThreshold()), "distance along the z when to merge points");


  arg.param("numThreads", numThreads, 1, "numver of threads for openmp");
  arg.param("processEachN", processEachN, 1, "compute an image every X") ;
 
  arg.paramLeftOver("working directory", directory, ".", "directory where are the file to use", true);
  arg.paramLeftOver("bench-file", benchmarkFilename, "bench.m", "name of the file that will contains the benchmark data", true);

  arg.parseArgs(argc, argv);

  // Calibration matrix
  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  
  // Extract all the file names inside the working path
  std::vector<string> filenames;
  std::set<string> filenamesset = readDir(directory);
  for(set<string>::const_iterator it =filenamesset.begin(); it!=filenamesset.end(); it++) {
    filenames.push_back(*it);
  }
  cout << "There are " << filenames.size() << " files inside the working directory" << endl;

  // Set rgbd registration variables
  float vals[] = {525.0f, 0.0f, 319.5f,
		  0.0f, 525.0f, 239.5f,
		  0.0f, 0.0f, 1.0f};
  float identVals[] = {1.0f, 0.0f, 0.0f, 0.0f,
		       0.0f, 1.0f, 0.0f, 0.0f,
		       0.0f, 0.0f, 1.0f, 0.0f,
		       0.0f, 0.0f, 0.0f, 1.0f};
  float focalLength = 575;                
  Mat cvCameraMatrix = Mat(3, 3, CV_32FC1, vals);
  Mat distCoeff(1, 5, CV_32FC1, Scalar(0));

  pcl::PointCloud<pcl::PointXYZI> cloudAligned;
  int transformationType = RIGID_BODY_MOTION;
  Mat referenceImage, currentImage;
  Mat currentDepth, referenceDepth, referenceDepthFlt, currentDepthFlt;
  Mat Rt;
  Isometry3f rgbdTrajectory = Isometry3f::Identity();

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
  
  const float minDepth = 0.f; // in meters
  const float maxDepth = 10.f; // in meters
  const float maxDepthDiff = 0.07f; // in meters

  // Set normal generator and the aligner
  normalGenerator.setStep(ng_step);
  normalGenerator.setMinPoints(ng_minPoints);
  normalGenerator.setImageRadius(ng_imageRadius);
  normalGenerator.setWorldRadius(ng_worldRadius);
  normalGenerator.setMaxCurvature(ng_maxCurvature);
#ifdef _PWN_USE_OPENMP_
  normalGenerator.setNumThreads(numThreads);
#endif //_PWN_USE_OPENMP_

  aligner.setInlierDistanceThreshold(al_inlierDistance);
  aligner.setInlierCurvatureRatioThreshold(al_inlierCurvatureRatio);
  aligner.setInlierNormalAngularThreshold(al_inlierNormalAngle);
  aligner.setInlierMaxChi2(al_inlierMaxChi2);
  aligner.setMinInliers(al_minInliers);
  aligner.setScale(al_scale);
  aligner.setFlatCurvatureThreshold(al_flatCurvatureThreshold);
  aligner.setOuterIterations(al_outerIterations);
  aligner.setLinearIterations(al_linearIterations);
  aligner.setNonLinearIterations(al_nonLinearIterations);
  aligner.setLambda(al_lambda);
  aligner.setDebug(al_debug);
#ifdef _PWN_USE_OPENMP_
  aligner.setNumThreads(numThreads);
#endif //_PWN_USE_OPENMP_

  // Set dummy aligner
  dummyAligner.setInlierDistanceThreshold(0.05f);
  dummyAligner.setInlierCurvatureRatioThreshold(al_inlierCurvatureRatio);
  dummyAligner.setInlierNormalAngularThreshold(al_inlierNormalAngle);
  dummyAligner.setInlierMaxChi2(al_inlierMaxChi2);
  dummyAligner.setMinInliers(al_minInliers);
  dummyAligner.setScale(al_scale);
  dummyAligner.setFlatCurvatureThreshold(al_flatCurvatureThreshold);
  dummyAligner.setOuterIterations(1);
  dummyAligner.setLinearIterations(0);
  dummyAligner.setNonLinearIterations(1);
  dummyAligner.setLambda(al_lambda);
  dummyAligner.setDebug(al_debug);
#ifdef _PWN_USE_OPENMP_
  dummyAligner.setNumThreads(numThreads);
#endif //_PWN_USE_OPENMP_

  // Set the merger 
  float mergerScale =al_scale;
  Matrix3f mergerCameraMatrix = cameraMatrix;
  mergerCameraMatrix.block<2, 3>(0, 0) *= mergerScale;
  merger.setImageSize(480*mergerScale, 640*mergerScale);
  merger.setCameraMatrix(mergerCameraMatrix);
  merger.setDistanceThreshold(me_distanceThreshold);
  merger.setNormalThreshold(cos(me_normalThreshold));
  
  // Set support variables necessary for pwn sequential registration
  DepthFrame* referenceFrame = 0;
  string baseFilename = benchmarkFilename.substr(0, benchmarkFilename.find_last_of('.'));
  Scene* globalScene = new Scene;
  Scene* partialScene = new Scene;

  Isometry3f trajectory;
  trajectory.setIdentity();
  
  // ICP variables
  Isometry3f icpTrajectory = Isometry3f::Identity();

  // Time variables
  double ostart, oend;

  // Benchamark file streams
  ofstream pwnOS;
  pwnOS.clear();
  pwnOS.open("./benchmark_results/pwnm_benchmark.txt", ios::out);
  /*ofstream rgbdOS;
  rgbdOS.clear();
  rgbdOS.open("./benchmark_results/rgbd_benchmark.txt", ios::out);
  ofstream icpOS;
  icpOS.clear();
  icpOS.open("./benchmark_results/icp_benchmark.txt", ios::out);*/

  // Start sequential registration
  for(size_t i = 0; i < filenames.size(); i += processEachN) {
    cout << endl << endl << endl;
    cout << ">>>>>>>>>>>>>>>>>>>>>>>> PROCESSING " << filenames[i] << " <<<<<<<<<<<<<<<<<<<<<<<<" << endl;            
    
    // Load current frame
    DepthFrame* currentFrame = new DepthFrame();
    currentDepth = imread(filenames[i].c_str(), -1);
    currentDepth.convertTo(currentDepthFlt, CV_32FC1, 1./1000);
    currentImage = imread(("intensity/" + filenames[i]).c_str(), -1);
    DepthImage depthImage;
    if(!depthImage.load(filenames[i].c_str())) {
      cout << "Failure while loading the depth image: " << filenames[i] << ", skipping" << endl;
      delete currentFrame;
      break;
    }

    // Set current frame parameters
    currentFrame->_cameraMatrix = cameraMatrix;
    currentFrame->_baseline = 0.075;
    currentFrame->_maxDistance = 10;
    currentFrame->setImage(depthImage);
    currentFrame->_updateSVDsFromPoints(normalGenerator, cameraMatrix);
    currentFrame->_suppressNoNormals();

    if(referenceFrame) {
      // Create pcl point cloud for the fitting
      pcl::PointCloud<pcl::PointXYZI>::Ptr referenceCloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZI>);
      makePointCloud(referenceCloud, referenceImage, referenceDepth, focalLength);
      makePointCloud(currentCloud, currentImage, currentDepth, focalLength);
      
      // Set inputs for the aligner
      globalScene->subScene(*partialScene,cameraMatrix,trajectory);
      //aligner.setReferenceScene(referenceFrame);
      aligner.setReferenceScene(partialScene);
      aligner.setCurrentScene(currentFrame);      
      dummyAligner.setReferenceScene(referenceFrame);
      //dummyAligner.setReferenceScene(partialScene);
      dummyAligner.setCurrentScene(currentFrame);

      // Compute registration
      aligner.setImageSize(currentFrame->image().rows(), currentFrame->image().cols());
      dummyAligner.setImageSize(currentFrame->image().rows(), currentFrame->image().cols());
      Matrix6f omega;
      Vector6f mean;
      float tratio, rratio, error;
      Eigen::Isometry3f X = trajectory;
      ostart = get_time();
      int result = aligner.align(error, X, mean, omega, tratio, rratio);
      oend = get_time();
      cout << "Inliers: " << result << " --- Error: " << error << " --- Error/Inliers Ratio: " << error/result << endl;
      cout << "Local Transform : " << endl;
      cout << (trajectory.inverse()*X).matrix() << endl;
      Isometry3f tmp = trajectory.inverse()*X;
      trajectory = X;
      cout << "Global Transform: " << endl;
      cout << trajectory.matrix() << endl;
      cout << "Alignment took: " << oend-ostart << " sec." << endl;
      cout << "Aligner scaled image size: " << aligner.scaledImageRows() << " " << aligner.scaledImageCols() << endl;
      result = dummyAligner.align(error, tmp, mean, omega, tratio, rratio, true); 
      Eigen::Isometry3f motionFromFirstFrame = trajectory;
      Eigen::AngleAxisf rotationFromFirstFrame(trajectory.linear());
      cout << "Motion from first frame: " << motionFromFirstFrame.translation().transpose() << " d:" << motionFromFirstFrame.translation().norm() << endl;
      cout << "Rotation from first frame: " << rotationFromFirstFrame.axis().transpose() << " a:" << rotationFromFirstFrame.angle() << endl;
      float pwnScore = 1000000.0f;
      if(result > 0) {
      	//pcl::transformPointCloud(*currentCloud, cloudAligned, (trajectory.inverse()*X).translation(), Quaternionf((trajectory.inverse()*X).linear()));
      	//pwnScore = getFitnessScore(referenceCloud, cloudAligned.makeShared());
      }	
      Quaternionf pwnq = Quaternionf(X.linear());
      pwnOS << result << "\t" << error << "\t" << error/result << "\t" << oend-ostart << "\t" << motionFromFirstFrame.translation().norm() << "\t" << rotationFromFirstFrame.angle() << "\t" << pwnScore << "\t" << X.translation().x() << "\t" << X.translation().y() << "\t" << X.translation().z() << "\t" << pwnq.x() << "\t" << pwnq.y() << "\t" << pwnq.z() << "\t" << pwnq.w() << endl;

      //********************************************************************
      /*cout << "*************************************************************" << endl;
      Rt = Mat(4, 4, CV_32FC1, identVals);
      ostart = get_time();
      bool isFound = cv::RGBDOdometry(Rt, Mat(),
				      referenceImage, referenceDepthFlt, Mat(),
				      currentImage, currentDepthFlt, Mat(),
				      cvCameraMatrix, minDepth, maxDepth, maxDepthDiff,
				      iterCounts, minGradMagnitudes, transformationType);
      oend = get_time();
      if(isFound)
	cout << "Transformation found" << endl;
      else
	cout << "Transformation not found" << endl;
      Eigen::Isometry3f rgbdX = fromMat(Rt);
      result = dummyAligner.align(error, rgbdX, mean, omega, tratio, rratio, true);
      cout << "Inliers: " << result << " --- Error: " << error << " --- Error/Inliers Ratio: " << error/result << endl;
      cout << "Local Transform : " << endl;
      cout << rgbdX.matrix() << endl;
      rgbdTrajectory = rgbdTrajectory*rgbdX;
      cout << "Global Transform: " << endl;
      cout << rgbdTrajectory.matrix() << endl;
      cout << "Alignment took: " << oend-ostart << " sec." << endl;
      cout << "Aligner scaled image size: " << dummyAligner.scaledImageRows() << " " << dummyAligner.scaledImageCols() << endl;
      Eigen::Isometry3f rgbdMotionFromFirstFrame = rgbdTrajectory;
      Eigen::AngleAxisf rgbdRotationFromFirstFrame(rgbdTrajectory.linear());
      cout << "Motion from first frame: " << rgbdMotionFromFirstFrame.translation().transpose() << " --> d:" << rgbdMotionFromFirstFrame.translation().norm() << endl;
      cout << "Rotation from first frame: " << rgbdRotationFromFirstFrame.axis().transpose() << " --> a:" << rgbdRotationFromFirstFrame.angle() << endl;
      
      float rgbdScore = 1000000.0f;
      if(result > 0) {
	//        pcl::transformPointCloud(*currentCloud, cloudAligned, rgbdX.translation(), Quaternionf(rgbdX.linear()));
        //rgbdScore = getFitnessScore(referenceCloud, cloudAligned.makeShared());
      }
      Quaternionf rgbdq = Quaternionf(rgbdX.linear());
      rgbdOS << result << "\t" << error << "\t" << error/result << "\t" << oend-ostart << "\t" << rgbdMotionFromFirstFrame.translation().norm() << "\t" << rgbdRotationFromFirstFrame.angle() << "\t" << rgbdScore << "\t" << rgbdX.translation().x() << "\t" << rgbdX.translation().y() << "\t" << rgbdX.translation().z() << "\t" << rgbdq.x() << "\t" << rgbdq.y() << "\t" << rgbdq.z() << "\t" << rgbdq.w() << endl;

      // *******************************************************************
      cout << "*************************************************************" << endl;
      std::vector<int> indices;
      pcl::PointCloud<pcl::PointXYZI>::Ptr referenceCloudNoNaN(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloudNoNaN(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::removeNaNFromPointCloud(*referenceCloud, *referenceCloudNoNaN, indices);
      pcl::removeNaNFromPointCloud(*currentCloud, *currentCloudNoNaN, indices);
      
      pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
       // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
      icp.setMaxCorrespondenceDistance(0.05);
      // Set the maximum number of iterations
      icp.setMaximumIterations(50);
      // Set the transformation epsilon
      icp.setTransformationEpsilon(1e-8);
      // Set the euclidean distance difference epsilon
      icp.setEuclideanFitnessEpsilon(1);
      icp.setInputCloud(currentCloudNoNaN);
      icp.setInputTarget(referenceCloudNoNaN);
      ostart = get_time();
      icp.align(cloudAligned);
      oend = get_time();
      
      if(icp.hasConverged())
	std::cout << "ICP has converged " << icp.getFitnessScore() << endl;
      else
	std::cout << "ICP has not converged " << icp.getFitnessScore() << endl;
      Eigen::Matrix4f transformation = icp.getFinalTransformation();
      Isometry3f icpX(transformation);
      result = dummyAligner.align(error, icpX, mean, omega, tratio, rratio, true);
      cout << "Inliers: " << result << " --- Error: " << error << " --- Error/Inliers Ratio: " << error/result << endl;
      cout << "Local Transform : " << endl;
      cout << icpX.matrix() << endl;
      icpTrajectory = icpTrajectory*icpX;
      cout << "Global Transform: " << endl;
      cout << icpTrajectory.matrix() << endl;
      cout << "Alignment took: " << oend-ostart << " sec." << endl;
      cout << "Aligner scaled image size: " << dummyAligner.scaledImageRows() << " " << dummyAligner.scaledImageCols() << endl;
      Eigen::Isometry3f icpMotionFromFirstFrame = icpTrajectory;
      Eigen::AngleAxisf icpRotationFromFirstFrame(icpTrajectory.linear());
      cout << "Motion from first frame: " << icpMotionFromFirstFrame.translation().transpose() << " --> d:" << icpMotionFromFirstFrame.translation().norm() << endl;
      cout << "Rotation from first frame: " << icpRotationFromFirstFrame.axis().transpose() << " --> a:" << icpRotationFromFirstFrame.angle() << endl;
      float icpScore = 1000000.0f;
      if(result > 0) {
	//	icpScore = getFitnessScore(referenceCloud, cloudAligned.makeShared());
      }
      Quaternionf icpq = Quaternionf(icpX.linear());
      icpOS << result << "\t" << error << "\t" << error/result << "\t" << oend-ostart << "\t" << icpMotionFromFirstFrame.translation().norm() << "\t" << icpRotationFromFirstFrame.angle() << "\t" << icpScore << "\t" << icpX.translation().x() << "\t" << icpX.translation().y() << "\t" << icpX.translation().z() << "\t" << icpq.x() << "\t" << icpq.y() << "\t" << icpq.z() << "\t" << icpq.w() << endl;

      // *******************************************************************
      cout << "Fitness scores: " << endl 
	   << "pwn \t --> " << pwnScore << endl 
	   << "rgbd \t --> " << rgbdScore << endl
	   << "icp \t --> " << icpScore << endl;*/
    }

    if(referenceFrame)
      delete referenceFrame;
    referenceFrame = 0;
    referenceFrame = currentFrame;
    referenceDepth = currentDepth;
    referenceDepthFlt = currentDepthFlt;
    referenceImage = currentImage;
    globalScene->add(*currentFrame,trajectory);
    merger.merge(globalScene, trajectory);
  }

  pwnOS.close();
  /*rgbdOS.close();
    icpOS.close();*/

  return 0;
}
