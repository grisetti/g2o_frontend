#include <dirent.h>
#include <fstream>
#include <set>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"

#undef _PWN_USE_CUDA_

#ifdef _PWN_USE_CUDA_
#include "g2o_frontend/pwn_cuda/cualigner.h"
#endif// PWN_CUDA

using namespace std;
using namespace g2o;
using namespace Eigen;
using namespace pwn;

set<string> readDirectory(string dir);

int main(int argc, char** argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string directory;
  string g2o_filename;

  // Variables for the input parameters. 
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
  string sensorType;

  // Input parameters handling.
  CommandArgs arg;
  
  // Optional input parameters.
  arg.param("ng_minImageRadius", ng_minImageRadius, 10, "Specify the minimum number of pixels composing the square where to take points for a normal computation");
  arg.param("ng_maxImageRadius", ng_maxImageRadius, 30, "Specify the maximum number of pixels composing the square where to take points for a normal computation");
  arg.param("ng_minPoints", ng_minPoints, 50, "Specify the minimum number of points to be used to compute a normal");
  arg.param("ng_worldRadius", ng_worldRadius, 0.1f, "Specify the max distance for a point to be used to compute a normal");
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
  arg.param("al_inlierMaxChi2", al_inlierMaxChi2, 9e3, "Max chi2 error value for the alignment step");
  arg.param("sensorType", sensorType, "kinect", "sensor type: xtion640/xtion480/kinect");

  // Last parameter has to be the working directory.
  arg.paramLeftOver("directory", directory, "", "directory were the depth images are saved ", true);
  arg.paramLeftOver("g2o_ouput_filename", g2o_filename, "", "g2o output filename", true);

  // Set parser input.
  arg.parseArgs(argc, argv);

  cerr << "Directory to process " << directory << endl;

  vector<string> filenames;
  set<string> filenamesSet = readDirectory(directory);
  for(set<string>::const_iterator it = filenamesSet.begin(); it != filenamesSet.end(); it++) {
    filenames.push_back(*it);
  }
  cerr << "Starting to process " << filenames.size() << " files in the given directory" << endl; 

  /************************************************************************
   *                           Objects Initialization                     *
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

  if (sensorType=="xtion640") {
    cameraMatrix << 
    570.342, 0,       320,
    0,       570.342, 240,
    0.0f, 0.0f, 1.0f;  
  }  else if (sensorType=="xtion320") {
    cameraMatrix << 
      570.342, 0,       320,
      0,       570.342, 240,
      0.0f, 0.0f, 1.0f;  
    cameraMatrix.block<2,3>(0,0)*=0.5;
  } else if (sensorType=="kinect") {
    cameraMatrix << 
      525.0f, 0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f, 0.0f, 1.0f;  
  } else {
    cerr << "unknown sensor type: [" << sensorType << "], aborting (you need to specify either xtion or kinect)" << endl;
    return 0;
  }

  // kinect
  // cameraMatrix << 
  float scale = 1.0f / ng_scale;
  scaledCameraMatrix = cameraMatrix * scale;
  scaledCameraMatrix(2, 2) = 1.0f;
  
  // Set the camera matrix to the pinhole point projector
  projector.setCameraMatrix(scaledCameraMatrix);
    
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
  converter._curvatureThreshold = ng_curvatureThreshold;
  DepthImage depthImage, scaledDepthImage;
  Eigen::MatrixXi indexImage, scaledIndexImage;

  // Correspondece finder and linearizer
  CorrespondenceFinder correspondenceFinder;
  correspondenceFinder.setInlierDistanceThreshold(cf_inlierDistanceThreshold);
  correspondenceFinder.setFlatCurvatureThreshold(cf_flatCurvatureThreshold);  
  correspondenceFinder.setInlierCurvatureRatioThreshold(cf_inlierCurvatureRatioThreshold);
  correspondenceFinder.setInlierNormalAngularThreshold(cosf(cos(cf_inlierNormalAngularThreshold)));
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

  /************************************************************************
   *                           Processing Files                           *
   ************************************************************************/
  Frame *referenceFrame = 0;
  int numFrames = 0;
  int graphNum = 0;
  int previousIndex=-1;
  Isometry3f trajectory = Isometry3f::Identity();
  trajectory.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  string baseFilename = g2o_filename.substr(0, g2o_filename.find_last_of('.'));
  ostringstream os(g2o_filename.c_str());
  bool restarted = true;
  for(size_t i = 0; i < filenames.size(); i++) {
    cerr << ">>>>>>>>>>>>>>>>>>>>>>>> PROCESSING " << filenames[i] << " <<<<<<<<<<<<<<<<<<<<<<<<" <<  endl;

    // Load depth image
    if(!depthImage.load(filenames[i].c_str(), true)) {
      cerr << "Impossible to load image " << filenames[i] << ", skipping" << endl;
      continue;
    }
    numFrames++;

    // If it is the first frame add the parameter
    if(restarted || ! referenceFrame) {
      cerr << "@@@@@@@@@@@@@@@@@@@@2REFFREMA" << endl;
      os << "PARAMS_CAMERACALIB 0 " 
	 << sensorOffset.translation().x() << " " 
	 << sensorOffset.translation().y() << " " 
	 << sensorOffset.translation().z() << " "
	 << quaternion.x() << " "
	 << quaternion.y() << " "
	 << quaternion.z() << " "
	 << quaternion.w() << " "
	 << cameraMatrix(0, 0) << " "
	 << cameraMatrix(1, 1) << " "
	 << cameraMatrix(0, 2) << " "
	 << cameraMatrix(1, 2) << " " << endl;

      imageRows = depthImage.rows();
      imageCols = depthImage.cols();
      scaledImageRows = imageRows / ng_scale;
      scaledImageCols = imageCols / ng_scale;
      correspondenceFinder.setSize(scaledImageRows, scaledImageCols);

      trajectory = Isometry3f::Identity();
      trajectory.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      restarted = false;
    }

    // Add the vertex
    Vector6f x = t2v(trajectory);
    Vector3f t = x.head<3>();
    Vector3f mq = x.tail<3>();
    float w = mq.squaredNorm();
    if( w > 1) {
      mq.setZero();
      w = 1.0f;
    } 
    else {
      w = sqrtf(1 - w);
    }
    
    os << "VERTEX_SE3:QUAT " << i << " " << t.transpose() << " " << mq.transpose() << " " << w << endl;
    os << "RGBD_DATA 0 " << filenames[i] << " 0 hostname 0 " << endl;
  
    // Compute stats
    Frame *currentFrame = new Frame();
    DepthImage::scale(scaledDepthImage, depthImage, ng_scale);
    converter.compute(*currentFrame, scaledDepthImage, sensorOffset, false);

    // Align
    if(referenceFrame) {
      Isometry3f initialGuess = Isometry3f::Identity();
      initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      aligner.setReferenceFrame(referenceFrame);
      aligner.setCurrentFrame(currentFrame);
      aligner.setInitialGuess(initialGuess);
      aligner.setSensorOffset(sensorOffset);
      double ostart = get_time();
      aligner.align();
      double oend = get_time();

      Eigen::Isometry3f localTransformation = aligner.T();
      trajectory = trajectory * localTransformation;
      trajectory.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      cerr << "Local transform : " << endl << localTransformation.matrix() << endl;
      cerr << "Global transform: " << endl << trajectory.matrix() << endl;
      cerr << "Inliers/Minimum number of inliers: " << aligner.inliers() << " / " << al_minNumInliers << endl;
      if(aligner.inliers() != 0)
	cerr << "Error: " << aligner.error() / aligner.inliers() << " / " << al_minError << endl;
      else
	cerr << "Error: " << std::numeric_limits<float>::max() << " / " << al_minError << endl;
      cerr << "Alignment took: " << oend - ostart << " seconds" << endl;
      cerr << "Aligner scaled image size: " << scaledImageRows << "x" << scaledImageCols << endl;
      // If the aligner fails write down the g2o file
      if(aligner.inliers() < al_minNumInliers || aligner.error() / aligner.inliers() > al_minError) {
	cerr << "ALIGNER FAILURE" << endl;
	if(numFrames > 10) {
	  char buff[1024];
	  sprintf(buff, "%s-%03d.g2o", baseFilename.c_str(), graphNum);	
	  ofstream gs(buff);
	  gs << os.str();
	  gs.close();
	}
	os.str("");
	os.clear();
	if(referenceFrame)
	  delete referenceFrame;
	referenceFrame = 0;
	graphNum++;
	numFrames = 0;
	restarted=true;
      }
      // If the aligner does not fail add an edge between the two verteces 
      else {
	Vector6f x = t2v(localTransformation);
	Vector3f t = x.head<3>();
	Vector3f mq = x.tail<3>();
	float w = mq.squaredNorm();
	if(w > 1) {
	  mq.setZero();
	  w = 1.0f;
	} 
	else {
	  w = sqrtf(1 - w);
	}
	
	os << "EDGE_SE3:QUAT " << previousIndex << " " << i << " ";
	os << t.transpose() << " " << mq.transpose() << " " << w <<  " ";
	for(int r = 0; r < 6; r++) {
	  for(int c = r; c < 6; c++) {
	    if(r == c) {
	      if(r < 3)
		os << "100 ";
	      else
		os << "1000 ";
	    }
	    else
	      os << "0 ";
	  }
	} 
	os << endl;
	restarted = false;
      }
    }

    previousIndex = i;
    if(referenceFrame)
      delete referenceFrame;
    referenceFrame = currentFrame;
  }

  char buff[1024];
  sprintf(buff, "%s-%03d.g2o", baseFilename.c_str(), graphNum);	
  ofstream gs(buff);
  gs << os.str();
  gs.close();
  return 0;
}

set<string> readDirectory(string dir) {
  DIR *dp;
  struct dirent *dirp;
  struct stat filestat;
  set<string> filenames;
  dp = opendir(dir.c_str());
  if(dp == NULL) {
    return filenames;
  }
  
  while((dirp = readdir(dp))) {
    string filepath = dir + "/" + dirp->d_name;

    // If the file is a directory (or is in some way invalid) we'll skip it 
    if(stat(filepath.c_str(), &filestat)) {
      continue;
    }
    if(S_ISDIR(filestat.st_mode)) {
      continue;
    }

    filenames.insert(filepath);
  }

  closedir(dp);

  return filenames;
}
