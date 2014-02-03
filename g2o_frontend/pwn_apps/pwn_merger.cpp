#include <fstream>
#include <iostream>
#include <string>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <set>

#include <opencv2/highgui/highgui.hpp>

#include "g2o_frontend/pwn_core/pwn_static.h"
#include "g2o_frontend/pwn_core/pinholepointprojector.h"
#include "g2o_frontend/pwn_core/statscalculatorintegralimage.h"
#include "g2o_frontend/pwn_core/depthimageconverterintegralimage.h"
#include "g2o_frontend/pwn_core/merger.h"

using namespace Eigen;
using namespace std;
using namespace pwn;

set<string> readDirectory(string directory);

int main(int argc, char **argv) {
  // Input handling
  if(argc < 2 || 
     string(argv[1]) == "-h" || string(argv[1]) == "-help" || 
     string(argv[1]) == "--h" || string(argv[1]) == "--help") {
    std::cout << "USAGE: ";
    std::cout << "pwn_cloud_prop_viewer depth_images_directory" << std::endl;
    std::cout << "   depth_images_directory \t-->\t directory where the depth images to process are located" << std::endl;
    return 0;
  }
  string directory(argv[1]);    
  vector<string> filenames;
  set<string> filenamesSet = readDirectory(directory);
  for(set<string>::const_iterator it = filenamesSet.begin(); it != filenamesSet.end(); it++) {
    filenames.push_back(*it);
  }

  // Init PWN objects
  Matrix3f cameraMatrix;
  cameraMatrix <<
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f,   0.0f,   1.0f;
  Isometry3f sensorOffset = Isometry3f::Identity();
  sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  int imageRows = 480, imageCols = 640;
  PinholePointProjector pointProjector;
  pointProjector.setCameraMatrix(cameraMatrix);
  pointProjector.setImageSize(imageRows, imageCols);
  pointProjector.setMaxDistance(2.0f);

  float curvatureThreshold = 1.0f;
  StatsCalculatorIntegralImage statsCalculator;
  statsCalculator.setCurvatureThreshold(curvatureThreshold);

  PointInformationMatrixCalculator pointInformationMatrixCalculator;
  NormalInformationMatrixCalculator normalInformationMatrixCalculator;  
  pointInformationMatrixCalculator.setCurvatureThreshold(curvatureThreshold);
  normalInformationMatrixCalculator.setCurvatureThreshold(curvatureThreshold);

  DepthImageConverterIntegralImage converter(&pointProjector, &statsCalculator,
					     &pointInformationMatrixCalculator, 
					     &normalInformationMatrixCalculator);

  RawDepthImage rawDepthImage;
  DepthImage depthImage;
  Cloud pointCloud, globalCloud, mergedCloud;

  for(size_t i = 0; i < filenames.size(); i++) {
    std::cout << "Processing " << filenames[i] << endl;

    // Load current depth image and add it to the global point cloud
    rawDepthImage = cv::imread(filenames[i], CV_LOAD_IMAGE_UNCHANGED);
    if(rawDepthImage.data == NULL) {
      std::cerr << "ERROR: impossible to load input .pgm depth image " << filenames[i] << " ... skipping!"<<endl;
      continue;
    }   
    DepthImage_convert_16UC1_to_32FC1(depthImage, rawDepthImage, 0.001f);
    converter.compute(pointCloud, depthImage, sensorOffset); 
    globalCloud.add(pointCloud);
    std::cout << "Added " << pointCloud.points().size() << " to the global point cloud that now has " 
	      << globalCloud.points().size() << " points." << std::endl; 
  }
  
  globalCloud.save("globalCloud.pwn", Eigen::Isometry3f::Identity(), true);

  Merger merger;
  merger.setDepthImageConverter(&converter);
  merger.setImageSize(480, 640);
  merger.merge(&globalCloud, Eigen::Isometry3f::Identity());
  globalCloud.save("mergedCloud.pwn", Eigen::Isometry3f::Identity(), true);

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

    // If the file name does not ends with the string "_depth.pgm" skip it
    if(filepath.rfind(".pgm") == string::npos) {
      continue;
    }
    
    filenames.insert(filepath);
  }

  closedir(dp);

  return filenames;
}
