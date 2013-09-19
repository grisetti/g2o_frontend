#include "g2o_frontend/pwn_benchmark/pwn_performances_analyzer.h"

using namespace std;
using namespace Eigen;
using namespace g2o;
using namespace pwn;

int main(int argc, char **argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string directory, benchmarkFilename, absoluteTrajectoryFilename, relativeTrajectoryFilename;
  string groundTruthFilename, associationsFilename;
  int verbose;
  int ng_minImageRadius;
  int ng_maxImageRadius;
  int pj_imageRows;
  int pj_imageCols;
  int ng_minPoints;
  int chunkStep;

  float ng_worldRadius;
  float ng_scale;
  float ng_curvatureThreshold;

  float cf_inlierNormalAngularThreshold;
  float cf_flatCurvatureThreshold;
  float cf_inlierCurvatureRatioThreshold;
  float cf_inlierDistanceThreshold;

  int al_innerIterations;
  int al_outerIterations;
  float al_inlierMaxChi2;
  float pj_maxDistance;
  float di_scaleFactor;
  string sensorType;

  // Input parameters handling
  CommandArgs arg;
  
  // Optional input parameters
  arg.param("pj_maxDistance", pj_maxDistance, 6, "Maximum distance of the points");
  arg.param("verbose", verbose, 0, "Verbose level: 0/1");
  arg.param("pj_imageRows", pj_imageRows, 480, "Number of rows of the depth images");
  arg.param("pj_imageCols", pj_imageCols, 640, "Number of columns of the depth images");
  arg.param("di_scaleFactor", di_scaleFactor, 0.001f, "Scale factor to apply to convert depth images in meters");
  arg.param("sensorType", sensorType, "kinect", "Sensor type: xtion640/xtion320/kinect/kinectFreiburg1/kinectFreiburg2/kinectFreiburg3");

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
  arg.param("al_inlierMaxChi2", al_inlierMaxChi2, 1e3, "Max chi2 error value for the alignment step");

  arg.param("chunkStep", chunkStep, 30, "Reset the merging process every chunkStep images");

  arg.param("benchmarkFilename", benchmarkFilename, "pwn_benchmark.txt", "Specify the name of the file that will contain the results of the benchmark");
  arg.param("absoluteTrajectoryFilename", absoluteTrajectoryFilename, "pwn_absolute_trajectory.txt", "Specify the name of the file that will contain the absolute trajectory computed");
  arg.param("relativeTrajectoryFilename", relativeTrajectoryFilename, "pwn_relative_trajectory.txt", "Specify the name of the file that will contain the relative trajectory computed");
  arg.param("associationsFilename", associationsFilename, "pwn_associations.txt", "Specify the name of the file that will contain the images associations");
  arg.param("groundTruthFilename", groundTruthFilename, "groudtruth.txt", "Specify the name of the file that will contain the ground truth trajectory");
  arg.paramLeftOver("directory", directory, ".", "Directory where the program will find the ground truth file and the subfolders depth and rgb containing the images", true);
  
  // Set parser input
  arg.parseArgs(argc, argv);

  /************************************************************************
   *                   Environment Initialization                         *
   ************************************************************************/
  // Get starting transformation
  PWNPerformancesAnalyzer pwnPerformancesAnalyzer;
  pwnPerformancesAnalyzer.openEvaluationFiles(groundTruthFilename, associationsFilename,
					      absoluteTrajectoryFilename, relativeTrajectoryFilename,
					      benchmarkFilename);

  pwnPerformancesAnalyzer.setSensorType(sensorType);
  pwnPerformancesAnalyzer.setDepthImageSize(pj_imageRows, pj_imageCols);
  pwnPerformancesAnalyzer.setScale(ng_scale); 
  pwnPerformancesAnalyzer.setScaleFactor(di_scaleFactor);
  pwnPerformancesAnalyzer.setWorldRadius(ng_worldRadius);
  pwnPerformancesAnalyzer.setMinImageRadius(ng_minImageRadius);
  pwnPerformancesAnalyzer.setMaxImageRadius(ng_maxImageRadius);
  pwnPerformancesAnalyzer.setMinPoints(ng_minPoints);
  pwnPerformancesAnalyzer.setCurvatureThreshold(ng_curvatureThreshold);
  pwnPerformancesAnalyzer.setInlierDistanceThreshold(cf_inlierDistanceThreshold);
  pwnPerformancesAnalyzer.setFlatCurvatureThreshold(cf_flatCurvatureThreshold);
  pwnPerformancesAnalyzer.setInlierCurvatureRatioThreshold(cf_inlierCurvatureRatioThreshold);
  pwnPerformancesAnalyzer.setInlierNormalAngularThreshold(cf_inlierNormalAngularThreshold);
  pwnPerformancesAnalyzer.setInlierMaxChi2(al_inlierMaxChi2);
  pwnPerformancesAnalyzer.setOuterIterations(al_outerIterations);
  pwnPerformancesAnalyzer.setInnerIterations(al_innerIterations);
  pwnPerformancesAnalyzer.setVerbose(verbose);
  pwnPerformancesAnalyzer.setMaxDistance(pj_maxDistance);
  pwnPerformancesAnalyzer.setChunkStep(chunkStep);

  /************************************************************************
   *                       Sequential Alignment                           *
   ************************************************************************/
  while(pwnPerformancesAnalyzer.loadNextPair()) {
    pwnPerformancesAnalyzer.computeAlignment();
    
    pwnPerformancesAnalyzer.writeBenchmarkStats();
    
    pwnPerformancesAnalyzer.writePose();
  }

  return 0;
}
