#include <iostream> 

#include <opencv2/highgui/highgui.hpp>

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/data/robot_laser.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"

#include "g2o/stuff/command_args.h"

#include "frequency_map.h"

using namespace std;
using namespace Eigen;
using namespace g2o;

int main(int argc, char **argv) {
  /************************************************************************
   *                          Input handling                              *
   ************************************************************************/
  int threshold, rows, cols, gain, square_size;
  float resolution, max_range, usable_range;
  string g2oFilename, mapFilename;
  g2o::CommandArgs arg;
  arg.param("resolution", resolution, 0.05f, "resolution of the map (how much is in meters a pixel)");
  arg.param("threshold", threshold, 20, "threshold to apply to the frequency map (values under the threshold are discarded)");
  arg.param("rows", rows, 0, "impose the resulting map to have this number of rows");
  arg.param("cols", cols, 0, "impose the resulting map to have this number of columns");
  arg.param("max_range", max_range, -1.0f, "max laser range to consider for map building");
  arg.param("usable_range", usable_range, -1.0f, "usable laser range for map building");
  arg.param("gain", gain, 1, "gain to impose to the pixels of the map");
  arg.param("square_size", square_size, 1, "square size of the region where increment the hits");
  arg.paramLeftOver("input_graph.g2o", g2oFilename, "", "input g2o graph to use to build the map", false);
  arg.paramLeftOver("output_map", mapFilename, "", "output filename where to save the map (without extension)", false);  
  arg.parseArgs(argc, argv);

  /************************************************************************
   *                          Loading Graph                               *
   ************************************************************************/
  // Load graph
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  SlamLinearSolver *linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton *solverGauss = new OptimizationAlgorithmGaussNewton(blockSolver);
  SparseOptimizer *graph = new SparseOptimizer();
  graph->setAlgorithm(solverGauss);    
  graph->load(g2oFilename.c_str());
  
  // Sort verteces
  vector<int> vertexIds(graph->vertices().size());
  int k = 0;
  for(OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it) {
    vertexIds[k++] = (it->first);
  }  
  sort(vertexIds.begin(), vertexIds.end());
  
  /************************************************************************
   *                          Compute map size                            *
   ************************************************************************/
  // Check the entire graph to find map bounding box
  Eigen::Matrix2d boundingBox = Eigen::Matrix2d::Zero();
  std::vector<RobotLaser*> robotLasers;
  std::vector<SE2> robotPoses;
  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = graph->vertex(vertexIds[i]);
    VertexSE2 *v = dynamic_cast<VertexSE2*>(_v);
    if(!v) { continue; }
    OptimizableGraph::Data *d = v->userData();
    while(d) {
      RobotLaser *robotLaser = dynamic_cast<RobotLaser*>(d);
      if(!robotLaser) {
	d = d->next();
	continue;
      }      
      robotLasers.push_back(robotLaser);
      robotPoses.push_back(v->estimate());
      Eigen::Vector2f translation = Eigen::Vector2f((float) v->estimate().translation().x(),
						    (float) v->estimate().translation().y());
      if(translation.x() < boundingBox(0, 0)) { boundingBox(0, 0) = translation.x(); }
      if(translation.x() >= boundingBox(0, 1)) { boundingBox(0, 1) = translation.x(); }
      if(translation.y() < boundingBox(1, 0)) { boundingBox(1, 0) = translation.y(); }
      if(translation.y() >= boundingBox(1, 1)) { boundingBox(1, 1) = translation.y(); }
      d = d->next();
    }
  }
  std::cout << "Found " << robotLasers.size() << " laser scans"<< std::endl;
  std::cout << "Bounding box: " << std::endl << boundingBox << std::endl; 
  if(robotLasers.size() == 0)  {
    std::cout << "No laser scans found ... quitting!" << std::endl;
    return 0;
  }

  /************************************************************************
   *                          Compute the map                             *
   ************************************************************************/
  // Create the map
  float maxRange;
  if(max_range < 0.0f) { maxRange = robotLasers[0]->laserParams().maxRange; }
  else { maxRange = max_range; }
  Eigen::Vector2i size;
  if(rows != 0 && cols != 0) { size = Eigen::Vector2i(rows, cols); }
  else {
    size = Eigen::Vector2i((boundingBox(0, 1) - boundingBox(0, 0) + maxRange / 2.0f) / resolution,
			   (boundingBox(1, 1) - boundingBox(1, 0) + maxRange / 2.0f)/ resolution);
    } 
  std::cout << "Map size: " << size.transpose() << std::endl;
  if(size.x() == 0 || size.y() == 0) {
    std::cout << "Zero map size ... quitting!" << std::endl;
    return 0;
  }

  Eigen::Vector2f offset(-size.x() * resolution / 2.0f, -size.y() * resolution / 2.0f);
  FrequencyMapCell unknownCell;
  FrequencyMap map = FrequencyMap(resolution, offset, size, unknownCell);

  // Fill the map with scans
  for(size_t i = 0; i < robotLasers.size(); i++) {
    RobotLaser *robotLaser = robotLasers[i];
    SE2 robotPose = robotPoses[i];
    map.integrateScan(robotLaser, robotPose, max_range, usable_range, gain, square_size);
  }

  /************************************************************************
   *                          Save map image                              *
   ************************************************************************/
  cv::Mat mapImage(map.rows(), map.cols(), CV_8UC1);
  mapImage.setTo(cv::Scalar(0));
  for(int c = 0; c < map.cols(); c++) {
    for(int r = 0; r < map.rows(); r++) {
      if(map(r, c).misses() == 0 && map(r, c).hits() == 0) {
	mapImage.at<unsigned char>(r, c) = 127;
      } 
      else if(map(r, c).hits() > threshold) {
	mapImage.at<unsigned char>(r, c) = 255;
      }
      else {
	mapImage.at<unsigned char>(r, c) = 0;
      }
    }
  }
  cv::imwrite(mapFilename + ".png", cv::Scalar(255) - mapImage);

  /************************************************************************
   *                          Write yaml file                             *
   ************************************************************************/
  std::ofstream ofs(string(mapFilename + ".yaml").c_str());
  Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
  ofs << "image: " << mapFilename << ".png" << std::endl
      << "resolution: " << resolution << std::endl
      << "origin: [" << origin.x() << ", " << origin.y() << ", " << origin.z() << "]" << std::endl
      << "negate: 0" << std::endl
      << "occupied_thresh: " << 0.65f << std::endl
      << "free_thresh: " << 0.2f << std::endl;
  return 0;
}
