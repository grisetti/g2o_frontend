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
  float resolution, max_range, usable_range, angle;
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
  arg.param("angle", angle, 0, "rotate the map of x degrees");
  arg.paramLeftOver("input_graph.g2o", g2oFilename, "", "input g2o graph to use to build the map", false);
  arg.paramLeftOver("output_map", mapFilename, "", "output filename where to save the map (without extension)", false);  
  arg.parseArgs(argc, argv);

  angle = angle*M_PI/180.0;

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
  double xmin=std::numeric_limits<double>::max();
  double xmax=std::numeric_limits<double>::min();
  double ymin=std::numeric_limits<double>::max();
  double ymax=std::numeric_limits<double>::min();

  SE2 baseTransform(0,0,angle);

  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = graph->vertex(vertexIds[i]);
    VertexSE2 *v = dynamic_cast<VertexSE2*>(_v);
    if(!v) { continue; }
    v->setEstimate(baseTransform*v->estimate());
    OptimizableGraph::Data *d = v->userData();

    while(d) {
      RobotLaser *robotLaser = dynamic_cast<RobotLaser*>(d);
      if(!robotLaser) {
	d = d->next();
	continue;
      }      
      robotLasers.push_back(robotLaser);
      robotPoses.push_back(v->estimate());
      double x = v->estimate().translation().x();
      double y = v->estimate().translation().y();
      
      xmax = xmax > x+usable_range ? xmax : x+usable_range;
      ymax = ymax > y+usable_range ? ymax : y+usable_range;
      xmin = xmin < x-usable_range ? xmin : x-usable_range;
      ymin = ymin < y-usable_range ? ymin : y-usable_range;
 
      d = d->next();
    }
  }

  boundingBox(0,0)=xmin;
  boundingBox(0,1)=xmax;
  boundingBox(1,0)=ymin;
  boundingBox(1,1)=ymax;

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
  Eigen::Vector2i size;
  if(rows != 0 && cols != 0) { size = Eigen::Vector2i(rows, cols); }
  else {
    size = Eigen::Vector2i((boundingBox(0, 1) - boundingBox(0, 0))/ resolution,
			   (boundingBox(1, 1) - boundingBox(1, 0))/ resolution);
    } 
  std::cout << "Map size: " << size.transpose() << std::endl;
  if(size.x() == 0 || size.y() == 0) {
    std::cout << "Zero map size ... quitting!" << std::endl;
    return 0;
  }

  

  //Eigen::Vector2f offset(-size.x() * resolution / 2.0f, -size.y() * resolution / 2.0f);
  Eigen::Vector2f offset(boundingBox(0, 0),boundingBox(1, 0));
  FrequencyMapCell unknownCell;
  FrequencyMap map = FrequencyMap(resolution, offset, size, unknownCell);

  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = graph->vertex(vertexIds[i]);
    VertexSE2 *v = dynamic_cast<VertexSE2*>(_v);
    if(!v) { continue; }
    OptimizableGraph::Data *d = v->userData();
    SE2 robotPose = v->estimate();
    
    while(d) {
      RobotLaser *robotLaser = dynamic_cast<RobotLaser*>(d);
      if(!robotLaser) {
	d = d->next();
	continue;
      }      
      map.integrateScan(robotLaser, robotPose, max_range, usable_range, gain, square_size);
      d = d->next();
    }
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
      } else {
	float fraction = map(r, c).hits()/(map(r, c).hits()+map(r, c).misses());
	float val = 255*(1-fraction);
	mapImage.at<unsigned char>(r, c) = (unsigned char)val;
      }
      // else if(map(r, c).hits() > threshold) {
      // 	mapImage.at<unsigned char>(r, c) = 255;
      // }
      // else {
      // 	mapImage.at<unsigned char>(r, c) = 0;
      // }
    }
  }
  cv::imwrite(mapFilename + ".png", mapImage);

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
