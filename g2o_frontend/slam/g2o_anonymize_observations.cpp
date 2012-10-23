#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o_frontend/data/feature_data.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include <fstream>

using namespace g2o;
using namespace std;


int main(int argc, char** argv) {
  CommandArgs arg;


  std::string outputFilename;
  std::string inputFilename;

  arg.param("o", outputFilename, "", "output file name"); 
  arg.paramLeftOver("input-filename ", inputFilename, "", "graph file to read", true);
  arg.parseArgs(argc, argv);
  OptimizableGraph graph;
  if (!graph.load(inputFilename.c_str())){
    cerr << "Error: cannot load a file from \"" << inputFilename << "\", aborting." << endl;
    return 0;
  }
  HyperGraph::EdgeSet removedEdges;
  HyperGraph::VertexSet removedVertices;
  for (HyperGraph::EdgeSet::iterator it = graph.edges().begin(); it!=graph.edges().end(); it++) {
    HyperGraph::Edge* e = *it;
    EdgeSE2PointXY* edgePointXY = dynamic_cast<EdgeSE2PointXY*>(e);
    if (edgePointXY) {
      VertexSE2* pose    = dynamic_cast<VertexSE2*>(edgePointXY->vertex(0));
      VertexPointXY* landmark = dynamic_cast<VertexPointXY*>(edgePointXY->vertex(1));
      FeaturePointXYData * feature = new FeaturePointXYData();
      feature->setPositionMeasurement(edgePointXY->measurement());
      feature->setPositionInformation(edgePointXY->information());
      pose->addUserData(feature); 
      removedEdges.insert(edgePointXY);
      removedVertices.insert(landmark);
    }
  }
  
  for (HyperGraph::EdgeSet::iterator it = removedEdges.begin(); it!=removedEdges.end(); it++){
    OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(*it);
    graph.removeEdge(e);
  }

  for (HyperGraph::VertexSet::iterator it = removedVertices.begin(); it!=removedVertices.end(); it++){
    OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(*it);
    graph.removeVertex(v);
  }

  
  if (outputFilename.length()){
    graph.save(outputFilename.c_str());
  }
 
}
