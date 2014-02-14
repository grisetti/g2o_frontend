#include <signal.h>

//#include "g2o/core/hyper_dijkstra.h"
//#include "g2o/stuff/macros.h"
//#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
//#include "g2o/core/optimization_algorithm_gauss_newton.h"
//#include "g2o/core/optimization_algorithm_levenberg.h"
//#include "g2o/solvers/csparse/linear_solver_csparse.h"

//#include "g2o/types/slam2d/types_slam2d.h"
//#include "g2o/types/slam2d_addons/types_slam2d_addons.h"

#include "graph_gui.h"
#include "drawable_region.h"
#include "drawable_vertex.h"
#include "drawable_laser.h"
#include "g2o_frontend/mapper/graph_slam/graph_slam.h"


using namespace Eigen;
using namespace g2o;
using namespace std;



volatile bool hasToStop;
void sigquit_handler(int sig)
{
    if(sig == SIGINT)
    {
        hasToStop = 1;
        static int cnt = 0;
        if(cnt++ == 2)
        {
            cerr << __PRETTY_FUNCTION__ << " forcing exit" << endl;
            exit(1);
        }
    }
}


int main(int argc, char**argv)
{
    QApplication app(argc, argv);

    hasToStop = false;
    string filename;
    CommandArgs arg;
    arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
    arg.parseArgs(argc, argv);

    if(filename.size() == 0)
    {
        cerr << "No input data specified" << endl;
        return 0;
    }

    GraphSLAM gslam(filename);
    gslam.initialize(1000);


    // graph construction
    SparseOptimizer* graph = gslam.graph();

    // sort the vertices based on the id
    vector<int> vertexIds(graph->vertices().size());
    int k = 0;

    GraphGUI* gui = new GraphGUI();
    gui->show();


    for(OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it)
    {
        vertexIds[k++] = (it->first);
    }
    sort(vertexIds.begin(), vertexIds.end());


    for(size_t i = 0; i < vertexIds.size(); ++i)
    {
        app.processEvents();
        HyperGraph::Vertex* v = graph->vertex(vertexIds[i]);
        if(!v)
        {
            continue;
        }

        VertexSE2* vse2 = dynamic_cast<VertexSE2*>(v);
        DrawableVertex* dv = new DrawableVertex;
        dv->setVertex(vse2);

        gslam.localMap()->addToSubMap(vse2);
        DrawableRegion* dr = new DrawableRegion;
        dr->setRegion(gslam.localMap()->currentVertices());

//        cout << "dr vertices size: " << dr->region()->size() << endl;

//        cout << "INNER map size: " << gslam.innerMap()->currentVertices()->size() << endl;
//        cout << "OUTER map size: " << gslam.outerMap()->currentVertices()->size() << endl;

        OptimizableGraph::Data* d = vse2->userData();
        LaserRobotData* laserRobotData = dynamic_cast<LaserRobotData*>(d);
        DrawableLaser* dl = new DrawableLaser(laserRobotData);
        dv->setPayload(dl);

        gui->viewer->addDrawable(dv);
//        gui->viewer->addDrawable(dr);

        gui->viewer->updateGL();
        usleep(50000);
    }

    while(1)
    {
        app.processEvents();
        usleep(50000);
    }

    exit(0);
}
