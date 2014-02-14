#include <QFileDialog>

#include "graph_gui.h"
#include "moc_graph_gui.cpp"


//using namespace g2o;
using namespace std;



GraphGUI::GraphGUI()
{
    setupUi(this);
//    QObject::connect(pushButton, SIGNAL(clicked()), this, SLOT(loadReferenceGraph()));
//    QObject::connect(pushButton_2, SIGNAL(clicked()), this, SLOT(startSLAM()));
//    QObject::connect(pushButton_3, SIGNAL(clicked()), this, SLOT(clearReferenceGraph()));
}


//void GraphGUI::loadReferenceGraph()
//{
//    QString selection = QFileDialog::getOpenFileName();
//    string filename = selection.toUtf8().constData();
//    OptimizableGraph* graph = new OptimizableGraph;
//    graph->load(filename.c_str());

//    // sort the vertices based on the id
//    vector<int> vertexIds(graph->vertices().size());
//    int k = 0;

//    for(OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it)
//    {
//        vertexIds[k++] = (it->first);
//    }
//    sort(vertexIds.begin(), vertexIds.end());

//    for(size_t i = 0; i < vertexIds.size(); ++i)
//    {
//        HyperGraph::Vertex* v = graph->vertex(vertexIds[i]);
//        if(!v)
//        {
//            continue;
//        }

//        HyperGraph::EdgeSet edgeSet = v->edges();
//        for(HyperGraph::EdgeSet::const_iterator it = edgeSet.begin(); it != edgeSet.end(); ++it)
//        {
//            HyperGraph::Edge* e = *it;
//            _edges.push_back(e);
//        }
//        _vertices.push_back(v);
//    }
//    cout << "The graph contains " << _vertices.size() << " nodes" << endl;

//    Vertices pippo(1);
//    pippo.push_back(_vertices[0]);
//    viewer->setReferenceGraph(pippo);
//    viewer->updateGL();
//    usleep(10000);
//}


//void GraphGUI::startSLAM()
//{
//    Vertices pippo;
//    for(uint i = 0; i < _vertices.size(); ++i)
//    {
//        pippo.push_back(_vertices[i]);
//        viewer->setReferenceGraph(pippo);
//        viewer->updateGL();
//        usleep(10000);
//    }
//}


//void GraphGUI::clearReferenceGraph()
//{
//    _vertices.clear();
//    viewer->cancelReferenceGraph();
//    viewer->updateGL();
//    usleep(10000);
//}
