#include <QFileDialog>

#include "map_gui.h"
#include "moc_map_gui.cpp"


using namespace g2o;
using namespace std;



MapGUI::MapGUI()
{
    setupUi(this);
    QObject::connect(pushButton, SIGNAL(clicked()), this, SLOT(loadReferenceGraph()));
    QObject::connect(pushButton_2, SIGNAL(clicked()), this, SLOT(clearReferenceGraph()));
    QObject::connect(pushButton_3, SIGNAL(clicked()), this, SLOT(loadCurrentGraph()));
    QObject::connect(pushButton_4, SIGNAL(clicked()), this, SLOT(clearCurrentGraph()));
}


void MapGUI::loadReferenceGraph()
{
    QString selection = QFileDialog::getOpenFileName();
    string filename = selection.toUtf8().constData();
    OptimizableGraph* graph = new OptimizableGraph;
    graph->load(filename.c_str());

    // sort the vertices based on the id
    vector<int> vertexIds(graph->vertices().size());
    int k = 0;

    for(OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it)
    {
        vertexIds[k++] = (it->first);
    }
    sort(vertexIds.begin(), vertexIds.end());

    Vertices drawableVertices;
    Edges drawableEdges;

    for(size_t i = 0; i < vertexIds.size(); ++i)
    {
        HyperGraph::Vertex* v = graph->vertex(vertexIds[i]);
        if(!v)
        {
            continue;
        }

        HyperGraph::EdgeSet edgeSet = v->edges();
        for(HyperGraph::EdgeSet::const_iterator it = edgeSet.begin(); it != edgeSet.end(); ++it)
        {
            HyperGraph::Edge* e = *it;
            drawableEdges.push_back(e);
        }
        drawableVertices.push_back(v);
    }
    viewer->setReferenceGraph(drawableVertices, drawableEdges);
    viewer->updateGL();
}


void MapGUI::clearReferenceGraph()
{
    viewer->cancelReferenceGraph();
    viewer->updateGL();
}


void MapGUI::loadCurrentGraph()
{
    QString selection = QFileDialog::getOpenFileName();
    string filename = selection.toUtf8().constData();
    OptimizableGraph* graph = new OptimizableGraph;
    graph->load(filename.c_str());

    // sort the vertices based on the id
    vector<int> vertexIds(graph->vertices().size());
    int k = 0;

    for(OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it)
    {
        vertexIds[k++] = (it->first);
    }
    sort(vertexIds.begin(), vertexIds.end());

    Vertices drawableVertices;
    Edges drawableEdges;

    for(size_t i = 0; i < vertexIds.size(); ++i)
    {
        HyperGraph::Vertex* v = graph->vertex(vertexIds[i]);
        if(!v)
        {
            continue;
        }

        HyperGraph::EdgeSet edgeSet = v->edges();
        for(HyperGraph::EdgeSet::const_iterator it = edgeSet.begin(); it != edgeSet.end(); ++it)
        {
            HyperGraph::Edge* e = *it;
            drawableEdges.push_back(e);
        }
        drawableVertices.push_back(v);
    }
    viewer->setCurrentGraph(drawableVertices, drawableEdges);
    viewer->updateGL();
}


void MapGUI::clearCurrentGraph()
{
    viewer->cancelCurrentGraph();
    viewer->updateGL();
}
