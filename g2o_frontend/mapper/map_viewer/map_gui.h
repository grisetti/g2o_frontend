#ifndef MAPGUI_H_
#define MAPGUI_H_

#include "interface.h"
#include <fstream>
#include <Qt/qapplication.h>

#include "g2o/core/hyper_graph.h"


typedef std::vector<g2o::HyperGraph::Vertex*> Vertices;
typedef std::vector<g2o::HyperGraph::Edge*> Edges;
//typedef std::vector<g2o::VertexSE2*> Vertices;
//typedef std::vector<g2o::EdgeSE2*> Edges;

class MapGUI : public QMainWindow, public Ui::MainWindow
{
    Q_OBJECT

public:
    MapGUI(Vertices* verticesVector_, Edges* edgesVector_, QWidget *parent = 0);

    Vertices* _vertices;
    Edges* _edges;


public slots:
        void loadGraph();
};

#endif // MAPGUI_H_
