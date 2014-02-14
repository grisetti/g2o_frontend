#ifndef GRAPHGUI_H
#define GRAPHGUI_H

#include "interface.h"
#include <fstream>
#include <Qt/qapplication.h>

//#include "g2o/core/optimizable_graph.h"

//typedef std::vector<g2o::HyperGraph::Vertex*> Vertices;
//typedef std::vector<g2o::HyperGraph::Edge*> Edges;

class GraphGUI : public QMainWindow, public Ui::MainWindow
{
    Q_OBJECT

public:
    GraphGUI();

//    Vertices _vertices;
//    Edges _edges;

//public slots:
//        void loadReferenceGraph();
//        void startSLAM();
//        void clearReferenceGraph();
};

#endif // GRAPHGUI_H
